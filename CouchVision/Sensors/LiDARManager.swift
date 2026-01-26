import ARKit
import Combine
import Foundation

public struct LiDARData {
    public let depthImage: ROSImage?
    public let confidenceImage: ROSImage?
    public let pointCloud: PointCloud2?
    public let cameraTransform: simd_float4x4?

    public init(
        depthImage: ROSImage? = nil,
        confidenceImage: ROSImage? = nil,
        pointCloud: PointCloud2? = nil,
        cameraTransform: simd_float4x4? = nil
    ) {
        self.depthImage = depthImage
        self.confidenceImage = confidenceImage
        self.pointCloud = pointCloud
        self.cameraTransform = cameraTransform
    }
}

public struct LiDARConfig {
    public let generatePointCloud: Bool
    public let publishConfidence: Bool
    public let pointCloudDownsample: Int
    public let maxDepth: Float

    public init(
        generatePointCloud: Bool = true,
        publishConfidence: Bool = false,
        pointCloudDownsample: Int = 4,
        maxDepth: Float = 5.0
    ) {
        self.generatePointCloud = generatePointCloud
        self.publishConfidence = publishConfidence
        self.pointCloudDownsample = max(1, pointCloudDownsample)
        self.maxDepth = maxDepth
    }
}

public final class LiDARManager: NSObject, ObservableObject {

    @Published public private(set) var state: SensorState = .unknown
    @Published public var isEnabled: Bool = false

    private let dataSubject = PassthroughSubject<TimestampedData<LiDARData>, Never>()
    public var dataPublisher: AnyPublisher<TimestampedData<LiDARData>, Never> {
        dataSubject.eraseToAnyPublisher()
    }

    public var config = LiDARConfig()

    private var arSession: ARSession?
    private let frameId = "iphone_lidar"

    public let sensorId = "lidar"
    public let displayName = "LiDAR"

    override public init() {
        super.init()
        checkAvailability()
    }

    deinit {
        arSession?.pause()
    }

    @discardableResult
    public func checkAvailability() -> Bool {
        let isSupported = ARWorldTrackingConfiguration.supportsFrameSemantics(.sceneDepth)
        state = isSupported ? .ready : .unavailable
        return isSupported
    }

    public func requestPermissions() async -> Bool {
        // ARKit uses camera permission, which should be requested separately
        checkAvailability()
    }

    public func start() throws {
        guard state == .ready else { throw LiDARError.notAvailable }

        let session = ARSession()
        session.delegate = self

        let configuration = ARWorldTrackingConfiguration()
        if ARWorldTrackingConfiguration.supportsFrameSemantics(.sceneDepth) {
            configuration.frameSemantics.insert(.sceneDepth)
        }
        if ARWorldTrackingConfiguration.supportsFrameSemantics(.smoothedSceneDepth) {
            configuration.frameSemantics.insert(.smoothedSceneDepth)
        }

        session.run(configuration)
        arSession = session
        DispatchQueue.main.async { [weak self] in self?.state = .running }
    }

    public func stop() {
        arSession?.pause()
        arSession = nil
        DispatchQueue.main.async { [weak self] in self?.state = .ready }
    }

    private func processFrame(_ frame: ARFrame) {
        let timestamp = TimeUtils.toUnixTimestamp(frame.timestamp)

        guard let sceneDepth = frame.smoothedSceneDepth ?? frame.sceneDepth else { return }

        let depthImage = pixelBufferToROSImage(sceneDepth.depthMap, encoding: "32FC1", timestamp: timestamp)

        var confidenceImage: ROSImage?
        if config.publishConfidence, let confMap = sceneDepth.confidenceMap {
            confidenceImage = pixelBufferToROSImage(confMap, encoding: "mono8", timestamp: timestamp)
        }

        var pointCloud: PointCloud2?
        if config.generatePointCloud {
            pointCloud = createPointCloud(
                from: sceneDepth.depthMap,
                confidence: sceneDepth.confidenceMap,
                camera: frame.camera,
                timestamp: timestamp
            )
        }

        let data = LiDARData(
            depthImage: depthImage,
            confidenceImage: confidenceImage,
            pointCloud: pointCloud,
            cameraTransform: frame.camera.transform
        )
        dataSubject.send(TimestampedData(data: data, timestamp: timestamp, frameId: frameId))
    }

    private func pixelBufferToROSImage(_ buffer: CVPixelBuffer, encoding: String, timestamp: TimeInterval) -> ROSImage {
        CVPixelBufferLockBaseAddress(buffer, .readOnly)
        defer { CVPixelBufferUnlockBaseAddress(buffer, .readOnly) }

        let width = CVPixelBufferGetWidth(buffer)
        let height = CVPixelBufferGetHeight(buffer)
        let bytesPerRow = CVPixelBufferGetBytesPerRow(buffer)

        guard let baseAddress = CVPixelBufferGetBaseAddress(buffer) else {
            return ROSImage(
                header: ROSHeader(timeInterval: timestamp, frameId: frameId),
                height: 0,
                width: 0,
                encoding: encoding,
                step: 0,
                data: Data()
            )
        }

        let data = Data(bytes: baseAddress, count: height * bytesPerRow)
        return ROSImage(
            header: ROSHeader(timeInterval: timestamp, frameId: frameId),
            height: UInt32(height),
            width: UInt32(width),
            encoding: encoding,
            isBigEndian: 0,
            step: UInt32(bytesPerRow),
            data: data
        )
    }

    private func createPointCloud(
        from depthMap: CVPixelBuffer,
        confidence: CVPixelBuffer?,
        camera: ARCamera,
        timestamp: TimeInterval
    ) -> PointCloud2 {
        CVPixelBufferLockBaseAddress(depthMap, .readOnly)
        defer { CVPixelBufferUnlockBaseAddress(depthMap, .readOnly) }

        if let conf = confidence { CVPixelBufferLockBaseAddress(conf, .readOnly) }
        defer { if let conf = confidence { CVPixelBufferUnlockBaseAddress(conf, .readOnly) } }

        let width = CVPixelBufferGetWidth(depthMap)
        let height = CVPixelBufferGetHeight(depthMap)

        guard let depthData = CVPixelBufferGetBaseAddress(depthMap)?.assumingMemoryBound(to: Float32.self) else {
            return PointCloud2.xyzCloud(points: [], frameId: frameId, timestamp: timestamp)
        }

        let confData = confidence.flatMap { CVPixelBufferGetBaseAddress($0)?.assumingMemoryBound(to: UInt8.self) }

        let intrinsics = camera.intrinsics
        let fx = intrinsics[0, 0], fy = intrinsics[1, 1]
        let cx = intrinsics[2, 0], cy = intrinsics[2, 1]

        let imageRes = camera.imageResolution
        let scaleX = Float(width) / Float(imageRes.width)
        let scaleY = Float(height) / Float(imageRes.height)

        var points: [(x: Float, y: Float, z: Float, intensity: Float)] = []
        points.reserveCapacity(width * height / (config.pointCloudDownsample * config.pointCloudDownsample))

        let downsample = config.pointCloudDownsample
        for y in stride(from: 0, to: height, by: downsample) {
            for x in stride(from: 0, to: width, by: downsample) {
                let index = y * width + x
                let depth = depthData[index]

                guard depth > 0, depth < config.maxDepth else { continue }

                var conf: Float = 1.0
                if let confData {
                    let confValue = confData[index]
                    if confValue == 0 { continue }
                    conf = Float(confValue) / 2.0
                }

                let u = Float(x) / scaleX
                let v = Float(y) / scaleY
                let ptX = (u - cx) * depth / fx
                let ptY = (v - cy) * depth / fy

                // ARKit: (right, up, back) -> ROS: (forward, left, up)
                points.append((x: depth, y: -ptX, z: -ptY, intensity: conf))
            }
        }

        return PointCloud2.xyziCloud(points: points, frameId: frameId, timestamp: timestamp)
    }
}

extension LiDARManager: ARSessionDelegate {
    public func session(_ session: ARSession, didUpdate frame: ARFrame) {
        processFrame(frame)
    }

    public func session(_ session: ARSession, didFailWithError error: Error) {
        DispatchQueue.main.async { [weak self] in self?.state = .error(error.localizedDescription) }
    }

    public func sessionWasInterrupted(_ session: ARSession) {
        DispatchQueue.main.async { [weak self] in self?.state = .error("Session interrupted") }
    }

    public func sessionInterruptionEnded(_ session: ARSession) {
        if case .error("Session interrupted") = state { try? start() }
    }
}

public enum LiDARError: Error, LocalizedError {
    case notAvailable
    case sessionFailed(String)

    public var errorDescription: String? {
        switch self {
        case .notAvailable: "LiDAR not available (requires iPhone 12 Pro or later)"
        case .sessionFailed(let reason): "ARKit session failed: \(reason)"
        }
    }
}
