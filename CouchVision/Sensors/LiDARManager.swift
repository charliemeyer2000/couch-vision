import ARKit
import Combine
import Foundation
import UIKit

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

    private let transformSubject = PassthroughSubject<TimestampedData<TransformStamped>, Never>()
    public var transformPublisher: AnyPublisher<TimestampedData<TransformStamped>, Never> {
        transformSubject.eraseToAnyPublisher()
    }

    private let cameraFrameSubject = PassthroughSubject<TimestampedData<CameraFrame>, Never>()
    public var cameraFramePublisher: AnyPublisher<TimestampedData<CameraFrame>, Never> {
        cameraFrameSubject.eraseToAnyPublisher()
    }

    private let cameraInfoSubject = PassthroughSubject<TimestampedData<CameraInfo>, Never>()
    public var cameraInfoPublisher: AnyPublisher<TimestampedData<CameraInfo>, Never> {
        cameraInfoSubject.eraseToAnyPublisher()
    }

    public var config = LiDARConfig()
    public var jpegQuality: CGFloat = 0.8

    private var arSession: ARSession?
    private let encodingQueue = DispatchQueue(label: "com.couchvision.lidar.encoding")
    private var isEncoding = false
    private let ciContext = CIContext()
    private let _framePrefixLock = NSLock()
    private var _framePrefix = "iphone"
    public var framePrefix: String {
        get { _framePrefixLock.withLock { _framePrefix } }
        set { _framePrefixLock.withLock { _framePrefix = newValue } }
    }

    private var frameId: String { "\(framePrefix)_lidar" }

    public let sensorId = "lidar"
    public let displayName = "LiDAR"

    override public init() {
        super.init()
        checkAvailability()
    }

    deinit {
        arSession?.delegate = nil
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
        switch state {
        case .ready:
            break
        case .error:
            stop()
        default:
            throw LiDARError.notAvailable
        }

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
        state = .ready
        arSession?.delegate = nil
        arSession?.pause()
        arSession = nil
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

        let baseLink = "\(framePrefix)_base_link"
        let transformStamped = TransformStamped(
            header: ROSHeader(timeInterval: timestamp, frameId: "world"),
            childFrameId: baseLink,
            transform: simdToROSTransform(frame.camera.transform)
        )
        transformSubject.send(TimestampedData(data: transformStamped, timestamp: timestamp, frameId: "world"))

        // Static identity transforms so rviz2 can resolve sensor frame_ids
        let lidarTf = TransformStamped(
            header: ROSHeader(timeInterval: timestamp, frameId: baseLink),
            childFrameId: frameId,
            transform: .identity
        )
        let cameraTf = TransformStamped(
            header: ROSHeader(timeInterval: timestamp, frameId: baseLink),
            childFrameId: "\(framePrefix)_camera_arkit",
            transform: .identity
        )
        transformSubject.send(TimestampedData(data: lidarTf, timestamp: timestamp, frameId: baseLink))
        transformSubject.send(TimestampedData(data: cameraTf, timestamp: timestamp, frameId: baseLink))

        publishCameraFrame(from: frame, timestamp: timestamp)
    }

    private func publishCameraFrame(from frame: ARFrame, timestamp: TimeInterval) {
        guard !isEncoding else { return }
        isEncoding = true

        let pixelBuffer = deepCopyPixelBuffer(frame.capturedImage)
        let intrinsics = frame.camera.intrinsics
        let imageRes = frame.camera.imageResolution
        let quality = jpegQuality
        let prefix = framePrefix
        let cameraFrameId = "\(prefix)_camera_arkit"

        encodingQueue.async { [weak self] in
            defer { self?.isEncoding = false }
            guard let self, let pixelBuffer else { return }

            let ciImage = CIImage(cvPixelBuffer: pixelBuffer)
            guard let cgImage = ciContext.createCGImage(ciImage, from: ciImage.extent),
                  let jpegData = UIImage(cgImage: cgImage).jpegData(compressionQuality: quality) else { return }

            let width = Int(imageRes.width)
            let height = Int(imageRes.height)
            let cameraIntrinsics = CameraIntrinsics(
                fx: intrinsics.columns.0.x,
                fy: intrinsics.columns.1.y,
                cx: intrinsics.columns.2.x,
                cy: intrinsics.columns.2.y
            )

            let cameraFrame = CameraFrame(
                cameraId: "arkit",
                jpegData: jpegData,
                width: width,
                height: height,
                intrinsics: cameraIntrinsics
            )
            cameraFrameSubject.send(TimestampedData(data: cameraFrame, timestamp: timestamp, frameId: cameraFrameId))

            let header = ROSHeader(timeInterval: timestamp, frameId: cameraFrameId)
            let cameraInfo = CameraInfo(
                header: header,
                height: UInt32(height),
                width: UInt32(width),
                distortionModel: "plumb_bob",
                d: [0, 0, 0, 0, 0],
                k: cameraIntrinsics.kMatrix,
                r: [1, 0, 0, 0, 1, 0, 0, 0, 1],
                p: cameraIntrinsics.pMatrix
            )
            cameraInfoSubject.send(TimestampedData(data: cameraInfo, timestamp: timestamp, frameId: cameraFrameId))
        }
    }

    private func deepCopyPixelBuffer(_ source: CVPixelBuffer) -> CVPixelBuffer? {
        let width = CVPixelBufferGetWidth(source)
        let height = CVPixelBufferGetHeight(source)
        let format = CVPixelBufferGetPixelFormatType(source)

        var copy: CVPixelBuffer?
        CVPixelBufferCreate(nil, width, height, format, nil, &copy)
        guard let copy else { return nil }

        CVPixelBufferLockBaseAddress(source, .readOnly)
        CVPixelBufferLockBaseAddress(copy, [])
        defer {
            CVPixelBufferUnlockBaseAddress(copy, [])
            CVPixelBufferUnlockBaseAddress(source, .readOnly)
        }

        let planeCount = CVPixelBufferGetPlaneCount(source)
        if planeCount > 0 {
            for plane in 0 ..< planeCount {
                guard let dest = CVPixelBufferGetBaseAddressOfPlane(copy, plane),
                      let src = CVPixelBufferGetBaseAddressOfPlane(source, plane) else { continue }
                let h = CVPixelBufferGetHeightOfPlane(source, plane)
                let bytesPerRow = CVPixelBufferGetBytesPerRowOfPlane(source, plane)
                memcpy(dest, src, h * bytesPerRow)
            }
        } else {
            guard let dest = CVPixelBufferGetBaseAddress(copy),
                  let src = CVPixelBufferGetBaseAddress(source) else { return copy }
            let bytesPerRow = CVPixelBufferGetBytesPerRow(source)
            memcpy(dest, src, height * bytesPerRow)
        }
        return copy
    }

    private func simdToROSTransform(_ m: simd_float4x4) -> Transform {
        let translation = Vector3(
            x: Double(m.columns.3.x),
            y: Double(m.columns.3.y),
            z: Double(m.columns.3.z)
        )
        let quat = simd_quatf(m)
        let rotation = Quaternion(
            x: Double(quat.imag.x),
            y: Double(quat.imag.y),
            z: Double(quat.imag.z),
            w: Double(quat.real)
        )

        return Transform(translation: translation, rotation: rotation)
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
        Log.sensor.error("ARSession error: \(error.localizedDescription)")
        stop()
    }

    public func sessionWasInterrupted(_ session: ARSession) {
        Log.sensor.info("ARSession interrupted")
    }

    public func sessionInterruptionEnded(_ session: ARSession) {
        Log.sensor.info("ARSession interruption ended")
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
