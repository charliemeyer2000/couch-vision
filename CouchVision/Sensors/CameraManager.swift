import AVFoundation
import Combine
import Foundation
import UIKit

public struct CameraFrame {
    public let cameraId: String
    public let jpegData: Data
    public let width: Int
    public let height: Int
    public let intrinsics: CameraIntrinsics?

    public init(cameraId: String, jpegData: Data, width: Int, height: Int, intrinsics: CameraIntrinsics? = nil) {
        self.cameraId = cameraId
        self.jpegData = jpegData
        self.width = width
        self.height = height
        self.intrinsics = intrinsics
    }
}

public struct CameraIntrinsics {
    public let fx: Float
    public let fy: Float
    public let cx: Float
    public let cy: Float

    public var kMatrix: [Double] {
        [Double(fx), 0, Double(cx), 0, Double(fy), Double(cy), 0, 0, 1]
    }

    public var pMatrix: [Double] {
        [Double(fx), 0, Double(cx), 0, 0, Double(fy), Double(cy), 0, 0, 0, 1, 0]
    }
}

public struct CameraConfig {
    public enum Resolution: String, CaseIterable {
        case p1080
        case p720
        case p480

        var preset: AVCaptureSession.Preset {
            switch self {
            case .p1080: .hd1920x1080
            case .p720: .hd1280x720
            case .p480: .vga640x480
            }
        }
    }

    public let resolution: Resolution
    public let frameRate: Int
    public let jpegQuality: CGFloat

    public init(resolution: Resolution = .p720, frameRate: Int = 30, jpegQuality: CGFloat = 0.8) {
        self.resolution = resolution
        self.frameRate = frameRate
        self.jpegQuality = jpegQuality
    }
}

public enum CameraType: String, CaseIterable {
    case backWide = "back_wide"
    case backUltraWide = "back_ultrawide"
    case backTelephoto = "back_telephoto"
    case front

    var deviceType: AVCaptureDevice.DeviceType {
        switch self {
        case .backWide,
             .front: .builtInWideAngleCamera
        case .backUltraWide: .builtInUltraWideCamera
        case .backTelephoto: .builtInTelephotoCamera
        }
    }

    var position: AVCaptureDevice.Position {
        self == .front ? .front : .back
    }

    func frameId(prefix: String) -> String {
        "\(prefix)_camera_\(rawValue)"
    }
}

public final class CameraManager: NSObject, ObservableObject {

    @Published public private(set) var state: SensorState = .unknown
    @Published public private(set) var activeCamera: CameraType?
    @Published public var isEnabled: Bool = false
    private let _framePrefixLock = NSLock()
    private var _framePrefix = "iphone"
    public var framePrefix: String {
        get { _framePrefixLock.withLock { _framePrefix } }
        set { _framePrefixLock.withLock { _framePrefix = newValue } }
    }

    private let dataSubject = PassthroughSubject<TimestampedData<CameraFrame>, Never>()
    public var dataPublisher: AnyPublisher<TimestampedData<CameraFrame>, Never> {
        dataSubject.eraseToAnyPublisher()
    }

    private let cameraInfoSubject = PassthroughSubject<TimestampedData<CameraInfo>, Never>()
    public var cameraInfoPublisher: AnyPublisher<TimestampedData<CameraInfo>, Never> {
        cameraInfoSubject.eraseToAnyPublisher()
    }

    public var config = CameraConfig() {
        didSet { if state == .running { reconfigureSession() } }
    }

    public var selectedCamera: CameraType = .backWide {
        didSet { if state == .running, oldValue != selectedCamera { switchCamera(to: selectedCamera) } }
    }

    private var captureSession: AVCaptureSession?
    private var videoOutput: AVCaptureVideoDataOutput?
    private var currentDevice: AVCaptureDevice?
    private let sessionQueue = DispatchQueue(label: "com.couchvision.camera.session")
    private let outputQueue = DispatchQueue(label: "com.couchvision.camera.output")

    public let sensorId = "camera"
    public let displayName = "Camera"

    override public init() {
        super.init()
        checkAvailability()
    }

    @discardableResult
    public func checkAvailability() -> Bool {
        guard AVCaptureDevice.default(for: .video) != nil else {
            state = .unavailable
            return false
        }
        updateAuthorizationState()
        return true
    }

    private func updateAuthorizationState() {
        switch AVCaptureDevice.authorizationStatus(for: .video) {
        case .authorized: state = .ready
        case .notDetermined,
             .denied,
             .restricted: state = .unauthorized
        @unknown default: state = .unknown
        }
    }

    public func requestPermissions() async -> Bool {
        let status = AVCaptureDevice.authorizationStatus(for: .video)
        if status == .authorized {
            await MainActor.run { state = .ready }
            return true
        }
        if status == .notDetermined {
            let granted = await AVCaptureDevice.requestAccess(for: .video)
            await MainActor.run { state = granted ? .ready : .unauthorized }
            return granted
        }
        return false
    }

    public func start() throws {
        guard state == .ready || state == .unknown else {
            throw state == .unauthorized ? CameraError.notAuthorized : CameraError.notAvailable
        }
        sessionQueue.async { [weak self] in self?.setupCaptureSession() }
    }

    public func stop() {
        state = .ready
        activeCamera = nil
        sessionQueue.async { [weak self] in
            guard let self else { return }
            if let session = captureSession, session.isRunning {
                session.stopRunning()
            }
            captureSession = nil
            videoOutput = nil
            currentDevice = nil
        }
    }

    private func setupCaptureSession() {
        let session = AVCaptureSession()
        session.beginConfiguration()
        session.sessionPreset = config.resolution.preset

        guard let device = getDevice(for: selectedCamera),
              let input = try? AVCaptureDeviceInput(device: device),
              session.canAddInput(input) else {
            DispatchQueue.main.async { [weak self] in
                self?.state = .error("Failed to create camera input")
            }
            return
        }

        session.addInput(input)
        currentDevice = device
        configureFrameRate(device: device)

        let output = AVCaptureVideoDataOutput()
        output.videoSettings = [kCVPixelBufferPixelFormatTypeKey as String: kCVPixelFormatType_32BGRA]
        output.setSampleBufferDelegate(self, queue: outputQueue)
        output.alwaysDiscardsLateVideoFrames = true

        if session.canAddOutput(output) {
            session.addOutput(output)
            videoOutput = output
        }

        session.commitConfiguration()
        session.startRunning()
        captureSession = session

        DispatchQueue.main.async { [weak self] in
            self?.state = .running
            self?.activeCamera = self?.selectedCamera
        }
    }

    private func getDevice(for cameraType: CameraType) -> AVCaptureDevice? {
        AVCaptureDevice.DiscoverySession(
            deviceTypes: [cameraType.deviceType],
            mediaType: .video,
            position: cameraType.position
        ).devices.first
    }

    private func configureFrameRate(device: AVCaptureDevice) {
        guard (try? device.lockForConfiguration()) != nil else { return }
        defer { device.unlockForConfiguration() }

        let targetRate = Double(config.frameRate)
        for format in device.formats {
            for range in format.videoSupportedFrameRateRanges where range.maxFrameRate >= targetRate {
                device.activeFormat = format
                device.activeVideoMinFrameDuration = CMTime(value: 1, timescale: CMTimeScale(config.frameRate))
                device.activeVideoMaxFrameDuration = CMTime(value: 1, timescale: CMTimeScale(config.frameRate))
                return
            }
        }
    }

    private func switchCamera(to cameraType: CameraType) {
        sessionQueue.async { [weak self] in
            guard let self, let session = captureSession else { return }
            session.beginConfiguration()

            if let currentInput = session.inputs.first as? AVCaptureDeviceInput {
                session.removeInput(currentInput)
            }

            guard let device = getDevice(for: cameraType),
                  let input = try? AVCaptureDeviceInput(device: device),
                  session.canAddInput(input) else {
                session.commitConfiguration()
                return
            }

            session.addInput(input)
            currentDevice = device
            configureFrameRate(device: device)
            session.commitConfiguration()

            DispatchQueue.main.async { self.activeCamera = cameraType }
        }
    }

    private func reconfigureSession() {
        sessionQueue.async { [weak self] in
            guard let self, let session = captureSession else { return }
            session.beginConfiguration()
            session.sessionPreset = config.resolution.preset
            if let device = currentDevice { configureFrameRate(device: device) }
            session.commitConfiguration()
        }
    }

    private func getIntrinsics(from sampleBuffer: CMSampleBuffer) -> CameraIntrinsics? {
        guard let pixelBuffer = CMSampleBufferGetImageBuffer(sampleBuffer) else { return nil }
        let width = CVPixelBufferGetWidth(pixelBuffer)
        let height = CVPixelBufferGetHeight(pixelBuffer)
        // Approximate - real intrinsics require calibration
        let focal = Float(max(width, height)) * 1.2
        return CameraIntrinsics(fx: focal, fy: focal, cx: Float(width) / 2, cy: Float(height) / 2)
    }
}

extension CameraManager: AVCaptureVideoDataOutputSampleBufferDelegate {

    public func captureOutput(_ output: AVCaptureOutput, didOutput sampleBuffer: CMSampleBuffer, from connection: AVCaptureConnection) {
        guard let pixelBuffer = CMSampleBufferGetImageBuffer(sampleBuffer) else { return }

        let timestamp = TimeUtils.toUnixTimestamp(CMTimeGetSeconds(CMSampleBufferGetPresentationTimeStamp(sampleBuffer)))
        let width = CVPixelBufferGetWidth(pixelBuffer)
        let height = CVPixelBufferGetHeight(pixelBuffer)
        let frameId = activeCamera?.frameId(prefix: framePrefix) ?? "\(framePrefix)_camera"

        let ciImage = CIImage(cvPixelBuffer: pixelBuffer)
        guard let cgImage = CIContext().createCGImage(ciImage, from: ciImage.extent),
              let jpegData = UIImage(cgImage: cgImage).jpegData(compressionQuality: config.jpegQuality) else { return }

        let intrinsics = getIntrinsics(from: sampleBuffer)
        let frame = CameraFrame(
            cameraId: activeCamera?.rawValue ?? "unknown",
            jpegData: jpegData,
            width: width,
            height: height,
            intrinsics: intrinsics
        )

        dataSubject.send(TimestampedData(data: frame, timestamp: timestamp, frameId: frameId))

        if let intrinsics {
            let header = ROSHeader(timeInterval: timestamp, frameId: frameId)
            let cameraInfo = CameraInfo(
                header: header,
                height: UInt32(height),
                width: UInt32(width),
                distortionModel: "plumb_bob",
                d: [0, 0, 0, 0, 0],
                k: intrinsics.kMatrix,
                r: [1, 0, 0, 0, 1, 0, 0, 0, 1],
                p: intrinsics.pMatrix
            )
            cameraInfoSubject.send(TimestampedData(data: cameraInfo, timestamp: timestamp, frameId: frameId))
        }
    }

    public func captureOutput(_ output: AVCaptureOutput, didDrop sampleBuffer: CMSampleBuffer, from connection: AVCaptureConnection) {
        // Dropped frames are expected under thermal throttling
    }
}

public enum CameraError: Error, LocalizedError {
    case notAvailable
    case notAuthorized
    case configurationFailed(String)

    public var errorDescription: String? {
        switch self {
        case .notAvailable: "Camera is not available on this device"
        case .notAuthorized: "Camera access not authorized"
        case .configurationFailed(let reason): "Camera configuration failed: \(reason)"
        }
    }
}
