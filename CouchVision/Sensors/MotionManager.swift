import Combine
@preconcurrency import CoreMotion
import Foundation

public struct MotionConfig {
    public let imuRate: SensorRateConfig
    public let publishRawAccelerometer: Bool
    public let publishRawGyroscope: Bool

    public init(
        imuRate: SensorRateConfig = .hz100,
        publishRawAccelerometer: Bool = false,
        publishRawGyroscope: Bool = false
    ) {
        self.imuRate = imuRate
        self.publishRawAccelerometer = publishRawAccelerometer
        self.publishRawGyroscope = publishRawGyroscope
    }
}

public final class MotionManager: ObservableObject {

    @Published public private(set) var state: SensorState = .unknown
    @Published public var isEnabled: Bool = false

    private let imuSubject = PassthroughSubject<TimestampedData<ImuMessage>, Never>()
    public var imuPublisher: AnyPublisher<TimestampedData<ImuMessage>, Never> {
        imuSubject.eraseToAnyPublisher()
    }

    private let accelerometerSubject = PassthroughSubject<TimestampedData<Vector3Stamped>, Never>()
    public var accelerometerPublisher: AnyPublisher<TimestampedData<Vector3Stamped>, Never> {
        accelerometerSubject.eraseToAnyPublisher()
    }

    private let gyroscopeSubject = PassthroughSubject<TimestampedData<Vector3Stamped>, Never>()
    public var gyroscopePublisher: AnyPublisher<TimestampedData<Vector3Stamped>, Never> {
        gyroscopeSubject.eraseToAnyPublisher()
    }

    public var config = MotionConfig() {
        didSet { if state == .running { reconfigure() } }
    }

    private let motionManager = CMMotionManager()
    private let operationQueue: OperationQueue
    private let frameId = "iphone_imu"

    // Estimated covariance values for iPhone sensors
    private let accelCovariance: Covariance3x3 = [0.01, 0, 0, 0, 0.01, 0, 0, 0, 0.01]
    private let gyroCovariance: Covariance3x3 = [0.0001, 0, 0, 0, 0.0001, 0, 0, 0, 0.0001]
    private let orientationCovariance: Covariance3x3 = [0.001, 0, 0, 0, 0.001, 0, 0, 0, 0.001]

    public let sensorId = "motion"
    public let displayName = "Motion (IMU)"

    public init() {
        operationQueue = OperationQueue()
        operationQueue.name = "com.couchvision.motion"
        operationQueue.maxConcurrentOperationCount = 1
        checkAvailability()
    }

    @discardableResult
    public func checkAvailability() -> Bool {
        let available = motionManager.isDeviceMotionAvailable
        state = available ? .ready : .unavailable
        return available
    }

    public func requestPermissions() async -> Bool {
        guard motionManager.isDeviceMotionAvailable else {
            await MainActor.run { state = .unavailable }
            return false
        }

        // CoreMotion prompts on first use - briefly start to trigger prompt
        let granted = await withCheckedContinuation { continuation in
            motionManager.startDeviceMotionUpdates(to: operationQueue) { [weak self] motion, _ in
                self?.motionManager.stopDeviceMotionUpdates()
                continuation.resume(returning: motion != nil)
            }

            // Timeout after 2 seconds - assume granted if no callback
            let manager = motionManager
            DispatchQueue.global().asyncAfter(deadline: .now() + 2.0) {
                manager.stopDeviceMotionUpdates()
            }
        }

        await MainActor.run { state = granted ? .ready : .unauthorized }
        return granted
    }

    public func start() throws {
        guard motionManager.isDeviceMotionAvailable else { throw MotionError.notAvailable }

        motionManager.deviceMotionUpdateInterval = config.imuRate.interval
        motionManager.startDeviceMotionUpdates(using: .xArbitraryCorrectedZVertical, to: operationQueue) { [weak self] motion, _ in
            guard let self, let motion else { return }
            processDeviceMotion(motion)
        }

        if config.publishRawAccelerometer {
            motionManager.accelerometerUpdateInterval = config.imuRate.interval
            motionManager.startAccelerometerUpdates(to: operationQueue) { [weak self] data, _ in
                guard let self, let data else { return }
                processAccelerometer(data)
            }
        }

        if config.publishRawGyroscope {
            motionManager.gyroUpdateInterval = config.imuRate.interval
            motionManager.startGyroUpdates(to: operationQueue) { [weak self] data, _ in
                guard let self, let data else { return }
                processGyroscope(data)
            }
        }

        DispatchQueue.main.async { [weak self] in self?.state = .running }
    }

    public func stop() {
        state = .ready
        motionManager.stopDeviceMotionUpdates()
        motionManager.stopAccelerometerUpdates()
        motionManager.stopGyroUpdates()
    }

    private func reconfigure() {
        stop()
        try? start()
    }

    private func processDeviceMotion(_ motion: CMDeviceMotion) {
        let timestamp = TimeUtils.toUnixTimestamp(motion.timestamp)

        let attitude = motion.attitude.quaternion
        let orientation = Quaternion(x: attitude.x, y: attitude.y, z: attitude.z, w: attitude.w)

        let rate = motion.rotationRate
        let angularVelocity = Vector3(x: rate.x, y: rate.y, z: rate.z)

        // Combine user acceleration + gravity, convert from G's to m/s^2
        let g = 9.80665
        let gravity = motion.gravity
        let userAccel = motion.userAcceleration
        let linearAcceleration = Vector3(
            x: (userAccel.x + gravity.x) * g,
            y: (userAccel.y + gravity.y) * g,
            z: (userAccel.z + gravity.z) * g
        )

        let header = ROSHeader(timeInterval: timestamp, frameId: frameId)
        let msg = ImuMessage(
            header: header,
            orientation: orientation, orientationCovariance: orientationCovariance,
            angularVelocity: angularVelocity, angularVelocityCovariance: gyroCovariance,
            linearAcceleration: linearAcceleration, linearAccelerationCovariance: accelCovariance
        )

        imuSubject.send(TimestampedData(data: msg, timestamp: timestamp, frameId: frameId))
    }

    private func processAccelerometer(_ data: CMAccelerometerData) {
        let timestamp = TimeUtils.toUnixTimestamp(data.timestamp)
        let g = 9.80665
        let header = ROSHeader(timeInterval: timestamp, frameId: frameId)
        let vector = Vector3(x: data.acceleration.x * g, y: data.acceleration.y * g, z: data.acceleration.z * g)
        let msg = Vector3Stamped(header: header, vector: vector)
        accelerometerSubject.send(TimestampedData(data: msg, timestamp: timestamp, frameId: frameId))
    }

    private func processGyroscope(_ data: CMGyroData) {
        let timestamp = TimeUtils.toUnixTimestamp(data.timestamp)
        let header = ROSHeader(timeInterval: timestamp, frameId: frameId)
        let vector = Vector3(x: data.rotationRate.x, y: data.rotationRate.y, z: data.rotationRate.z)
        let msg = Vector3Stamped(header: header, vector: vector)
        gyroscopeSubject.send(TimestampedData(data: msg, timestamp: timestamp, frameId: frameId))
    }
}

public enum MotionError: Error, LocalizedError {
    case notAvailable
    case notAuthorized

    public var errorDescription: String? {
        switch self {
        case .notAvailable: "Motion sensors not available on this device"
        case .notAuthorized: "Motion sensor access not authorized"
        }
    }
}
