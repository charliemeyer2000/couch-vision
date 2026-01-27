import Combine
@preconcurrency import CoreMotion
import Foundation

public struct EnvironmentConfig {
    public let magnetometerRate: SensorRateConfig
    public let barometerEnabled: Bool
    public let magnetometerEnabled: Bool

    public init(
        magnetometerRate: SensorRateConfig = .hz50,
        barometerEnabled: Bool = true,
        magnetometerEnabled: Bool = true
    ) {
        self.magnetometerRate = magnetometerRate
        self.barometerEnabled = barometerEnabled
        self.magnetometerEnabled = magnetometerEnabled
    }
}

public final class EnvironmentManager: ObservableObject {

    @Published public private(set) var state: SensorState = .unknown
    @Published public var isEnabled: Bool = false

    private let magneticFieldSubject = PassthroughSubject<TimestampedData<MagneticField>, Never>()
    public var magneticFieldPublisher: AnyPublisher<TimestampedData<MagneticField>, Never> {
        magneticFieldSubject.eraseToAnyPublisher()
    }

    private let pressureSubject = PassthroughSubject<TimestampedData<FluidPressure>, Never>()
    public var pressurePublisher: AnyPublisher<TimestampedData<FluidPressure>, Never> {
        pressureSubject.eraseToAnyPublisher()
    }

    private let altitudeSubject = PassthroughSubject<TimestampedData<Float64Msg>, Never>()
    public var altitudePublisher: AnyPublisher<TimestampedData<Float64Msg>, Never> {
        altitudeSubject.eraseToAnyPublisher()
    }

    public var config = EnvironmentConfig() {
        didSet { if state == .running { reconfigure() } }
    }

    private let motionManager = CMMotionManager()
    private let altimeter = CMAltimeter()
    private let operationQueue: OperationQueue
    public var framePrefix = "iphone"
    private var magnetometerFrameId: String { "\(framePrefix)_magnetometer" }
    private var barometerFrameId: String { "\(framePrefix)_barometer" }

    // Magnetometer covariance (estimated)
    private let magCovariance: [Double] = [0.0001, 0, 0, 0, 0.0001, 0, 0, 0, 0.0001]

    public let sensorId = "environment"
    public let displayName = "Environment (Mag/Baro)"

    public init() {
        operationQueue = OperationQueue()
        operationQueue.name = "com.couchvision.environment"
        operationQueue.maxConcurrentOperationCount = 1
        checkAvailability()
    }

    @discardableResult
    public func checkAvailability() -> Bool {
        let magAvailable = motionManager.isMagnetometerAvailable
        let baroAvailable = CMAltimeter.isRelativeAltitudeAvailable()
        let available = magAvailable || baroAvailable
        state = available ? .ready : .unavailable
        return available
    }

    public func requestPermissions() async -> Bool {
        // CoreMotion doesn't require explicit permissions for magnetometer/barometer
        let available = checkAvailability()
        await MainActor.run { state = available ? .ready : .unavailable }
        return available
    }

    public func start() throws {
        var started = false

        if config.magnetometerEnabled, motionManager.isMagnetometerAvailable {
            motionManager.magnetometerUpdateInterval = config.magnetometerRate.interval
            motionManager.startMagnetometerUpdates(to: operationQueue) { [weak self] data, _ in
                guard let self, let data else { return }
                processMagnetometer(data)
            }
            started = true
        }

        if config.barometerEnabled, CMAltimeter.isRelativeAltitudeAvailable() {
            altimeter.startRelativeAltitudeUpdates(to: operationQueue) { [weak self] data, _ in
                guard let self, let data else { return }
                processAltimeter(data)
            }
            started = true
        }

        guard started else { throw EnvironmentError.notAvailable }
        DispatchQueue.main.async { [weak self] in self?.state = .running }
    }

    public func stop() {
        state = .ready
        motionManager.stopMagnetometerUpdates()
        altimeter.stopRelativeAltitudeUpdates()
    }

    private func reconfigure() {
        stop()
        try? start()
    }

    private func processMagnetometer(_ data: CMMagnetometerData) {
        let timestamp = TimeUtils.toUnixTimestamp(data.timestamp)

        // CoreMotion gives microteslas, ROS expects Tesla
        let field = data.magneticField
        let teslaFactor = 1e-6
        let magneticField = Vector3(
            x: field.x * teslaFactor,
            y: field.y * teslaFactor,
            z: field.z * teslaFactor
        )

        let header = ROSHeader(timeInterval: timestamp, frameId: magnetometerFrameId)
        let msg = MagneticField(
            header: header,
            magneticField: magneticField,
            magneticFieldCovariance: magCovariance
        )

        magneticFieldSubject.send(TimestampedData(data: msg, timestamp: timestamp, frameId: magnetometerFrameId))
    }

    private func processAltimeter(_ data: CMAltitudeData) {
        let timestamp = Date().timeIntervalSince1970

        // Pressure: CMAltitudeData gives kPa, ROS expects Pa
        let pressurePa = data.pressure.doubleValue * 1000.0
        let header = ROSHeader(timeInterval: timestamp, frameId: barometerFrameId)
        let pressureMsg = FluidPressure(header: header, fluidPressure: pressurePa)
        pressureSubject.send(TimestampedData(data: pressureMsg, timestamp: timestamp, frameId: barometerFrameId))

        // Relative altitude in meters
        let altitudeMsg = Float64Msg(data: data.relativeAltitude.doubleValue)
        altitudeSubject.send(TimestampedData(data: altitudeMsg, timestamp: timestamp, frameId: barometerFrameId))
    }
}

public enum EnvironmentError: Error, LocalizedError {
    case notAvailable

    public var errorDescription: String? {
        switch self {
        case .notAvailable: "Environment sensors not available"
        }
    }
}
