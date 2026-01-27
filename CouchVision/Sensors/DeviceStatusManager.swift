import Combine
import Foundation
import UIKit

public struct DeviceStatusConfig {
    public let batteryUpdateInterval: TimeInterval
    public let proximityEnabled: Bool

    public init(
        batteryUpdateInterval: TimeInterval = 1.0,
        proximityEnabled: Bool = false
    ) {
        self.batteryUpdateInterval = batteryUpdateInterval
        self.proximityEnabled = proximityEnabled
    }
}

@MainActor
public final class DeviceStatusManager: ObservableObject {

    @Published public private(set) var state: SensorState = .ready
    @Published public var isEnabled: Bool = false

    private let batterySubject = PassthroughSubject<TimestampedData<BatteryState>, Never>()
    public var batteryPublisher: AnyPublisher<TimestampedData<BatteryState>, Never> {
        batterySubject.eraseToAnyPublisher()
    }

    private let thermalSubject = PassthroughSubject<TimestampedData<Int32Msg>, Never>()
    public var thermalPublisher: AnyPublisher<TimestampedData<Int32Msg>, Never> {
        thermalSubject.eraseToAnyPublisher()
    }

    private let proximitySubject = PassthroughSubject<TimestampedData<BoolMsg>, Never>()
    public var proximityPublisher: AnyPublisher<TimestampedData<BoolMsg>, Never> {
        proximitySubject.eraseToAnyPublisher()
    }

    public var config = DeviceStatusConfig()

    private var batteryTimer: Timer?
    private var thermalObserver: NSObjectProtocol?
    private var proximityObserver: NSObjectProtocol?
    public var framePrefix = "iphone"
    private var frameId: String { "\(framePrefix)_device" }

    public let sensorId = "device_status"
    public let displayName = "Device Status"

    public init() {}

    public func requestPermissions() async -> Bool {
        true // No permissions needed
    }

    public func start() {
        UIDevice.current.isBatteryMonitoringEnabled = true

        // Battery updates via timer
        batteryTimer = Timer.scheduledTimer(withTimeInterval: config.batteryUpdateInterval, repeats: true) { [weak self] _ in
            Task { @MainActor in self?.publishBatteryState() }
        }
        publishBatteryState()

        // Thermal state updates
        thermalObserver = NotificationCenter.default.addObserver(
            forName: ProcessInfo.thermalStateDidChangeNotification,
            object: nil,
            queue: .main
        ) { [weak self] _ in
            Task { @MainActor in self?.publishThermalState() }
        }
        publishThermalState()

        // Proximity sensor
        if config.proximityEnabled {
            UIDevice.current.isProximityMonitoringEnabled = true
            proximityObserver = NotificationCenter.default.addObserver(
                forName: UIDevice.proximityStateDidChangeNotification,
                object: nil,
                queue: .main
            ) { [weak self] _ in
                Task { @MainActor in self?.publishProximityState() }
            }
        }

        state = .running
    }

    public func stop() {
        batteryTimer?.invalidate()
        batteryTimer = nil

        if let thermalObserver {
            NotificationCenter.default.removeObserver(thermalObserver)
        }
        thermalObserver = nil

        if let proximityObserver {
            NotificationCenter.default.removeObserver(proximityObserver)
        }
        proximityObserver = nil

        UIDevice.current.isBatteryMonitoringEnabled = false
        UIDevice.current.isProximityMonitoringEnabled = false

        state = .ready
    }

    private func publishBatteryState() {
        let timestamp = Date().timeIntervalSince1970
        let header = ROSHeader(timeInterval: timestamp, frameId: frameId)

        let level = UIDevice.current.batteryLevel
        let uiState = UIDevice.current.batteryState

        let status: UInt8 = switch uiState {
        case .unknown: BatteryState.powerSupplyStatusUnknown
        case .unplugged: BatteryState.powerSupplyStatusDischarging
        case .charging: BatteryState.powerSupplyStatusCharging
        case .full: BatteryState.powerSupplyStatusFull
        @unknown default: BatteryState.powerSupplyStatusUnknown
        }

        // Thermal state affects battery health reporting
        let health: UInt8 = switch ProcessInfo.processInfo.thermalState {
        case .nominal,
             .fair: BatteryState.powerSupplyHealthGood
        case .serious,
             .critical: BatteryState.powerSupplyHealthOverheat
        @unknown default: BatteryState.powerSupplyHealthUnknown
        }

        let msg = BatteryState(
            header: header,
            percentage: level >= 0 ? level : .nan,
            powerSupplyStatus: status,
            powerSupplyHealth: health
        )

        batterySubject.send(TimestampedData(data: msg, timestamp: timestamp, frameId: frameId))
    }

    private func publishThermalState() {
        let timestamp = Date().timeIntervalSince1970
        let thermalState = ProcessInfo.processInfo.thermalState

        let stateValue: Int32 = switch thermalState {
        case .nominal: 0
        case .fair: 1
        case .serious: 2
        case .critical: 3
        @unknown default: -1
        }

        let msg = Int32Msg(data: stateValue)
        thermalSubject.send(TimestampedData(data: msg, timestamp: timestamp, frameId: frameId))
    }

    private func publishProximityState() {
        let timestamp = Date().timeIntervalSince1970
        let isNear = UIDevice.current.proximityState
        let msg = BoolMsg(data: isNear)
        proximitySubject.send(TimestampedData(data: msg, timestamp: timestamp, frameId: frameId))
    }
}
