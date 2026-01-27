import Foundation

// MARK: - sensor_msgs/BatteryState

public struct BatteryState {
    public static let powerSupplyStatusUnknown: UInt8 = 0
    public static let powerSupplyStatusCharging: UInt8 = 1
    public static let powerSupplyStatusDischarging: UInt8 = 2
    public static let powerSupplyStatusNotCharging: UInt8 = 3
    public static let powerSupplyStatusFull: UInt8 = 4

    public static let powerSupplyHealthUnknown: UInt8 = 0
    public static let powerSupplyHealthGood: UInt8 = 1
    public static let powerSupplyHealthOverheat: UInt8 = 2
    public static let powerSupplyHealthDead: UInt8 = 3
    public static let powerSupplyHealthOverVoltage: UInt8 = 4
    public static let powerSupplyHealthUnspecFailure: UInt8 = 5
    public static let powerSupplyHealthCold: UInt8 = 6
    public static let powerSupplyHealthWatchdogTimerExpire: UInt8 = 7
    public static let powerSupplyHealthSafetyTimerExpire: UInt8 = 8

    public static let powerSupplyTechnologyUnknown: UInt8 = 0
    public static let powerSupplyTechnologyNiMH: UInt8 = 1
    public static let powerSupplyTechnologyLiIon: UInt8 = 2
    public static let powerSupplyTechnologyLiPo: UInt8 = 3
    public static let powerSupplyTechnologyLiFe: UInt8 = 4
    public static let powerSupplyTechnologyNiCd: UInt8 = 5
    public static let powerSupplyTechnologyLiMn: UInt8 = 6

    public let header: ROSHeader
    public let voltage: Float
    public let temperature: Float
    public let current: Float
    public let charge: Float
    public let capacity: Float
    public let designCapacity: Float
    public let percentage: Float
    public let powerSupplyStatus: UInt8
    public let powerSupplyHealth: UInt8
    public let powerSupplyTechnology: UInt8
    public let present: Bool
    public let cellVoltage: [Float]
    public let cellTemperature: [Float]
    public let location: String
    public let serialNumber: String

    public init(
        header: ROSHeader,
        percentage: Float,
        powerSupplyStatus: UInt8,
        powerSupplyHealth: UInt8 = BatteryState.powerSupplyHealthUnknown,
        location: String = "iPhone"
    ) {
        self.header = header
        voltage = .nan
        temperature = .nan
        current = .nan
        charge = .nan
        capacity = .nan
        designCapacity = .nan
        self.percentage = percentage
        self.powerSupplyStatus = powerSupplyStatus
        self.powerSupplyHealth = powerSupplyHealth
        powerSupplyTechnology = BatteryState.powerSupplyTechnologyLiIon
        present = true
        cellVoltage = []
        cellTemperature = []
        self.location = location
        serialNumber = ""
    }
}

// MARK: - std_msgs/Int32

public struct Int32Msg {
    public let data: Int32

    public init(data: Int32) {
        self.data = data
    }
}

// MARK: - std_msgs/Bool

public struct BoolMsg {
    public let data: Bool

    public init(data: Bool) {
        self.data = data
    }
}
