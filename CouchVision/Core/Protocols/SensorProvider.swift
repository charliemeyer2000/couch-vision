import Combine
import Foundation

/// Represents a timestamped sensor reading
public struct TimestampedData<T> {
    public let data: T
    public let timestamp: TimeInterval
    public let frameId: String

    public init(data: T, timestamp: TimeInterval = Date().timeIntervalSince1970, frameId: String) {
        self.data = data
        self.timestamp = timestamp
        self.frameId = frameId
    }
}

/// State of a sensor
public enum SensorState: Equatable {
    case unknown
    case unavailable
    case unauthorized
    case ready
    case running
    case error(String)

    var isOperational: Bool {
        switch self {
        case .ready,
             .running: true
        default: false
        }
    }
}

/// Configuration for sensor update rates
public struct SensorRateConfig {
    public let interval: TimeInterval
    public var frequencyHz: Double { 1.0 / interval }

    public static let hz100 = SensorRateConfig(interval: 1.0 / 100.0)
    public static let hz50 = SensorRateConfig(interval: 1.0 / 50.0)
    public static let hz30 = SensorRateConfig(interval: 1.0 / 30.0)
    public static let hz15 = SensorRateConfig(interval: 1.0 / 15.0)
    public static let hz10 = SensorRateConfig(interval: 1.0 / 10.0)
    public static let hz1 = SensorRateConfig(interval: 1.0)

    public init(interval: TimeInterval) {
        self.interval = max(0.001, interval)
    }

    public init(frequencyHz: Double) {
        interval = 1.0 / max(1.0, frequencyHz)
    }
}
