import Foundation

/// Utilities for timestamp conversion between iOS sensor timestamps and Unix epoch
public enum TimeUtils {
    /// Offset to convert system uptime timestamps to Unix epoch
    /// Sensor APIs (CoreMotion, ARKit, AVFoundation) use time-since-boot
    private static var bootTimeOffset: TimeInterval {
        Date().timeIntervalSince1970 - ProcessInfo.processInfo.systemUptime
    }

    /// Convert a sensor timestamp (time since boot) to Unix epoch timestamp
    public static func toUnixTimestamp(_ sensorTimestamp: TimeInterval) -> TimeInterval {
        sensorTimestamp + bootTimeOffset
    }
}
