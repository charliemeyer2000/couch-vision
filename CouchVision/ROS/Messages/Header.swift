import Foundation

/// ROS2 builtin_interfaces/msg/Time
public struct ROSTime: Equatable {
    public let sec: Int32
    public let nanosec: UInt32

    public init(sec: Int32, nanosec: UInt32) {
        self.sec = sec
        self.nanosec = nanosec
    }

    public init(timeInterval: TimeInterval) {
        let wholeSec = floor(timeInterval)
        sec = Int32(wholeSec)
        nanosec = UInt32((timeInterval - wholeSec) * 1_000_000_000)
    }

    public static var now: ROSTime { ROSTime(timeInterval: Date().timeIntervalSince1970) }
}

/// ROS2 std_msgs/msg/Header
public struct ROSHeader: Equatable {
    public let stamp: ROSTime
    public let frameId: String

    public init(stamp: ROSTime, frameId: String) {
        self.stamp = stamp
        self.frameId = frameId
    }

    public static func now(frameId: String) -> ROSHeader {
        ROSHeader(stamp: .now, frameId: frameId)
    }

    public init(timeInterval: TimeInterval, frameId: String) {
        stamp = ROSTime(timeInterval: timeInterval)
        self.frameId = frameId
    }
}
