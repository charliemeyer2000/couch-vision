import Foundation

/// ROS2 geometry_msgs/msg/Vector3
public struct Vector3: Equatable {
    public let x: Double
    public let y: Double
    public let z: Double

    public init(x: Double, y: Double, z: Double) {
        self.x = x
        self.y = y
        self.z = z
    }

    public static let zero = Vector3(x: 0, y: 0, z: 0)
}

/// ROS2 geometry_msgs/msg/Quaternion
public struct Quaternion: Equatable {
    public let x: Double
    public let y: Double
    public let z: Double
    public let w: Double

    public init(x: Double, y: Double, z: Double, w: Double) {
        self.x = x
        self.y = y
        self.z = z
        self.w = w
    }

    public static let identity = Quaternion(x: 0, y: 0, z: 0, w: 1)
}

/// 3x3 covariance matrix (row-major, 9 elements)
public typealias Covariance3x3 = [Double]

/// ROS2 sensor_msgs/msg/Imu
public struct ImuMessage {
    public let header: ROSHeader
    public let orientation: Quaternion
    public let orientationCovariance: Covariance3x3
    public let angularVelocity: Vector3
    public let angularVelocityCovariance: Covariance3x3
    public let linearAcceleration: Vector3
    public let linearAccelerationCovariance: Covariance3x3

    public init(
        header: ROSHeader,
        orientation: Quaternion,
        orientationCovariance: Covariance3x3,
        angularVelocity: Vector3,
        angularVelocityCovariance: Covariance3x3,
        linearAcceleration: Vector3,
        linearAccelerationCovariance: Covariance3x3
    ) {
        self.header = header
        self.orientation = orientation
        self.orientationCovariance = orientationCovariance
        self.angularVelocity = angularVelocity
        self.angularVelocityCovariance = angularVelocityCovariance
        self.linearAcceleration = linearAcceleration
        self.linearAccelerationCovariance = linearAccelerationCovariance
    }

    public static let defaultCovariance: Covariance3x3 = [0.01, 0, 0, 0, 0.01, 0, 0, 0, 0.01]
    public static let unavailableCovariance: Covariance3x3 = [-1, 0, 0, 0, 0, 0, 0, 0, 0]
}

/// ROS2 geometry_msgs/msg/Vector3Stamped
public struct Vector3Stamped {
    public let header: ROSHeader
    public let vector: Vector3

    public init(header: ROSHeader, vector: Vector3) {
        self.header = header
        self.vector = vector
    }
}
