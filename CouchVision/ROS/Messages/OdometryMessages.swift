import Foundation

// MARK: - geometry_msgs/Pose

public struct Pose {
    public let position: Vector3
    public let orientation: Quaternion
    public init(position: Vector3, orientation: Quaternion) {
        self.position = position
        self.orientation = orientation
    }
}

// MARK: - geometry_msgs/PoseWithCovariance

public struct PoseWithCovariance {
    public let pose: Pose
    public let covariance: [Double] // float64[36], 6x6 row-major
    public init(pose: Pose, covariance: [Double]) {
        self.pose = pose
        self.covariance = covariance
    }
}

// MARK: - geometry_msgs/TwistWithCovariance

public struct TwistWithCovariance {
    public let twist: Twist
    public let covariance: [Double] // float64[36], 6x6 row-major
    public init(twist: Twist, covariance: [Double]) {
        self.twist = twist
        self.covariance = covariance
    }
}

// MARK: - nav_msgs/Odometry

public struct OdometryMessage {
    public let header: ROSHeader
    public let childFrameId: String
    public let pose: PoseWithCovariance
    public let twist: TwistWithCovariance
    public init(
        header: ROSHeader,
        childFrameId: String,
        pose: PoseWithCovariance,
        twist: TwistWithCovariance
    ) {
        self.header = header
        self.childFrameId = childFrameId
        self.pose = pose
        self.twist = twist
    }
}

extension OdometryMessage {
    static let defaultPoseCovariance: [Double] = {
        var c = [Double](repeating: 0, count: 36)
        c[0] = 0.001; c[7] = 0.001; c[14] = 0.001
        c[21] = 0.003; c[28] = 0.003; c[35] = 0.003
        return c
    }()

    static let defaultTwistCovariance: [Double] = {
        var c = [Double](repeating: 0, count: 36)
        c[0] = 0.01; c[7] = 0.01; c[14] = 0.01
        c[21] = 0.05; c[28] = 0.05; c[35] = 0.05
        return c
    }()

    static let unknownTwistCovariance: [Double] = {
        var c = [Double](repeating: 0, count: 36)
        c[0] = -1
        return c
    }()
}
