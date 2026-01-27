import Foundation

// MARK: - sensor_msgs/NavSatStatus

public struct NavSatStatus {
    public static let statusNoFix: Int8 = -1
    public static let statusFix: Int8 = 0
    public static let statusSbasFix: Int8 = 1
    public static let statusGbassFix: Int8 = 2

    public static let serviceGPS: UInt16 = 1
    public static let serviceGLONASS: UInt16 = 2
    public static let serviceCOMPASS: UInt16 = 4
    public static let serviceGALILEO: UInt16 = 8

    public let status: Int8
    public let service: UInt16

    public init(status: Int8 = NavSatStatus.statusFix, service: UInt16 = NavSatStatus.serviceGPS) {
        self.status = status
        self.service = service
    }
}

// MARK: - sensor_msgs/NavSatFix

public struct NavSatFix {
    public static let covarianceTypeUnknown: UInt8 = 0
    public static let covarianceTypeApproximated: UInt8 = 1
    public static let covarianceTypeDiagonalKnown: UInt8 = 2
    public static let covarianceTypeKnown: UInt8 = 3

    public let header: ROSHeader
    public let status: NavSatStatus
    public let latitude: Double
    public let longitude: Double
    public let altitude: Double
    public let positionCovariance: [Double] // 9 elements (3x3)
    public let positionCovarianceType: UInt8

    public init(
        header: ROSHeader,
        status: NavSatStatus = NavSatStatus(),
        latitude: Double,
        longitude: Double,
        altitude: Double,
        horizontalAccuracy: Double,
        verticalAccuracy: Double
    ) {
        self.header = header
        self.status = status
        self.latitude = latitude
        self.longitude = longitude
        self.altitude = altitude

        let hVar = horizontalAccuracy * horizontalAccuracy
        let vVar = verticalAccuracy * verticalAccuracy
        positionCovariance = [hVar, 0, 0, 0, hVar, 0, 0, 0, vVar]
        positionCovarianceType = NavSatFix.covarianceTypeDiagonalKnown
    }
}

// MARK: - geometry_msgs/Twist

public struct Twist {
    public let linear: Vector3
    public let angular: Vector3

    public init(linear: Vector3 = .zero, angular: Vector3 = .zero) {
        self.linear = linear
        self.angular = angular
    }
}

// MARK: - geometry_msgs/TwistStamped

public struct TwistStamped {
    public let header: ROSHeader
    public let twist: Twist

    public init(header: ROSHeader, twist: Twist) {
        self.header = header
        self.twist = twist
    }
}
