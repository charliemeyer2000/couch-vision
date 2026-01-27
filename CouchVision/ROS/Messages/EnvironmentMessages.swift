import Foundation

// MARK: - sensor_msgs/MagneticField

public struct MagneticField {
    public let header: ROSHeader
    public let magneticField: Vector3 // in Tesla
    public let magneticFieldCovariance: [Double] // 9 elements (3x3)

    public init(
        header: ROSHeader,
        magneticField: Vector3,
        magneticFieldCovariance: [Double] = [0, 0, 0, 0, 0, 0, 0, 0, 0]
    ) {
        self.header = header
        self.magneticField = magneticField
        self.magneticFieldCovariance = magneticFieldCovariance
    }
}

// MARK: - sensor_msgs/FluidPressure

public struct FluidPressure {
    public let header: ROSHeader
    public let fluidPressure: Double // in Pascals
    public let variance: Double

    public init(header: ROSHeader, fluidPressure: Double, variance: Double = 0) {
        self.header = header
        self.fluidPressure = fluidPressure
        self.variance = variance
    }
}

// MARK: - std_msgs/Float64

public struct Float64Msg {
    public let data: Double

    public init(data: Double) {
        self.data = data
    }
}
