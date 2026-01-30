import Foundation

/// Field datatype constants for PointCloud2
public enum PointFieldDatatype: UInt8 {
    case int8 = 1
    case uint8 = 2
    case int16 = 3
    case uint16 = 4
    case int32 = 5
    case uint32 = 6
    case float32 = 7
    case float64 = 8
}

/// ROS2 sensor_msgs/msg/PointField
public struct PointField {
    /// Name of the field
    public let name: String
    /// Offset from start of point struct
    public let offset: UInt32
    /// Datatype (see PointFieldDatatype)
    public let datatype: UInt8
    /// Number of elements in field
    public let count: UInt32

    public init(name: String, offset: UInt32, datatype: PointFieldDatatype, count: UInt32 = 1) {
        self.name = name
        self.offset = offset
        self.datatype = datatype.rawValue
        self.count = count
    }

    /// Standard XYZ fields for a basic point cloud
    public static func xyzFields() -> [PointField] {
        [
            PointField(name: "x", offset: 0, datatype: .float32),
            PointField(name: "y", offset: 4, datatype: .float32),
            PointField(name: "z", offset: 8, datatype: .float32)
        ]
    }

    /// XYZRGB fields
    public static func xyzrgbFields() -> [PointField] {
        [
            PointField(name: "x", offset: 0, datatype: .float32),
            PointField(name: "y", offset: 4, datatype: .float32),
            PointField(name: "z", offset: 8, datatype: .float32),
            PointField(name: "rgb", offset: 12, datatype: .float32)
        ]
    }

    /// XYZ with intensity
    public static func xyziFields() -> [PointField] {
        [
            PointField(name: "x", offset: 0, datatype: .float32),
            PointField(name: "y", offset: 4, datatype: .float32),
            PointField(name: "z", offset: 8, datatype: .float32),
            PointField(name: "intensity", offset: 12, datatype: .float32)
        ]
    }
}

/// ROS2 sensor_msgs/msg/PointCloud2
public struct PointCloud2 {
    public let header: ROSHeader

    /// 2D structure (if organized cloud). height=1 for unorganized.
    public let height: UInt32
    public let width: UInt32

    /// Describes the channels
    public let fields: [PointField]

    /// Is data big-endian?
    public let isBigEndian: Bool

    /// Length of a point in bytes
    public let pointStep: UInt32

    /// Length of a row in bytes
    public let rowStep: UInt32

    /// Point data
    public let data: Data

    /// True if there are no invalid points
    public let isDense: Bool

    public init(
        header: ROSHeader,
        height: UInt32,
        width: UInt32,
        fields: [PointField],
        isBigEndian: Bool = false,
        pointStep: UInt32,
        rowStep: UInt32,
        data: Data,
        isDense: Bool = false
    ) {
        self.header = header
        self.height = height
        self.width = width
        self.fields = fields
        self.isBigEndian = isBigEndian
        self.pointStep = pointStep
        self.rowStep = rowStep
        self.data = data
        self.isDense = isDense
    }

    /// Create an unorganized XYZ point cloud
    public static func xyzCloud(
        points: [(x: Float, y: Float, z: Float)],
        frameId: String,
        timestamp: TimeInterval? = nil
    ) -> PointCloud2 {
        let header = timestamp != nil
            ? ROSHeader(timeInterval: timestamp!, frameId: frameId)
            : ROSHeader.now(frameId: frameId)

        let pointStep: UInt32 = 12 // 3 floats * 4 bytes
        let numPoints = UInt32(points.count)

        var data = Data(capacity: Int(numPoints * pointStep))
        for point in points {
            withUnsafeBytes(of: point.x) { data.append(contentsOf: $0) }
            withUnsafeBytes(of: point.y) { data.append(contentsOf: $0) }
            withUnsafeBytes(of: point.z) { data.append(contentsOf: $0) }
        }

        return PointCloud2(
            header: header,
            height: 1,
            width: numPoints,
            fields: PointField.xyzFields(),
            pointStep: pointStep,
            rowStep: numPoints * pointStep,
            data: data
        )
    }

    /// Create an unorganized XYZI point cloud (with intensity/confidence)
    public static func xyziCloud(
        points: [(x: Float, y: Float, z: Float, intensity: Float)],
        frameId: String,
        timestamp: TimeInterval? = nil
    ) -> PointCloud2 {
        let header = timestamp != nil
            ? ROSHeader(timeInterval: timestamp!, frameId: frameId)
            : ROSHeader.now(frameId: frameId)

        let pointStep: UInt32 = 16 // 4 floats * 4 bytes
        let numPoints = UInt32(points.count)

        var data = Data(capacity: Int(numPoints * pointStep))
        for point in points {
            withUnsafeBytes(of: point.x) { data.append(contentsOf: $0) }
            withUnsafeBytes(of: point.y) { data.append(contentsOf: $0) }
            withUnsafeBytes(of: point.z) { data.append(contentsOf: $0) }
            withUnsafeBytes(of: point.intensity) { data.append(contentsOf: $0) }
        }

        return PointCloud2(
            header: header,
            height: 1,
            width: numPoints,
            fields: PointField.xyziFields(),
            pointStep: pointStep,
            rowStep: numPoints * pointStep,
            data: data
        )
    }
}
