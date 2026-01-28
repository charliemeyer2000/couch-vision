import Foundation

/// Protocol for types that can be encoded to CDR format
public protocol CDREncodable {
    func encode(to encoder: CDREncoder)
}

/// CDR (Common Data Representation) Encoder for ROS2 message serialization
/// Implements XCDR1 encoding used by DDS/ROS2
public final class CDREncoder {

    private var buffer: Data
    private var position: Int = 0

    private static let encapsulationHeader: [UInt8] = [0x00, 0x01, 0x00, 0x00] // CDR_LE

    public init(capacity: Int = 256) {
        buffer = Data(capacity: capacity)
    }

    public var encodedData: Data {
        var result = Data(CDREncoder.encapsulationHeader)
        result.append(buffer)
        return result
    }

    public var rawData: Data {
        buffer
    }

    // MARK: - Alignment

    private func align(to boundary: Int) {
        let remainder = position % boundary
        if remainder != 0 {
            let padding = boundary - remainder
            buffer.append(contentsOf: [UInt8](repeating: 0, count: padding))
            position += padding
        }
    }

    // MARK: - Primitives

    public func encode(_ value: Bool) {
        buffer.append(value ? 1 : 0); position += 1
    }

    public func encode(_ value: UInt8) {
        buffer.append(value); position += 1
    }

    public func encode(_ value: Int8) {
        buffer.append(UInt8(bitPattern: value)); position += 1
    }

    public func encode(_ value: UInt16) {
        align(to: 2); appendLittleEndian(value); position += 2
    }

    public func encode(_ value: Int16) {
        align(to: 2); appendLittleEndian(value); position += 2
    }

    public func encode(_ value: UInt32) {
        align(to: 4); appendLittleEndian(value); position += 4
    }

    public func encode(_ value: Int32) {
        align(to: 4); appendLittleEndian(value); position += 4
    }

    public func encode(_ value: UInt64) {
        align(to: 8); appendLittleEndian(value); position += 8
    }

    public func encode(_ value: Int64) {
        align(to: 8); appendLittleEndian(value); position += 8
    }

    public func encode(_ value: Float) {
        align(to: 4)
        appendLittleEndian(value.bitPattern)
        position += 4
    }

    public func encode(_ value: Double) {
        align(to: 8)
        appendLittleEndian(value.bitPattern)
        position += 8
    }

    private func appendLittleEndian(_ value: some FixedWidthInteger) {
        var v = value.littleEndian
        withUnsafeBytes(of: &v) { buffer.append(contentsOf: $0) }
    }

    // MARK: - String (length-prefixed with null terminator)

    public func encode(_ value: String) {
        let utf8 = value.utf8
        encode(UInt32(utf8.count + 1))
        buffer.append(contentsOf: utf8)
        buffer.append(0)
        position += utf8.count + 1
    }

    // MARK: - Sequences

    public func encode(_ values: [UInt8]) {
        encode(UInt32(values.count))
        buffer.append(contentsOf: values)
        position += values.count
    }

    public func encode(_ data: Data) {
        encode(UInt32(data.count))
        buffer.append(data)
        position += data.count
    }

    public func encode(_ values: [Double]) {
        encode(UInt32(values.count))
        values.forEach { encode($0) }
    }

    public func encode(_ values: [Float]) {
        encode(UInt32(values.count))
        values.forEach { encode($0) }
    }

    // MARK: - Fixed arrays (no length prefix)

    public func encodeFixed(_ values: [Double]) {
        values.forEach { encode($0) }
    }

    public func encodeFixed(_ values: [Float]) {
        values.forEach { encode($0) }
    }

    // MARK: - ROS2 Types

    public func encode(_ time: ROSTime) {
        encode(time.sec); encode(time.nanosec)
    }

    public func encode(_ header: ROSHeader) {
        encode(header.stamp); encode(header.frameId)
    }

    public func encode(_ v: Vector3) {
        encode(v.x); encode(v.y); encode(v.z)
    }

    public func encode(_ q: Quaternion) {
        encode(q.x); encode(q.y); encode(q.z); encode(q.w)
    }

    public func encode(_ msg: CompressedImage) {
        encode(msg.header); encode(msg.format); encode(msg.data)
    }

    public func encode(_ msg: ROSImage) {
        encode(msg.header); encode(msg.height); encode(msg.width)
        encode(msg.encoding); encode(msg.isBigEndian); encode(msg.step); encode(msg.data)
    }

    public func encode(_ msg: ImuMessage) {
        encode(msg.header)
        encode(msg.orientation); encodeFixed(msg.orientationCovariance)
        encode(msg.angularVelocity); encodeFixed(msg.angularVelocityCovariance)
        encode(msg.linearAcceleration); encodeFixed(msg.linearAccelerationCovariance)
    }

    public func encode(_ field: PointField) {
        encode(field.name); encode(field.offset); encode(field.datatype); encode(field.count)
    }

    public func encode(_ msg: PointCloud2) {
        encode(msg.header); encode(msg.height); encode(msg.width)
        encode(UInt32(msg.fields.count))
        msg.fields.forEach { encode($0) }
        encode(msg.isBigEndian); encode(msg.pointStep); encode(msg.rowStep)
        encode(msg.data); encode(msg.isDense)
    }

    public func encode(_ msg: Vector3Stamped) {
        encode(msg.header); encode(msg.vector)
    }

    public func encode(_ msg: NavSatStatus) {
        encode(msg.status); encode(msg.service)
    }

    public func encode(_ msg: NavSatFix) {
        encode(msg.header); encode(msg.status)
        encode(msg.latitude); encode(msg.longitude); encode(msg.altitude)
        encodeFixed(msg.positionCovariance); encode(msg.positionCovarianceType)
    }

    public func encode(_ msg: Twist) {
        encode(msg.linear); encode(msg.angular)
    }

    public func encode(_ msg: TwistStamped) {
        encode(msg.header); encode(msg.twist)
    }

    public func encode(_ msg: MagneticField) {
        encode(msg.header); encode(msg.magneticField); encodeFixed(msg.magneticFieldCovariance)
    }

    public func encode(_ msg: FluidPressure) {
        encode(msg.header); encode(msg.fluidPressure); encode(msg.variance)
    }

    public func encode(_ msg: Float64Msg) {
        encode(msg.data)
    }

    public func encode(_ msg: Int32Msg) {
        encode(msg.data)
    }

    public func encode(_ msg: BoolMsg) {
        encode(msg.data)
    }

    public func encode(_ msg: BatteryState) {
        encode(msg.header)
        encode(msg.voltage); encode(msg.temperature); encode(msg.current)
        encode(msg.charge); encode(msg.capacity); encode(msg.designCapacity)
        encode(msg.percentage)
        encode(msg.powerSupplyStatus); encode(msg.powerSupplyHealth); encode(msg.powerSupplyTechnology)
        encode(msg.present)
        encode(msg.cellVoltage); encode(msg.cellTemperature)
        encode(msg.location); encode(msg.serialNumber)
    }

    public func encode(_ msg: CameraInfo) {
        encode(msg.header); encode(msg.height); encode(msg.width)
        encode(msg.distortionModel); encode(msg.d)
        encodeFixed(msg.k); encodeFixed(msg.r); encodeFixed(msg.p)
        encode(msg.binningX); encode(msg.binningY)
        encode(msg.roiXOffset); encode(msg.roiYOffset)
        encode(msg.roiWidth); encode(msg.roiHeight); encode(msg.roiDoRectify)
    }

    public func encode(_ msg: Transform) {
        encode(msg.translation)
        encode(msg.rotation)
    }

    public func encode(_ msg: TransformStamped) {
        encode(msg.header)
        encode(msg.childFrameId)
        encode(msg.transform)
    }

    public func encode(_ msg: TFMessage) {
        encode(UInt32(msg.transforms.count))
        for t in msg.transforms {
            encode(t)
        }
    }

    public func encode(_ msg: Pose) {
        encode(msg.position); encode(msg.orientation)
    }

    public func encode(_ msg: PoseWithCovariance) {
        encode(msg.pose); encodeFixed(msg.covariance)
    }

    public func encode(_ msg: TwistWithCovariance) {
        encode(msg.twist); encodeFixed(msg.covariance)
    }

    public func encode(_ msg: OdometryMessage) {
        encode(msg.header); encode(msg.childFrameId); encode(msg.pose); encode(msg.twist)
    }

    // MARK: - Factory

    public static func encode(_ msg: CompressedImage) -> Data {
        let e = CDREncoder(); e.encode(msg); return e.encodedData
    }

    public static func encode(_ msg: ImuMessage) -> Data {
        let e = CDREncoder(); e.encode(msg); return e.encodedData
    }

    public static func encode(_ msg: PointCloud2) -> Data {
        let e = CDREncoder(); e.encode(msg); return e.encodedData
    }

    public static func encode(_ msg: ROSImage) -> Data {
        let e = CDREncoder(); e.encode(msg); return e.encodedData
    }

    public static func encode(_ msg: CameraInfo) -> Data {
        let e = CDREncoder(); e.encode(msg); return e.encodedData
    }

    public static func encode(_ msg: NavSatFix) -> Data {
        let e = CDREncoder(); e.encode(msg); return e.encodedData
    }

    public static func encode(_ msg: TwistStamped) -> Data {
        let e = CDREncoder(); e.encode(msg); return e.encodedData
    }

    public static func encode(_ msg: MagneticField) -> Data {
        let e = CDREncoder(); e.encode(msg); return e.encodedData
    }

    public static func encode(_ msg: FluidPressure) -> Data {
        let e = CDREncoder(); e.encode(msg); return e.encodedData
    }

    public static func encode(_ msg: Float64Msg) -> Data {
        let e = CDREncoder(); e.encode(msg); return e.encodedData
    }

    public static func encode(_ msg: Int32Msg) -> Data {
        let e = CDREncoder(); e.encode(msg); return e.encodedData
    }

    public static func encode(_ msg: BatteryState) -> Data {
        let e = CDREncoder(); e.encode(msg); return e.encodedData
    }

    public static func encode(_ msg: BoolMsg) -> Data {
        let e = CDREncoder(); e.encode(msg); return e.encodedData
    }

    public static func encode(_ msg: Vector3Stamped) -> Data {
        let e = CDREncoder(); e.encode(msg); return e.encodedData
    }

    public static func encode(_ msg: TFMessage) -> Data {
        let e = CDREncoder(); e.encode(msg); return e.encodedData
    }

    public static func encode(_ msg: OdometryMessage) -> Data {
        let e = CDREncoder(); e.encode(msg); return e.encodedData
    }
}
