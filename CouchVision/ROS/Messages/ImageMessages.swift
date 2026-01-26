import Foundation

/// ROS2 sensor_msgs/msg/CompressedImage
public struct CompressedImage {
    public let header: ROSHeader
    /// Image format (e.g., "jpeg", "png")
    public let format: String
    /// Compressed image data
    public let data: Data

    public init(header: ROSHeader, format: String, data: Data) {
        self.header = header
        self.format = format
        self.data = data
    }

    /// Create a JPEG compressed image
    public static func jpeg(data: Data, frameId: String, timestamp: TimeInterval? = nil) -> CompressedImage {
        let header = timestamp != nil
            ? ROSHeader(timeInterval: timestamp!, frameId: frameId)
            : ROSHeader.now(frameId: frameId)
        return CompressedImage(header: header, format: "jpeg", data: data)
    }
}

/// ROS2 sensor_msgs/msg/Image (uncompressed)
public struct ROSImage {
    public let header: ROSHeader
    /// Image height (rows)
    public let height: UInt32
    /// Image width (columns)
    public let width: UInt32
    /// Encoding (e.g., "rgb8", "bgr8", "mono8", "32FC1")
    public let encoding: String
    /// Is data big-endian? (0 = little, 1 = big)
    public let isBigEndian: UInt8
    /// Row step (full row length in bytes)
    public let step: UInt32
    /// Image data
    public let data: Data

    public init(
        header: ROSHeader,
        height: UInt32,
        width: UInt32,
        encoding: String,
        isBigEndian: UInt8 = 0,
        step: UInt32,
        data: Data
    ) {
        self.header = header
        self.height = height
        self.width = width
        self.encoding = encoding
        self.isBigEndian = isBigEndian
        self.step = step
        self.data = data
    }

    /// Create a depth image (32FC1 format)
    public static func depthImage(
        data: Data,
        width: UInt32,
        height: UInt32,
        frameId: String,
        timestamp: TimeInterval? = nil
    ) -> ROSImage {
        let header = timestamp != nil
            ? ROSHeader(timeInterval: timestamp!, frameId: frameId)
            : ROSHeader.now(frameId: frameId)

        return ROSImage(
            header: header,
            height: height,
            width: width,
            encoding: "32FC1",
            isBigEndian: 0,
            step: width * 4, // 4 bytes per float
            data: data
        )
    }
}

/// ROS2 sensor_msgs/msg/CameraInfo
public struct CameraInfo {
    public let header: ROSHeader
    /// Image dimensions
    public let height: UInt32
    public let width: UInt32
    /// Distortion model (e.g., "plumb_bob", "rational_polynomial")
    public let distortionModel: String
    /// Distortion parameters (D)
    public let d: [Double]
    /// Intrinsic camera matrix (K) - 3x3
    public let k: [Double] // 9 elements
    /// Rectification matrix (R) - 3x3
    public let r: [Double] // 9 elements
    /// Projection matrix (P) - 3x4
    public let p: [Double] // 12 elements
    /// Binning
    public let binningX: UInt32
    public let binningY: UInt32
    /// ROI
    public let roiXOffset: UInt32
    public let roiYOffset: UInt32
    public let roiWidth: UInt32
    public let roiHeight: UInt32
    public let roiDoRectify: Bool

    public init(
        header: ROSHeader,
        height: UInt32,
        width: UInt32,
        distortionModel: String = "plumb_bob",
        d: [Double] = [],
        k: [Double],
        r: [Double] = [1, 0, 0, 0, 1, 0, 0, 0, 1],
        p: [Double],
        binningX: UInt32 = 0,
        binningY: UInt32 = 0,
        roiXOffset: UInt32 = 0,
        roiYOffset: UInt32 = 0,
        roiWidth: UInt32 = 0,
        roiHeight: UInt32 = 0,
        roiDoRectify: Bool = false
    ) {
        self.header = header
        self.height = height
        self.width = width
        self.distortionModel = distortionModel
        self.d = d
        self.k = k
        self.r = r
        self.p = p
        self.binningX = binningX
        self.binningY = binningY
        self.roiXOffset = roiXOffset
        self.roiYOffset = roiYOffset
        self.roiWidth = roiWidth
        self.roiHeight = roiHeight
        self.roiDoRectify = roiDoRectify
    }
}
