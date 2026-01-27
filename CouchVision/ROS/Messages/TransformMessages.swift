import Foundation

// MARK: - geometry_msgs/Transform

public struct Transform {
    public let translation: Vector3
    public let rotation: Quaternion

    public init(translation: Vector3, rotation: Quaternion) {
        self.translation = translation
        self.rotation = rotation
    }

    public static let identity = Transform(translation: .zero, rotation: .identity)
}

// MARK: - geometry_msgs/TransformStamped

public struct TransformStamped {
    public let header: ROSHeader
    public let childFrameId: String
    public let transform: Transform

    public init(header: ROSHeader, childFrameId: String, transform: Transform) {
        self.header = header
        self.childFrameId = childFrameId
        self.transform = transform
    }
}

// MARK: - tf2_msgs/TFMessage

public struct TFMessage {
    public let transforms: [TransformStamped]

    public init(transforms: [TransformStamped]) {
        self.transforms = transforms
    }
}
