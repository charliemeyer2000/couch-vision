import ActivityKit
import Foundation

public struct StreamingActivityAttributes: ActivityAttributes {
    public struct ContentState: Codable, Hashable {
        public var isConnected: Bool
        public var cameraActive: Bool
        public var lidarActive: Bool
        public var imuActive: Bool
        public var messageRate: Double
        public var bytesSent: UInt64

        public init(
            isConnected: Bool = false,
            cameraActive: Bool = false,
            lidarActive: Bool = false,
            imuActive: Bool = false,
            messageRate: Double = 0,
            bytesSent: UInt64 = 0
        ) {
            self.isConnected = isConnected
            self.cameraActive = cameraActive
            self.lidarActive = lidarActive
            self.imuActive = imuActive
            self.messageRate = messageRate
            self.bytesSent = bytesSent
        }
    }

    public var endpoint: String

    public init(endpoint: String) {
        self.endpoint = endpoint
    }
}
