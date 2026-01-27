import ActivityKit
import Foundation

public struct StreamingActivityAttributes: ActivityAttributes {
    public struct ContentState: Codable, Hashable, Sendable {
        public var isConnected: Bool
        public var cameraActive: Bool
        public var lidarActive: Bool
        public var imuActive: Bool
        public var gpsActive: Bool
        public var environmentActive: Bool
        public var messageRate: Double
        public var bytesSent: UInt64

        public var isStreaming: Bool {
            cameraActive || lidarActive || imuActive || gpsActive || environmentActive
        }

        public var activeSensorCount: Int {
            [cameraActive, lidarActive, imuActive, gpsActive, environmentActive]
                .filter { $0 }.count
        }

        public init(
            isConnected: Bool = false,
            cameraActive: Bool = false,
            lidarActive: Bool = false,
            imuActive: Bool = false,
            gpsActive: Bool = false,
            environmentActive: Bool = false,
            messageRate: Double = 0,
            bytesSent: UInt64 = 0
        ) {
            self.isConnected = isConnected
            self.cameraActive = cameraActive
            self.lidarActive = lidarActive
            self.imuActive = imuActive
            self.gpsActive = gpsActive
            self.environmentActive = environmentActive
            self.messageRate = messageRate
            self.bytesSent = bytesSent
        }
    }

    public var endpoint: String

    public init(endpoint: String) {
        self.endpoint = endpoint
    }
}
