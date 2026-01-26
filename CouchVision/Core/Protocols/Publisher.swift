import Foundation

public enum QoS {
    case reliable
    case bestEffort

    var description: String {
        switch self {
        case .reliable: "reliable"
        case .bestEffort: "best_effort"
        }
    }
}

public struct PublisherConfig {
    let topic: String
    let qos: QoS
    let bufferSize: Int

    public init(topic: String, qos: QoS = .bestEffort, bufferSize: Int = 10) {
        self.topic = topic
        self.qos = qos
        self.bufferSize = bufferSize
    }
}

/// Abstraction for publishing CDR-serialized ROS2 messages.
/// Implementations: ZenohPublisher, MockPublisher
public protocol Publisher: AnyObject {
    var isConnected: Bool { get }
    var onConnectionStateChanged: ((Bool) -> Void)? { get set }

    func connect(to endpoint: String) async throws
    func disconnect() async
    func publish(_ data: Data, to topic: String) async throws
    func declarePublisher(config: PublisherConfig) async throws
}

public enum PublisherError: Error, LocalizedError {
    case notConnected
    case connectionFailed(String)
    case publishFailed(String)
    case invalidEndpoint(String)
    case timeout

    public var errorDescription: String? {
        switch self {
        case .notConnected: "Publisher is not connected"
        case .connectionFailed(let reason): "Connection failed: \(reason)"
        case .publishFailed(let reason): "Publish failed: \(reason)"
        case .invalidEndpoint(let endpoint): "Invalid endpoint: \(endpoint)"
        case .timeout: "Operation timed out"
        }
    }
}
