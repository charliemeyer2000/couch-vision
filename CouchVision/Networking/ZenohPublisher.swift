import Foundation
import Network

/// Zenoh publisher implementation
/// This will implement the Zenoh protocol for direct ROS2 communication
///
/// Current status: Placeholder - will be implemented using either:
/// 1. zenoh-pico C library compiled for iOS
/// 2. Native Swift implementation of Zenoh protocol
///
/// For now, this acts as a TCP-based raw publisher that can work with
/// a Zenoh bridge on the receiving end.
public final class ZenohPublisher: Publisher {

    // MARK: - Publisher Protocol

    public private(set) var isConnected: Bool = false
    public var onConnectionStateChanged: ((Bool) -> Void)?

    // MARK: - Configuration

    private var endpoint: String = ""
    private var connection: NWConnection?
    private let queue = DispatchQueue(label: "com.couchvision.zenoh")

    // MARK: - Declared Publishers

    private var declaredTopics: Set<String> = []

    // MARK: - Initialization

    public init() {}

    // MARK: - Publisher Protocol Implementation

    public func connect(to endpoint: String) async throws {
        self.endpoint = endpoint

        // Parse endpoint (format: tcp://host:port or zenoh://host:port)
        guard let (host, port) = parseEndpoint(endpoint) else {
            throw PublisherError.invalidEndpoint(endpoint)
        }

        return try await withCheckedThrowingContinuation { continuation in
            let nwEndpoint = NWEndpoint.hostPort(
                host: NWEndpoint.Host(host),
                port: NWEndpoint.Port(rawValue: port) ?? .init(integerLiteral: 7447)
            )

            let parameters = NWParameters.tcp
            parameters.allowLocalEndpointReuse = true

            let connection = NWConnection(to: nwEndpoint, using: parameters)

            connection.stateUpdateHandler = { [weak self] state in
                switch state {
                case .ready:
                    self?.isConnected = true
                    self?.onConnectionStateChanged?(true)
                    continuation.resume()

                case .failed(let error):
                    self?.isConnected = false
                    self?.onConnectionStateChanged?(false)
                    continuation.resume(throwing: PublisherError.connectionFailed(error.localizedDescription))

                case .cancelled:
                    self?.isConnected = false
                    self?.onConnectionStateChanged?(false)

                case .waiting(let error):
                    print("[ZenohPublisher] Waiting: \(error)")

                default:
                    break
                }
            }

            self.connection = connection
            connection.start(queue: queue)
        }
    }

    public func disconnect() async {
        connection?.cancel()
        connection = nil
        isConnected = false
        onConnectionStateChanged?(false)
        declaredTopics.removeAll()
    }

    public func publish(_ data: Data, to topic: String) async throws {
        guard isConnected, let connection else {
            throw PublisherError.notConnected
        }

        // Create a simple message frame
        // Format: [topic_len:4][topic][data_len:4][data]
        // This is a simple protocol that a bridge can parse
        // TODO: Replace with actual Zenoh protocol when implemented

        var frame = Data()

        // Topic length (4 bytes, little-endian)
        let topicData = topic.data(using: .utf8) ?? Data()
        var topicLen = UInt32(topicData.count).littleEndian
        frame.append(contentsOf: withUnsafeBytes(of: &topicLen) { Array($0) })

        // Topic
        frame.append(topicData)

        // Data length (4 bytes, little-endian)
        var dataLen = UInt32(data.count).littleEndian
        frame.append(contentsOf: withUnsafeBytes(of: &dataLen) { Array($0) })

        // Data
        frame.append(data)

        return try await withCheckedThrowingContinuation { continuation in
            connection.send(content: frame, completion: .contentProcessed { error in
                if let error {
                    continuation.resume(throwing: PublisherError.publishFailed(error.localizedDescription))
                } else {
                    continuation.resume()
                }
            })
        }
    }

    public func declarePublisher(config: PublisherConfig) async throws {
        declaredTopics.insert(config.topic)
        // In actual Zenoh, we'd send a publisher declaration here
    }

    // MARK: - Helpers

    private func parseEndpoint(_ endpoint: String) -> (host: String, port: UInt16)? {
        // Handle formats:
        // tcp://192.168.1.1:7447
        // zenoh://192.168.1.1:7447
        // 192.168.1.1:7447

        let cleaned = endpoint
            .replacingOccurrences(of: "tcp://", with: "")
            .replacingOccurrences(of: "zenoh://", with: "")

        let parts = cleaned.split(separator: ":")
        guard parts.count == 2,
              let port = UInt16(parts[1]) else {
            return nil
        }

        return (host: String(parts[0]), port: port)
    }
}

// MARK: - Future Zenoh Protocol Implementation Notes

/*
 Zenoh Protocol Overview (for future implementation):

 1. Session Establishment:
    - SCOUT: Discovery message
    - HELLO: Response to SCOUT
    - INIT: Session initialization
    - OPEN: Session opened

 2. Publishing:
    - DECLARE: Declare publisher for key expression
    - PUT: Publish data to key expression

 3. Key Expressions:
    - ROS2 topics map to Zenoh key expressions
    - Format: "rt/<topic_name>" for raw topics
    - Or use ROS2-specific conventions

 4. Data Format:
    - CDR serialized ROS2 messages (which we already have)
    - Zenoh adds its own framing

 Resources:
 - Zenoh Protocol Spec: https://github.com/eclipse-zenoh/roadmap/blob/main/rfcs/ALL/Zenoh%20Protocol.md
 - zenoh-pico source: https://github.com/eclipse-zenoh/zenoh-pico
 - ROS2 rmw_zenoh: https://github.com/ros2/rmw_zenoh

 Implementation Strategy:
 1. First attempt: Compile zenoh-pico for iOS and wrap in Swift
 2. Fallback: Implement minimal Zenoh protocol subset in Swift
 3. Bridge option: Simple TCP/UDP bridge that republishes via Zenoh on host

 Current implementation uses a simple framed protocol that requires
 a bridge on the receiving end to convert to Zenoh/ROS2.
 */
