import Foundation

/// Mock publisher for testing without network connectivity
/// Logs all publish calls and simulates connection
public final class MockPublisher: Publisher {

    // MARK: - Publisher Protocol

    public private(set) var isConnected: Bool = false
    public var onConnectionStateChanged: ((Bool) -> Void)?

    // MARK: - Mock State

    /// All messages published (for testing/debugging)
    public private(set) var publishedMessages: [(topic: String, data: Data, timestamp: Date)] = []

    /// Whether to log publishes to console
    public var verbose: Bool = true

    /// Simulated latency in seconds
    public var simulatedLatency: TimeInterval = 0.001

    /// Maximum messages to retain in history
    public var maxHistorySize: Int = 1000

    // MARK: - Stats

    public private(set) var totalMessagesPublished: UInt64 = 0
    public private(set) var totalBytesPublished: UInt64 = 0
    private var topicStats: [String: (count: UInt64, bytes: UInt64)] = [:]

    // MARK: - Initialization

    public init(verbose: Bool = true) {
        self.verbose = verbose
    }

    // MARK: - Publisher Protocol Implementation

    public func connect(to endpoint: String) async throws {
        if verbose {
            print("[MockPublisher] Connecting to: \(endpoint)")
        }

        // Simulate connection delay
        try await Task.sleep(nanoseconds: 100_000_000) // 100ms

        isConnected = true
        onConnectionStateChanged?(true)

        if verbose {
            print("[MockPublisher] Connected (mock)")
        }
    }

    public func disconnect() async {
        if verbose {
            print("[MockPublisher] Disconnecting...")
        }

        isConnected = false
        onConnectionStateChanged?(false)

        if verbose {
            print("[MockPublisher] Disconnected")
        }
    }

    public func publish(_ data: Data, to topic: String) async throws {
        guard isConnected else {
            throw PublisherError.notConnected
        }

        // Simulate network latency
        if simulatedLatency > 0 {
            try await Task.sleep(nanoseconds: UInt64(simulatedLatency * 1_000_000_000))
        }

        // Record the message
        let message = (topic: topic, data: data, timestamp: Date())
        publishedMessages.append(message)

        // Trim history if needed
        if publishedMessages.count > maxHistorySize {
            publishedMessages.removeFirst(publishedMessages.count - maxHistorySize)
        }

        // Update stats
        totalMessagesPublished += 1
        totalBytesPublished += UInt64(data.count)

        if var stats = topicStats[topic] {
            stats.count += 1
            stats.bytes += UInt64(data.count)
            topicStats[topic] = stats
        } else {
            topicStats[topic] = (count: 1, bytes: UInt64(data.count))
        }

        if verbose {
            print("[MockPublisher] Published \(data.count) bytes to \(topic)")
        }
    }

    public func declarePublisher(config: PublisherConfig) async throws {
        if verbose {
            print("[MockPublisher] Declared publisher for: \(config.topic) (QoS: \(config.qos.description))")
        }
    }

    // MARK: - Testing Helpers

    /// Clear all recorded messages
    public func clearHistory() {
        publishedMessages.removeAll()
    }

    /// Get messages for a specific topic
    public func messages(for topic: String) -> [(data: Data, timestamp: Date)] {
        publishedMessages
            .filter { $0.topic == topic }
            .map { (data: $0.data, timestamp: $0.timestamp) }
    }

    /// Get stats for a specific topic
    public func stats(for topic: String) -> (count: UInt64, bytes: UInt64)? {
        topicStats[topic]
    }

    /// Print summary of all published data
    public func printSummary() {
        print("\n=== MockPublisher Summary ===")
        print("Total messages: \(totalMessagesPublished)")
        print("Total bytes: \(formatBytes(totalBytesPublished))")
        print("\nBy topic:")
        for (topic, stats) in topicStats.sorted(by: { $0.key < $1.key }) {
            print("  \(topic): \(stats.count) msgs, \(formatBytes(stats.bytes))")
        }
        print("=============================\n")
    }

    private func formatBytes(_ bytes: UInt64) -> String {
        let formatter = ByteCountFormatter()
        formatter.countStyle = .binary
        return formatter.string(fromByteCount: Int64(bytes))
    }
}
