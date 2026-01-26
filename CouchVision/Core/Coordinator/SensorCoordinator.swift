import Combine
import Foundation

/// Configuration for the sensor coordinator
public struct CoordinatorConfig {
    /// Topic prefix (e.g., "/iphone")
    public let topicPrefix: String
    /// Target endpoint for publishing
    public let endpoint: String
    /// Default QoS
    public let defaultQoS: QoS

    public init(
        topicPrefix: String = "/iphone",
        endpoint: String = "tcp://192.168.1.1:7447",
        defaultQoS: QoS = .bestEffort
    ) {
        self.topicPrefix = topicPrefix
        self.endpoint = endpoint
        self.defaultQoS = defaultQoS
    }
}

/// Statistics for a single topic
public struct TopicStats {
    public let topic: String
    public var messageCount: UInt64 = 0
    public var bytesPublished: UInt64 = 0
    public var lastPublishTime: Date?
    public var averageHz: Double = 0

    private var recentTimes: [Date] = []

    public init(topic: String) {
        self.topic = topic
    }

    mutating func recordPublish(bytes: Int) {
        messageCount += 1
        bytesPublished += UInt64(bytes)
        let now = Date()
        lastPublishTime = now

        // Calculate Hz from recent publishes
        recentTimes.append(now)
        // Keep last 100 samples
        if recentTimes.count > 100 {
            recentTimes.removeFirst()
        }
        if recentTimes.count >= 2 {
            let duration = recentTimes.last!.timeIntervalSince(recentTimes.first!)
            if duration > 0 {
                averageHz = Double(recentTimes.count - 1) / duration
            }
        }
    }
}

/// Central coordinator that manages sensors and routes data to publishers
@MainActor
public final class SensorCoordinator: ObservableObject {

    // MARK: - Published State

    @Published public private(set) var isConnected: Bool = false
    @Published public private(set) var connectionError: String?
    @Published public private(set) var stats: [String: TopicStats] = [:]

    // MARK: - Configuration

    @Published public var config: CoordinatorConfig {
        didSet {
            // Reconnect if endpoint changed
            if oldValue.endpoint != config.endpoint, isConnected {
                Task { await reconnect() }
            }
        }
    }

    // MARK: - Dependencies

    private var publisher: Publisher?
    private var cancellables = Set<AnyCancellable>()

    // MARK: - Sensor Managers

    public let cameraManager: CameraManager
    public let lidarManager: LiDARManager
    public let motionManager: MotionManager
    // Additional managers can be added here

    // MARK: - Initialization

    public init(config: CoordinatorConfig? = nil) {
        // Load persisted settings or use provided config
        self.config = config ?? CoordinatorConfig(
            topicPrefix: SettingsStorage.topicPrefix,
            endpoint: SettingsStorage.endpoint
        )

        // Initialize sensor managers with persisted configs
        cameraManager = CameraManager()
        cameraManager.config = CameraConfig(
            resolution: SettingsStorage.cameraResolution,
            frameRate: SettingsStorage.cameraFrameRate,
            jpegQuality: CGFloat(SettingsStorage.jpegQuality / 100)
        )

        lidarManager = LiDARManager()
        lidarManager.config = LiDARConfig(
            generatePointCloud: SettingsStorage.generatePointCloud,
            publishConfidence: SettingsStorage.publishConfidence,
            pointCloudDownsample: SettingsStorage.pointCloudDownsample
        )

        motionManager = MotionManager()
        motionManager.config = MotionConfig(
            imuRate: SensorRateConfig(frequencyHz: Double(SettingsStorage.imuRate))
        )

        setupSubscriptions()
        forwardManagerChanges()
    }

    /// Forward objectWillChange from child managers to coordinator
    /// This ensures SwiftUI views update when manager state changes
    private func forwardManagerChanges() {
        cameraManager.objectWillChange
            .receive(on: RunLoop.main)
            .sink { [weak self] _ in self?.objectWillChange.send() }
            .store(in: &cancellables)

        lidarManager.objectWillChange
            .receive(on: RunLoop.main)
            .sink { [weak self] _ in self?.objectWillChange.send() }
            .store(in: &cancellables)

        motionManager.objectWillChange
            .receive(on: RunLoop.main)
            .sink { [weak self] _ in self?.objectWillChange.send() }
            .store(in: &cancellables)
    }

    // MARK: - Publisher Management

    /// Set the publisher implementation to use
    public func setPublisher(_ publisher: Publisher) {
        self.publisher = publisher
        publisher.onConnectionStateChanged = { [weak self] connected in
            Task { @MainActor in
                self?.isConnected = connected
                if !connected {
                    self?.connectionError = "Disconnected"
                }
            }
        }
    }

    /// Connect to the configured endpoint
    public func connect() async {
        guard let publisher else {
            connectionError = "No publisher configured"
            return
        }

        do {
            connectionError = nil
            try await publisher.connect(to: config.endpoint)
            isConnected = true
        } catch {
            connectionError = error.localizedDescription
            isConnected = false
        }
    }

    /// Disconnect from the endpoint
    public func disconnect() async {
        await publisher?.disconnect()
        isConnected = false
    }

    /// Reconnect (disconnect then connect)
    public func reconnect() async {
        await disconnect()
        try? await Task.sleep(nanoseconds: 100_000_000) // 100ms
        await connect()
    }

    // MARK: - Sensor Control

    /// Start all enabled sensors
    public func startAllSensors() {
        if cameraManager.isEnabled {
            try? cameraManager.start()
        }
        if lidarManager.isEnabled {
            try? lidarManager.start()
        }
        if motionManager.isEnabled {
            try? motionManager.start()
        }
    }

    /// Stop all sensors
    public func stopAllSensors() {
        cameraManager.stop()
        lidarManager.stop()
        motionManager.stop()
    }

    /// Request permissions for all sensors
    public func requestAllPermissions() async -> Bool {
        async let camera = cameraManager.requestPermissions()
        async let lidar = lidarManager.requestPermissions()
        async let motion = motionManager.requestPermissions()

        let results = await [camera, lidar, motion]
        return results.allSatisfy { $0 }
    }

    // MARK: - Private

    private func setupSubscriptions() {
        // Subscribe to camera data
        cameraManager.dataPublisher
            .sink { [weak self] data in
                Task { await self?.publishCameraFrame(data) }
            }
            .store(in: &cancellables)

        // Subscribe to LiDAR data
        lidarManager.dataPublisher
            .sink { [weak self] data in
                Task { await self?.publishLiDARData(data) }
            }
            .store(in: &cancellables)

        // Subscribe to IMU data
        motionManager.imuPublisher
            .sink { [weak self] data in
                Task { await self?.publishIMU(data) }
            }
            .store(in: &cancellables)
    }

    // MARK: - Publishing

    private func publishCameraFrame(_ frame: TimestampedData<CameraFrame>) async {
        guard isConnected, let publisher else { return }

        let topic = "\(config.topicPrefix)/camera/\(frame.data.cameraId)/image/compressed"

        let msg = CompressedImage.jpeg(
            data: frame.data.jpegData,
            frameId: frame.frameId,
            timestamp: frame.timestamp
        )
        let encoded = CDREncoder.encode(msg)

        do {
            try await publisher.publish(encoded, to: topic)
            updateStats(topic: topic, bytes: encoded.count)
        } catch {
            Log.ros.error("Failed to publish camera frame: \(error.localizedDescription)")
        }
    }

    private func publishLiDARData(_ data: TimestampedData<LiDARData>) async {
        guard isConnected, let publisher else { return }

        if let depthImage = data.data.depthImage {
            let depthTopic = "\(config.topicPrefix)/lidar/depth/image"
            let encoded = CDREncoder.encode(depthImage)
            do {
                try await publisher.publish(encoded, to: depthTopic)
                updateStats(topic: depthTopic, bytes: encoded.count)
            } catch {
                Log.ros.error("Failed to publish depth image: \(error.localizedDescription)")
            }
        }

        if let pointCloud = data.data.pointCloud {
            let cloudTopic = "\(config.topicPrefix)/lidar/points"
            let encoded = CDREncoder.encode(pointCloud)
            do {
                try await publisher.publish(encoded, to: cloudTopic)
                updateStats(topic: cloudTopic, bytes: encoded.count)
            } catch {
                Log.ros.error("Failed to publish point cloud: \(error.localizedDescription)")
            }
        }
    }

    private func publishIMU(_ data: TimestampedData<ImuMessage>) async {
        guard isConnected, let publisher else { return }

        let topic = "\(config.topicPrefix)/imu"
        let encoded = CDREncoder.encode(data.data)

        do {
            try await publisher.publish(encoded, to: topic)
            updateStats(topic: topic, bytes: encoded.count)
        } catch {
            Log.ros.error("Failed to publish IMU: \(error.localizedDescription)")
        }
    }

    private func updateStats(topic: String, bytes: Int) {
        if stats[topic] == nil {
            stats[topic] = TopicStats(topic: topic)
        }
        stats[topic]?.recordPublish(bytes: bytes)
    }
}
