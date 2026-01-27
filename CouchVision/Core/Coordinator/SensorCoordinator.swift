import Combine
import Foundation

public struct CoordinatorConfig {
    public let topicPrefix: String
    public let endpoint: String
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

        recentTimes.append(now)
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

@MainActor
public final class SensorCoordinator: ObservableObject {
    @Published public private(set) var isConnected: Bool = false
    @Published public private(set) var connectionError: String?
    @Published public private(set) var stats: [String: TopicStats] = [:]
    @Published public private(set) var isInBackground: Bool = false

    private var sensorsRunningBeforeBackground = (camera: false, lidar: false, motion: false)

    @Published public var config: CoordinatorConfig {
        didSet {
            // Reconnect if endpoint changed
            if oldValue.endpoint != config.endpoint, isConnected {
                Task { await reconnect() }
            }
        }
    }

    private var publisher: Publisher?
    private var cancellables = Set<AnyCancellable>()

    public let cameraManager: CameraManager
    public let lidarManager: LiDARManager
    public let motionManager: MotionManager

    public init(config: CoordinatorConfig? = nil) {
        self.config = config ?? CoordinatorConfig(
            topicPrefix: SettingsStorage.topicPrefix,
            endpoint: SettingsStorage.endpoint
        )

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

    public func disconnect() async {
        await publisher?.disconnect()
        isConnected = false
    }

    public func reconnect() async {
        await disconnect()
        try? await Task.sleep(nanoseconds: 100_000_000) // 100ms
        await connect()
    }

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
        LiveActivityManager.shared.startActivity(coordinator: self)
    }

    public func stopAllSensors() {
        cameraManager.stop()
        lidarManager.stop()
        motionManager.stop()
        LiveActivityManager.shared.endActivity()
    }

    public func enterBackground() {
        isInBackground = true
        sensorsRunningBeforeBackground = (
            camera: cameraManager.state == .running,
            lidar: lidarManager.state == .running,
            motion: motionManager.state == .running
        )
        if sensorsRunningBeforeBackground.lidar {
            lidarManager.stop()
        }
        if sensorsRunningBeforeBackground.camera {
            cameraManager.stop()
        }
        // IMU continues - CoreMotion works in background
    }

    public func enterForeground() {
        isInBackground = false
        if sensorsRunningBeforeBackground.lidar, lidarManager.isEnabled {
            try? lidarManager.start()
        }
        if sensorsRunningBeforeBackground.camera, cameraManager.isEnabled {
            try? cameraManager.start()
        }
    }

    public func requestAllPermissions() async -> Bool {
        async let camera = cameraManager.requestPermissions()
        async let lidar = lidarManager.requestPermissions()
        async let motion = motionManager.requestPermissions()

        let results = await [camera, lidar, motion]
        return results.allSatisfy { $0 }
    }

    private func setupSubscriptions() {
        cameraManager.dataPublisher
            .sink { [weak self] data in
                Task { await self?.publishCameraFrame(data) }
            }
            .store(in: &cancellables)

        lidarManager.dataPublisher
            .sink { [weak self] data in
                Task { await self?.publishLiDARData(data) }
            }
            .store(in: &cancellables)

        motionManager.imuPublisher
            .sink { [weak self] data in
                Task { await self?.publishIMU(data) }
            }
            .store(in: &cancellables)
    }

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
