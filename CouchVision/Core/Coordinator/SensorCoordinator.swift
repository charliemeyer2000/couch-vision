import AVFoundation
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

    private var sensorsRunningBeforeBackground = (
        camera: false, lidar: false, motion: false,
        location: false, environment: false, deviceStatus: false
    )

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
    public let locationManager: LocationManager
    public let environmentManager: EnvironmentManager
    public let deviceStatusManager: DeviceStatusManager

    private var backgroundAudioPlayer: AVAudioPlayer?

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

        locationManager = LocationManager()
        environmentManager = EnvironmentManager()
        deviceStatusManager = DeviceStatusManager()

        setupSubscriptions()
        forwardManagerChanges()
        setupBackgroundAudio()
    }

    private func setupBackgroundAudio() {
        do {
            let audioSession = AVAudioSession.sharedInstance()
            try audioSession.setCategory(.playback, mode: .default, options: .mixWithOthers)
            try audioSession.setActive(true)
        } catch {
            Log.app.error("Failed to setup background audio: \(error.localizedDescription)")
        }
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

        locationManager.objectWillChange
            .receive(on: RunLoop.main)
            .sink { [weak self] _ in self?.objectWillChange.send() }
            .store(in: &cancellables)

        environmentManager.objectWillChange
            .receive(on: RunLoop.main)
            .sink { [weak self] _ in self?.objectWillChange.send() }
            .store(in: &cancellables)

        deviceStatusManager.objectWillChange
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
        if locationManager.isEnabled {
            try? locationManager.start()
        }
        if environmentManager.isEnabled {
            try? environmentManager.start()
        }
        if deviceStatusManager.isEnabled {
            deviceStatusManager.start()
        }
        LiveActivityManager.shared.startActivity(coordinator: self)
    }

    public func stopAllSensors() {
        cameraManager.stop()
        lidarManager.stop()
        motionManager.stop()
        locationManager.stop()
        environmentManager.stop()
        deviceStatusManager.stop()
        LiveActivityManager.shared.endActivity()
    }

    public func enterBackground() {
        isInBackground = true
        sensorsRunningBeforeBackground = (
            camera: cameraManager.state == .running,
            lidar: lidarManager.state == .running,
            motion: motionManager.state == .running,
            location: locationManager.state == .running,
            environment: environmentManager.state == .running,
            deviceStatus: deviceStatusManager.state == .running
        )
        // Stop camera and LiDAR (ARKit doesn't work in background)
        if sensorsRunningBeforeBackground.lidar {
            lidarManager.stop()
        }
        if sensorsRunningBeforeBackground.camera {
            cameraManager.stop()
        }
        // IMU, GPS, environment sensors, and device status continue in background
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
        async let location = locationManager.requestPermissions()
        async let environment = environmentManager.requestPermissions()
        async let deviceStatus = deviceStatusManager.requestPermissions()

        let results = await [camera, lidar, motion, location, environment, deviceStatus]
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

        locationManager.locationPublisher
            .sink { [weak self] data in
                Task { await self?.publishLocation(data) }
            }
            .store(in: &cancellables)

        locationManager.velocityPublisher
            .sink { [weak self] data in
                Task { await self?.publishVelocity(data) }
            }
            .store(in: &cancellables)

        environmentManager.magneticFieldPublisher
            .sink { [weak self] data in
                Task { await self?.publishMagneticField(data) }
            }
            .store(in: &cancellables)

        environmentManager.pressurePublisher
            .sink { [weak self] data in
                Task { await self?.publishPressure(data) }
            }
            .store(in: &cancellables)

        environmentManager.altitudePublisher
            .sink { [weak self] data in
                Task { await self?.publishAltitude(data) }
            }
            .store(in: &cancellables)

        deviceStatusManager.batteryPublisher
            .sink { [weak self] data in
                Task { await self?.publishBattery(data) }
            }
            .store(in: &cancellables)

        deviceStatusManager.thermalPublisher
            .sink { [weak self] data in
                Task { await self?.publishThermal(data) }
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

    private func publishLocation(_ data: TimestampedData<NavSatFix>) async {
        guard isConnected, let publisher else { return }

        let topic = "\(config.topicPrefix)/gps/fix"
        let encoded = CDREncoder.encode(data.data)

        do {
            try await publisher.publish(encoded, to: topic)
            updateStats(topic: topic, bytes: encoded.count)
        } catch {
            Log.ros.error("Failed to publish GPS: \(error.localizedDescription)")
        }
    }

    private func publishVelocity(_ data: TimestampedData<TwistStamped>) async {
        guard isConnected, let publisher else { return }

        let topic = "\(config.topicPrefix)/gps/velocity"
        let encoded = CDREncoder.encode(data.data)

        do {
            try await publisher.publish(encoded, to: topic)
            updateStats(topic: topic, bytes: encoded.count)
        } catch {
            Log.ros.error("Failed to publish velocity: \(error.localizedDescription)")
        }
    }

    private func publishMagneticField(_ data: TimestampedData<MagneticField>) async {
        guard isConnected, let publisher else { return }

        let topic = "\(config.topicPrefix)/magnetic_field"
        let encoded = CDREncoder.encode(data.data)

        do {
            try await publisher.publish(encoded, to: topic)
            updateStats(topic: topic, bytes: encoded.count)
        } catch {
            Log.ros.error("Failed to publish magnetic field: \(error.localizedDescription)")
        }
    }

    private func publishPressure(_ data: TimestampedData<FluidPressure>) async {
        guard isConnected, let publisher else { return }

        let topic = "\(config.topicPrefix)/pressure"
        let encoded = CDREncoder.encode(data.data)

        do {
            try await publisher.publish(encoded, to: topic)
            updateStats(topic: topic, bytes: encoded.count)
        } catch {
            Log.ros.error("Failed to publish pressure: \(error.localizedDescription)")
        }
    }

    private func publishAltitude(_ data: TimestampedData<Float64Msg>) async {
        guard isConnected, let publisher else { return }

        let topic = "\(config.topicPrefix)/altitude"
        let encoded = CDREncoder.encode(data.data)

        do {
            try await publisher.publish(encoded, to: topic)
            updateStats(topic: topic, bytes: encoded.count)
        } catch {
            Log.ros.error("Failed to publish altitude: \(error.localizedDescription)")
        }
    }

    private func publishBattery(_ data: TimestampedData<BatteryState>) async {
        guard isConnected, let publisher else { return }

        let topic = "\(config.topicPrefix)/battery"
        let encoded = CDREncoder.encode(data.data)

        do {
            try await publisher.publish(encoded, to: topic)
            updateStats(topic: topic, bytes: encoded.count)
        } catch {
            Log.ros.error("Failed to publish battery: \(error.localizedDescription)")
        }
    }

    private func publishThermal(_ data: TimestampedData<Int32Msg>) async {
        guard isConnected, let publisher else { return }

        let topic = "\(config.topicPrefix)/thermal"
        let encoded = CDREncoder.encode(data.data)

        do {
            try await publisher.publish(encoded, to: topic)
            updateStats(topic: topic, bytes: encoded.count)
        } catch {
            Log.ros.error("Failed to publish thermal: \(error.localizedDescription)")
        }
    }

    private func updateStats(topic: String, bytes: Int) {
        if stats[topic] == nil {
            stats[topic] = TopicStats(topic: topic)
        }
        stats[topic]?.recordPublish(bytes: bytes)
    }
}
