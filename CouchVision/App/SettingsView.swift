import SwiftUI

struct SettingsView: View {
    @EnvironmentObject var coordinator: SensorCoordinator
    @Environment(\.dismiss) private var dismiss

    @State private var selectedCamera: CameraType = .backWide
    @State private var cameraResolution: CameraConfig.Resolution = .p720
    @State private var cameraFrameRate: Int = 30
    @State private var jpegQuality: Double = 80
    @State private var generatePointCloud: Bool = true
    @State private var publishConfidence: Bool = false
    @State private var pointCloudDownsample: Int = 4
    @State private var imuRate: Int = 100
    @State private var topicPrefix: String = "/iphone"
    @State private var endpoint: String = "tcp://192.168.1.1:7447"

    var body: some View {
        NavigationStack {
            Form {
                cameraSection
                lidarSection
                imuSection
                connectionSection
                aboutSection
            }
            .navigationTitle("Settings")
            .navigationBarTitleDisplayMode(.inline)
            .toolbar {
                ToolbarItem(placement: .topBarLeading) {
                    Button("Cancel") { dismiss() }
                }
                ToolbarItem(placement: .topBarTrailing) {
                    Button("Save") {
                        applyAndPersistSettings()
                        dismiss()
                    }
                    .fontWeight(.semibold)
                }
            }
            .onAppear { loadSettings() }
        }
    }

    // MARK: - Sections

    private var cameraSection: some View {
        Section("Camera") {
            if coordinator.cameraManager.availableCameras.count > 1 {
                Picker("Camera", selection: $selectedCamera) {
                    ForEach(coordinator.cameraManager.availableCameras, id: \.self) { camera in
                        Text(camera.displayName).tag(camera)
                    }
                }
            } else if let camera = coordinator.cameraManager.availableCameras.first {
                LabeledContent("Camera", value: camera.displayName)
            }
            Picker("Resolution", selection: $cameraResolution) {
                Text("1080p").tag(CameraConfig.Resolution.p1080)
                Text("720p").tag(CameraConfig.Resolution.p720)
                Text("480p").tag(CameraConfig.Resolution.p480)
            }
            Picker("Frame Rate", selection: $cameraFrameRate) {
                Text("30 fps").tag(30)
                Text("15 fps").tag(15)
                Text("10 fps").tag(10)
            }
            VStack(alignment: .leading) {
                Text("JPEG Quality: \(Int(jpegQuality))%")
                Slider(value: $jpegQuality, in: 50 ... 100, step: 5)
            }
        }
    }

    private var lidarSection: some View {
        Section("LiDAR") {
            Toggle("Generate Point Cloud", isOn: $generatePointCloud)
            Toggle("Publish Confidence Map", isOn: $publishConfidence)
            Picker("Point Cloud Downsample", selection: $pointCloudDownsample) {
                Text("Full (1x)").tag(1)
                Text("2x").tag(2)
                Text("4x").tag(4)
                Text("8x").tag(8)
            }
        }
    }

    private var imuSection: some View {
        Section("IMU") {
            Picker("Update Rate", selection: $imuRate) {
                Text("100 Hz").tag(100)
                Text("50 Hz").tag(50)
                Text("25 Hz").tag(25)
            }
        }
    }

    private var connectionSection: some View {
        Section("Connection") {
            TextField("Topic Prefix", text: $topicPrefix)
                .autocorrectionDisabled()
                .textInputAutocapitalization(.never)
            TextField("Endpoint", text: $endpoint)
                .autocorrectionDisabled()
                .textInputAutocapitalization(.never)
                .font(.system(.body, design: .monospaced))
        }
    }

    private var aboutSection: some View {
        Section("About") {
            LabeledContent("Version", value: "1.0.0")
            LabeledContent("Build", value: "1")
        }
    }

    // MARK: - Settings Logic

    private func loadSettings() {
        // Load from persisted storage first, then overlay live coordinator state
        selectedCamera = coordinator.cameraManager.selectedCamera
        cameraResolution = SettingsStorage.cameraResolution
        cameraFrameRate = SettingsStorage.cameraFrameRate
        jpegQuality = SettingsStorage.jpegQuality
        generatePointCloud = SettingsStorage.generatePointCloud
        publishConfidence = SettingsStorage.publishConfidence
        pointCloudDownsample = SettingsStorage.pointCloudDownsample
        imuRate = SettingsStorage.imuRate
        topicPrefix = SettingsStorage.topicPrefix
        endpoint = SettingsStorage.endpoint

        // Use coordinator's live state if different (handles runtime changes)
        if coordinator.config.endpoint != endpoint {
            endpoint = coordinator.config.endpoint
        }
        if coordinator.config.topicPrefix != topicPrefix {
            topicPrefix = coordinator.config.topicPrefix
        }
    }

    private func applyAndPersistSettings() {
        // Persist to UserDefaults
        SettingsStorage.selectedCamera = selectedCamera
        SettingsStorage.cameraResolution = cameraResolution
        SettingsStorage.cameraFrameRate = cameraFrameRate
        SettingsStorage.jpegQuality = jpegQuality
        SettingsStorage.generatePointCloud = generatePointCloud
        SettingsStorage.publishConfidence = publishConfidence
        SettingsStorage.pointCloudDownsample = pointCloudDownsample
        SettingsStorage.imuRate = imuRate
        SettingsStorage.topicPrefix = topicPrefix
        SettingsStorage.endpoint = endpoint

        // Apply to live coordinator
        coordinator.cameraManager.selectedCamera = selectedCamera
        coordinator.cameraManager.config = CameraConfig(
            resolution: cameraResolution,
            frameRate: cameraFrameRate,
            jpegQuality: CGFloat(jpegQuality / 100)
        )
        coordinator.lidarManager.config = LiDARConfig(
            generatePointCloud: generatePointCloud,
            publishConfidence: publishConfidence,
            pointCloudDownsample: pointCloudDownsample
        )
        coordinator.motionManager.config = MotionConfig(
            imuRate: SensorRateConfig(frequencyHz: Double(imuRate))
        )
        coordinator.config = CoordinatorConfig(
            topicPrefix: topicPrefix,
            endpoint: endpoint
        )
    }
}

#Preview {
    SettingsView()
        .environmentObject(SensorCoordinator())
}
