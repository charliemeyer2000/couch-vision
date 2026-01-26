import SwiftUI

struct SettingsView: View {
    @EnvironmentObject var coordinator: SensorCoordinator
    @Environment(\.dismiss) private var dismiss

    // Camera settings
    @State private var cameraResolution: CameraConfig.Resolution = .p720
    @State private var cameraFrameRate: Int = 30
    @State private var jpegQuality: Double = 80

    // LiDAR settings
    @State private var generatePointCloud: Bool = true
    @State private var publishConfidence: Bool = false
    @State private var pointCloudDownsample: Int = 4

    // IMU settings
    @State private var imuRate: Int = 100

    // Connection settings
    @State private var topicPrefix: String = "/iphone"
    @State private var endpoint: String = "tcp://192.168.1.1:7447"

    var body: some View {
        NavigationStack {
            Form {
                // Camera Section
                Section("Camera") {
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
                        Slider(value: $jpegQuality, in: 50...100, step: 5)
                    }
                }

                // LiDAR Section
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

                // IMU Section
                Section("IMU") {
                    Picker("Update Rate", selection: $imuRate) {
                        Text("100 Hz").tag(100)
                        Text("50 Hz").tag(50)
                        Text("25 Hz").tag(25)
                    }
                }

                // Connection Section
                Section("Connection") {
                    TextField("Topic Prefix", text: $topicPrefix)
                        .autocorrectionDisabled()
                        .textInputAutocapitalization(.never)

                    TextField("Endpoint", text: $endpoint)
                        .autocorrectionDisabled()
                        .textInputAutocapitalization(.never)
                        .font(.system(.body, design: .monospaced))
                }

                // About Section
                Section("About") {
                    HStack {
                        Text("Version")
                        Spacer()
                        Text("1.0.0")
                            .foregroundColor(.secondary)
                    }

                    HStack {
                        Text("Build")
                        Spacer()
                        Text("1")
                            .foregroundColor(.secondary)
                    }
                }
            }
            .navigationTitle("Settings")
            .navigationBarTitleDisplayMode(.inline)
            .toolbar {
                ToolbarItem(placement: .topBarLeading) {
                    Button("Cancel") {
                        dismiss()
                    }
                }

                ToolbarItem(placement: .topBarTrailing) {
                    Button("Save") {
                        applySettings()
                        dismiss()
                    }
                    .fontWeight(.semibold)
                }
            }
            .onAppear {
                loadCurrentSettings()
            }
        }
    }

    private func loadCurrentSettings() {
        // Load camera settings
        cameraResolution = coordinator.cameraManager.config.resolution
        cameraFrameRate = coordinator.cameraManager.config.frameRate
        jpegQuality = Double(coordinator.cameraManager.config.jpegQuality * 100)

        // Load LiDAR settings
        generatePointCloud = coordinator.lidarManager.config.generatePointCloud
        publishConfidence = coordinator.lidarManager.config.publishConfidence
        pointCloudDownsample = coordinator.lidarManager.config.pointCloudDownsample

        // Load IMU settings
        imuRate = Int(coordinator.motionManager.config.imuRate.frequencyHz)

        // Load connection settings
        topicPrefix = coordinator.config.topicPrefix
        endpoint = coordinator.config.endpoint
    }

    private func applySettings() {
        // Apply camera settings
        coordinator.cameraManager.config = CameraConfig(
            resolution: cameraResolution,
            frameRate: cameraFrameRate,
            jpegQuality: CGFloat(jpegQuality / 100)
        )

        // Apply LiDAR settings
        coordinator.lidarManager.config = LiDARConfig(
            generatePointCloud: generatePointCloud,
            publishConfidence: publishConfidence,
            pointCloudDownsample: pointCloudDownsample
        )

        // Apply IMU settings
        coordinator.motionManager.config = MotionConfig(
            imuRate: SensorRateConfig(frequencyHz: Double(imuRate))
        )

        // Apply connection settings
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
