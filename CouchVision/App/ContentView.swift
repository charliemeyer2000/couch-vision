import SwiftUI

struct ContentView: View {
    @EnvironmentObject var coordinator: SensorCoordinator
    @State private var showSettings = false
    @State private var endpointText: String = ""

    private var isStreaming: Bool {
        coordinator.cameraManager.state == .running ||
            coordinator.lidarManager.state == .running ||
            coordinator.motionManager.state == .running ||
            coordinator.locationManager.state == .running ||
            coordinator.environmentManager.state == .running ||
            coordinator.deviceStatusManager.state == .running
    }

    var body: some View {
        NavigationStack {
            ScrollView {
                VStack(spacing: 20) {
                    connectionStatusView
                    Divider()
                    sensorControlsView
                    Divider()
                    statsView
                }
                .padding()
            }
            .scrollDismissesKeyboard(.interactively)
            .safeAreaInset(edge: .bottom) {
                actionButtonsView
                    .padding()
                    .background(.regularMaterial)
            }
            .navigationTitle("CouchVision")
            .navigationBarTitleDisplayMode(.large)
            .toolbar {
                ToolbarItem(placement: .topBarTrailing) {
                    Button { showSettings = true } label: {
                        Image(systemName: "gear")
                    }
                }
            }
            .sheet(
                isPresented: $showSettings,
                onDismiss: { endpointText = coordinator.config.endpoint },
                content: { SettingsView() }
            )
            .task {
                _ = await coordinator.requestAllPermissions()
            }
            .onAppear {
                endpointText = coordinator.config.endpoint
            }
        }
    }

    private var connectionStatusView: some View {
        VStack(spacing: 12) {
            HStack {
                Circle()
                    .fill(coordinator.isConnected ? Color.green : Color.red)
                    .frame(width: 12, height: 12)
                Text(coordinator.isConnected ? "Connected" : "Disconnected")
                    .font(.headline)
                Spacer()
            }

            if let error = coordinator.connectionError {
                Text(error)
                    .font(.caption)
                    .foregroundColor(.red)
            }

            HStack {
                TextField("Endpoint", text: $endpointText)
                    .textFieldStyle(.roundedBorder)
                    .font(.system(.body, design: .monospaced))
                    .autocorrectionDisabled()
                    .textInputAutocapitalization(.never)
                    .onSubmit { connectToEndpoint() }

                Button("Connect") { connectToEndpoint() }
                    .buttonStyle(.bordered)
                    .disabled(coordinator.isConnected)
            }
        }
        .padding()
        .background(Color(.systemGray6))
        .cornerRadius(12)
    }

    private var sensorControlsView: some View {
        VStack(alignment: .leading, spacing: 16) {
            Text("Sensors")
                .font(.headline)

            SensorToggleRow(
                name: "Camera (Back Wide)",
                state: coordinator.cameraManager.state,
                isEnabled: Binding(
                    get: { coordinator.cameraManager.isEnabled },
                    set: { coordinator.cameraManager.isEnabled = $0 }
                )
            )

            SensorToggleRow(
                name: "LiDAR Depth",
                state: coordinator.lidarManager.state,
                isEnabled: Binding(
                    get: { coordinator.lidarManager.isEnabled },
                    set: { coordinator.lidarManager.isEnabled = $0 }
                )
            )

            SensorToggleRow(
                name: "IMU",
                state: coordinator.motionManager.state,
                isEnabled: Binding(
                    get: { coordinator.motionManager.isEnabled },
                    set: { coordinator.motionManager.isEnabled = $0 }
                )
            )

            SensorToggleRow(
                name: "GPS Location",
                state: coordinator.locationManager.state,
                isEnabled: Binding(
                    get: { coordinator.locationManager.isEnabled },
                    set: { coordinator.locationManager.isEnabled = $0 }
                )
            )

            SensorToggleRow(
                name: "Magnetometer/Barometer",
                state: coordinator.environmentManager.state,
                isEnabled: Binding(
                    get: { coordinator.environmentManager.isEnabled },
                    set: { coordinator.environmentManager.isEnabled = $0 }
                )
            )

            SensorToggleRow(
                name: "Device Status",
                state: coordinator.deviceStatusManager.state,
                isEnabled: Binding(
                    get: { coordinator.deviceStatusManager.isEnabled },
                    set: { coordinator.deviceStatusManager.isEnabled = $0 }
                )
            )
        }
        .padding()
        .background(Color(.systemGray6))
        .cornerRadius(12)
    }

    private var statsView: some View {
        VStack(alignment: .leading, spacing: 12) {
            Text("Publishing Stats")
                .font(.headline)

            if coordinator.stats.isEmpty {
                Text("No data published yet")
                    .font(.caption)
                    .foregroundColor(.secondary)
            } else {
                ForEach(Array(coordinator.stats.keys.sorted()), id: \.self) { topic in
                    if let stat = coordinator.stats[topic] {
                        HStack {
                            Text(topic)
                                .font(.system(.caption, design: .monospaced))
                                .lineLimit(1)
                            Spacer()
                            Text(String(format: "%.1f Hz", stat.averageHz))
                                .font(.caption)
                                .foregroundColor(.secondary)
                            Text(formatBytes(stat.bytesPublished))
                                .font(.caption)
                                .foregroundColor(.secondary)
                        }
                    }
                }
            }
        }
        .padding()
        .background(Color(.systemGray6))
        .cornerRadius(12)
    }

    private var actionButtonsView: some View {
        HStack(spacing: 16) {
            Button {
                coordinator.startAllSensors()
            } label: {
                HStack {
                    if isStreaming {
                        Circle()
                            .fill(.green)
                            .frame(width: 8, height: 8)
                    }
                    Label("Start All", systemImage: "play.fill")
                }
                .frame(maxWidth: .infinity)
            }
            .buttonStyle(.borderedProminent)
            .tint(isStreaming ? .gray : .green)
            .disabled(!coordinator.isConnected || isStreaming)

            Button {
                coordinator.stopAllSensors()
            } label: {
                Label("Stop All", systemImage: "stop.fill")
                    .frame(maxWidth: .infinity)
            }
            .buttonStyle(.borderedProminent)
            .tint(isStreaming ? .red : .gray)
            .disabled(!coordinator.isConnected || !isStreaming)
        }
    }

    private func connectToEndpoint() {
        Task {
            SettingsStorage.endpoint = endpointText
            coordinator.config = CoordinatorConfig(
                topicPrefix: coordinator.config.topicPrefix,
                endpoint: endpointText
            )
            await coordinator.connect()
        }
    }

    private func formatBytes(_ bytes: UInt64) -> String {
        let formatter = ByteCountFormatter()
        formatter.countStyle = .binary
        return formatter.string(fromByteCount: Int64(bytes))
    }
}

struct SensorToggleRow: View {
    let name: String
    let state: SensorState
    @Binding var isEnabled: Bool

    var body: some View {
        HStack {
            VStack(alignment: .leading, spacing: 4) {
                Text(name)
                    .font(.subheadline)
                Text(stateDescription)
                    .font(.caption)
                    .foregroundColor(stateColor)
            }
            Spacer()
            Toggle("", isOn: $isEnabled)
                .labelsHidden()
                .disabled(!canToggle)
        }
    }

    private var canToggle: Bool {
        switch state {
        case .ready,
             .running: true
        default: false
        }
    }

    private var stateDescription: String {
        switch state {
        case .unknown: "Checking..."
        case .unavailable: "Not available"
        case .unauthorized: "Permission required"
        case .ready: "Ready"
        case .running: "Running"
        case .error(let msg): "Error: \(msg)"
        }
    }

    private var stateColor: Color {
        switch state {
        case .running: .green
        case .ready: .blue
        case .error,
             .unauthorized: .red
        case .unavailable: .gray
        case .unknown: .secondary
        }
    }
}

#Preview {
    ContentView()
        .environmentObject(SensorCoordinator())
}
