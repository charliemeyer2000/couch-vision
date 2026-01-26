import SwiftUI

struct ContentView: View {
    @EnvironmentObject var coordinator: SensorCoordinator
    @State private var showSettings = false
    @State private var endpoint = "tcp://192.168.1.1:7447"

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
            .sheet(isPresented: $showSettings) {
                SettingsView()
            }
            .task {
                _ = await coordinator.requestAllPermissions()
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
                TextField("Endpoint", text: $endpoint)
                    .textFieldStyle(.roundedBorder)
                    .font(.system(.body, design: .monospaced))
                    .autocorrectionDisabled()
                    .textInputAutocapitalization(.never)

                Button {
                    Task {
                        coordinator.config = CoordinatorConfig(endpoint: endpoint)
                        await coordinator.connect()
                    }
                } label: {
                    Text("Connect")
                }
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
                Label("Start All", systemImage: "play.fill")
                    .frame(maxWidth: .infinity)
            }
            .buttonStyle(.borderedProminent)
            .disabled(!coordinator.isConnected)

            Button {
                coordinator.stopAllSensors()
            } label: {
                Label("Stop All", systemImage: "stop.fill")
                    .frame(maxWidth: .infinity)
            }
            .buttonStyle(.bordered)
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
        case .ready, .running: return true
        default: return false
        }
    }

    private var stateDescription: String {
        switch state {
        case .unknown: return "Checking..."
        case .unavailable: return "Not available"
        case .unauthorized: return "Permission required"
        case .ready: return "Ready"
        case .running: return "Running"
        case .error(let msg): return "Error: \(msg)"
        }
    }

    private var stateColor: Color {
        switch state {
        case .running: return .green
        case .ready: return .blue
        case .error, .unauthorized: return .red
        case .unavailable: return .gray
        case .unknown: return .secondary
        }
    }
}

#Preview {
    ContentView()
        .environmentObject(SensorCoordinator())
}
