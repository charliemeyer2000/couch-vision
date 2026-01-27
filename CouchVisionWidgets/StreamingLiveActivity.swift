import ActivityKit
import SwiftUI
import WidgetKit

private extension UInt64 {
    var formattedBytes: String {
        ByteCountFormatter.string(fromByteCount: Int64(self), countStyle: .binary)
    }
}

struct StreamingLiveActivity: Widget {
    var body: some WidgetConfiguration {
        ActivityConfiguration(for: StreamingActivityAttributes.self) { context in
            LockScreenView(state: context.state)
        } dynamicIsland: { context in
            DynamicIsland {
                DynamicIslandExpandedRegion(.leading) {
                    ConnectionStatusView(state: context.state)
                }
                DynamicIslandExpandedRegion(.trailing) {
                    StreamingRateView(state: context.state)
                }
                DynamicIslandExpandedRegion(.center) {
                    SensorStatusRow(state: context.state)
                }
                DynamicIslandExpandedRegion(.bottom) {
                    Text(context.state.bytesSent.formattedBytes)
                        .font(.caption2)
                        .foregroundStyle(.secondary)
                }
            } compactLeading: {
                CompactLeadingView(state: context.state)
            } compactTrailing: {
                CompactTrailingView(state: context.state)
            } minimal: {
                MinimalView(state: context.state)
            }
        }
    }
}

// MARK: - Connection Status

private struct ConnectionStatusView: View {
    let state: StreamingActivityAttributes.ContentState

    var body: some View {
        HStack(spacing: 4) {
            Image(systemName: state.isConnected ? "wifi" : "wifi.slash")
                .foregroundStyle(state.isConnected ? .green : .red)
            Text(statusText)
                .font(.caption2)
        }
    }

    private var statusText: String {
        if !state.isConnected {
            return "Disconnected"
        }
        return state.isStreaming ? "Streaming" : "Connected"
    }
}

// MARK: - Streaming Rate

private struct StreamingRateView: View {
    let state: StreamingActivityAttributes.ContentState

    var body: some View {
        if state.isConnected, state.isStreaming {
            Text(String(format: "%.0f Hz", state.messageRate))
                .font(.caption)
                .fontWeight(.semibold)
        } else {
            Text("--")
                .font(.caption)
                .foregroundStyle(.secondary)
        }
    }
}

// MARK: - Sensor Status Row

private struct SensorStatusRow: View {
    let state: StreamingActivityAttributes.ContentState

    var body: some View {
        HStack(spacing: 6) {
            SensorIndicator(name: "CAM", active: state.cameraActive && state.isConnected)
            SensorIndicator(name: "LiDAR", active: state.lidarActive && state.isConnected)
            SensorIndicator(name: "IMU", active: state.imuActive && state.isConnected)
            SensorIndicator(name: "GPS", active: state.gpsActive && state.isConnected)
        }
    }
}

// MARK: - Compact Views

private struct CompactLeadingView: View {
    let state: StreamingActivityAttributes.ContentState

    var body: some View {
        Image(systemName: iconName)
            .foregroundStyle(iconColor)
    }

    private var iconName: String {
        state.isConnected
            ? "antenna.radiowaves.left.and.right"
            : "antenna.radiowaves.left.and.right.slash"
    }

    private var iconColor: Color {
        if !state.isConnected { return .red }
        return state.isStreaming ? .green : .yellow
    }
}

private struct CompactTrailingView: View {
    let state: StreamingActivityAttributes.ContentState

    var body: some View {
        if state.isConnected, state.isStreaming {
            Text(String(format: "%.0f", state.messageRate))
                .font(.caption2)
                .fontWeight(.semibold)
        } else {
            Text("--")
                .font(.caption2)
                .foregroundStyle(.secondary)
        }
    }
}

private struct MinimalView: View {
    let state: StreamingActivityAttributes.ContentState

    var body: some View {
        Image(systemName: "antenna.radiowaves.left.and.right")
            .foregroundStyle(minimalColor)
    }

    private var minimalColor: Color {
        if !state.isConnected { return .red }
        return state.isStreaming ? .green : .yellow
    }
}

// MARK: - Lock Screen View

struct LockScreenView: View {
    let state: StreamingActivityAttributes.ContentState

    var body: some View {
        HStack {
            VStack(alignment: .leading, spacing: 4) {
                HStack(spacing: 6) {
                    Image(systemName: state.isConnected ? "wifi" : "wifi.slash")
                        .foregroundStyle(state.isConnected ? .green : .red)
                    Text("CouchVision")
                        .font(.headline)
                    if !state.isConnected {
                        Text("Disconnected")
                            .font(.caption)
                            .foregroundStyle(.red)
                    } else if state.isStreaming {
                        Text("Streaming")
                            .font(.caption)
                            .foregroundStyle(.green)
                    }
                }
                SensorStatusRow(state: state)
            }
            Spacer()
            VStack(alignment: .trailing, spacing: 4) {
                if state.isConnected, state.isStreaming {
                    Text(String(format: "%.0f Hz", state.messageRate))
                        .font(.title2)
                        .fontWeight(.semibold)
                    Text(state.bytesSent.formattedBytes)
                        .font(.caption)
                        .foregroundStyle(.secondary)
                } else {
                    Text("--")
                        .font(.title2)
                        .foregroundStyle(.secondary)
                    Text("Not streaming")
                        .font(.caption)
                        .foregroundStyle(.secondary)
                }
            }
        }
        .padding()
    }
}

// MARK: - Sensor Indicator

struct SensorIndicator: View {
    let name: String
    let active: Bool

    var body: some View {
        HStack(spacing: 2) {
            Circle()
                .fill(active ? Color.green : Color.gray.opacity(0.5))
                .frame(width: 6, height: 6)
            Text(name)
                .font(.caption2)
                .foregroundStyle(active ? .primary : .secondary)
        }
    }
}
