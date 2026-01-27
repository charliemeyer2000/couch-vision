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
            LockScreenView(context: context)
        } dynamicIsland: { context in
            DynamicIsland {
                DynamicIslandExpandedRegion(.leading) {
                    HStack(spacing: 4) {
                        Image(systemName: context.state.isConnected ? "wifi" : "wifi.slash")
                            .foregroundStyle(context.state.isConnected ? .green : .red)
                        Text(context.state.isConnected ? "Connected" : "Disconnected")
                            .font(.caption2)
                    }
                }
                DynamicIslandExpandedRegion(.trailing) {
                    Text(String(format: "%.0f Hz", context.state.messageRate))
                        .font(.caption)
                        .fontWeight(.semibold)
                }
                DynamicIslandExpandedRegion(.center) {
                    HStack(spacing: 8) {
                        SensorIndicator(name: "CAM", active: context.state.cameraActive)
                        SensorIndicator(name: "LiDAR", active: context.state.lidarActive)
                        SensorIndicator(name: "IMU", active: context.state.imuActive)
                    }
                }
                DynamicIslandExpandedRegion(.bottom) {
                    Text(context.state.bytesSent.formattedBytes)
                        .font(.caption2)
                        .foregroundStyle(.secondary)
                }
            } compactLeading: {
                Image(systemName: context.state
                    .isConnected ? "antenna.radiowaves.left.and.right" : "antenna.radiowaves.left.and.right.slash")
                    .foregroundStyle(context.state.isConnected ? .green : .red)
            } compactTrailing: {
                Text(String(format: "%.0f", context.state.messageRate))
                    .font(.caption2)
                    .fontWeight(.semibold)
            } minimal: {
                Image(systemName: "antenna.radiowaves.left.and.right")
                    .foregroundStyle(context.state.isConnected ? .green : .red)
            }
        }
    }
}

struct LockScreenView: View {
    let context: ActivityViewContext<StreamingActivityAttributes>

    var body: some View {
        HStack {
            VStack(alignment: .leading, spacing: 4) {
                HStack(spacing: 6) {
                    Image(systemName: context.state.isConnected ? "wifi" : "wifi.slash")
                        .foregroundStyle(context.state.isConnected ? .green : .red)
                    Text("CouchVision")
                        .font(.headline)
                }
                HStack(spacing: 12) {
                    SensorIndicator(name: "CAM", active: context.state.cameraActive)
                    SensorIndicator(name: "LiDAR", active: context.state.lidarActive)
                    SensorIndicator(name: "IMU", active: context.state.imuActive)
                }
            }
            Spacer()
            VStack(alignment: .trailing, spacing: 4) {
                Text(String(format: "%.0f Hz", context.state.messageRate))
                    .font(.title2)
                    .fontWeight(.semibold)
                Text(context.state.bytesSent.formattedBytes)
                    .font(.caption)
                    .foregroundStyle(.secondary)
            }
        }
        .padding()
    }
}

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
