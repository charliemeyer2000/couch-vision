import ActivityKit
import Foundation

@MainActor
public final class LiveActivityManager {
    public static let shared = LiveActivityManager()

    private var currentActivity: Activity<StreamingActivityAttributes>?
    private var updateTimer: Timer?
    private weak var coordinator: SensorCoordinator?

    private init() {}

    public func startActivity(coordinator: SensorCoordinator) {
        guard ActivityAuthorizationInfo().areActivitiesEnabled else {
            Log.app.warning("Live Activities not enabled")
            return
        }

        endActivity()
        self.coordinator = coordinator

        let attributes = StreamingActivityAttributes(endpoint: coordinator.config.endpoint)
        let initialState = makeContentState(from: coordinator)

        do {
            currentActivity = try Activity.request(
                attributes: attributes,
                content: .init(state: initialState, staleDate: nil),
                pushType: nil
            )
            Log.app.info("Started Live Activity")
            startUpdateTimer()
        } catch {
            Log.app.error("Failed to start Live Activity: \(error.localizedDescription)")
        }
    }

    public func endActivity() {
        updateTimer?.invalidate()
        updateTimer = nil
        coordinator = nil

        guard let activity = currentActivity else { return }
        currentActivity = nil

        Task {
            await activity.end(nil, dismissalPolicy: .immediate)
            Log.app.info("Ended Live Activity")
        }
    }

    private func startUpdateTimer() {
        updateTimer?.invalidate()
        updateTimer = Timer.scheduledTimer(withTimeInterval: 2.0, repeats: true) { [weak self] _ in
            Task { @MainActor in
                self?.updateActivity()
            }
        }
    }

    private func updateActivity() {
        guard let activity = currentActivity, let coordinator else { return }

        let newState = makeContentState(from: coordinator)
        Task {
            await activity.update(.init(state: newState, staleDate: nil))
        }
    }

    private func makeContentState(from coordinator: SensorCoordinator) -> StreamingActivityAttributes.ContentState {
        StreamingActivityAttributes.ContentState(
            isConnected: coordinator.isConnected,
            cameraActive: coordinator.cameraManager.state == .running,
            lidarActive: coordinator.lidarManager.state == .running,
            imuActive: coordinator.motionManager.state == .running,
            messageRate: coordinator.stats.values.reduce(0) { $0 + $1.averageHz },
            bytesSent: coordinator.stats.values.reduce(0) { $0 + $1.bytesPublished }
        )
    }
}
