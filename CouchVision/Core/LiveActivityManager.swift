import ActivityKit
import Combine
import Foundation

@MainActor
public final class LiveActivityManager {
    public static let shared = LiveActivityManager()

    private var currentActivity: Activity<StreamingActivityAttributes>?
    private var cancellables = Set<AnyCancellable>()
    private var statsUpdateTimer: Timer?
    private weak var boundCoordinator: SensorCoordinator?

    private init() {}

    public func startActivity(coordinator: SensorCoordinator) {
        guard ActivityAuthorizationInfo().areActivitiesEnabled else {
            Log.app.warning("Live Activities not enabled")
            return
        }

        endActivityImmediately()

        boundCoordinator = coordinator
        let attributes = StreamingActivityAttributes(endpoint: coordinator.config.endpoint)
        let initialState = makeContentState(from: coordinator)

        do {
            currentActivity = try Activity.request(
                attributes: attributes,
                content: .init(state: initialState, staleDate: nil),
                pushType: nil
            )
            Log.app.info("Started Live Activity")
            observeCoordinator(coordinator)
            startStatsTimer(coordinator: coordinator)
        } catch {
            Log.app.error("Failed to start Live Activity: \(error.localizedDescription)")
        }
    }

    public func endActivity() {
        statsUpdateTimer?.invalidate()
        statsUpdateTimer = nil
        cancellables.removeAll()

        guard let activity = currentActivity else {
            boundCoordinator = nil
            return
        }

        let disconnectedState = StreamingActivityAttributes.ContentState(
            isConnected: false,
            cameraActive: false,
            lidarActive: false,
            imuActive: false,
            gpsActive: false,
            environmentActive: false,
            messageRate: 0,
            bytesSent: boundCoordinator?.stats.values.reduce(0) { $0 + $1.bytesPublished } ?? 0
        )

        Task {
            await activity.update(.init(state: disconnectedState, staleDate: Date()))
            Log.app.info("Updated Live Activity to disconnected state")
            try? await Task.sleep(nanoseconds: 2_000_000_000)
            await activity.end(nil, dismissalPolicy: .immediate)
            Log.app.info("Ended Live Activity")
        }

        currentActivity = nil
        boundCoordinator = nil
    }

    private func endActivityImmediately() {
        cancellables.removeAll()
        statsUpdateTimer?.invalidate()
        statsUpdateTimer = nil
        boundCoordinator = nil

        guard let activity = currentActivity else { return }
        currentActivity = nil

        Task {
            await activity.end(nil, dismissalPolicy: .immediate)
        }
    }

    // MARK: - Reactive State Observation

    private func observeCoordinator(_ coordinator: SensorCoordinator) {
        coordinator.$isConnected
            .dropFirst()
            .removeDuplicates()
            .receive(on: DispatchQueue.main)
            .sink { [weak self, weak coordinator] isConnected in
                guard let self, let coordinator else { return }

                if isConnected {
                    updateActivity(from: coordinator)
                } else {
                    updateActivityToDisconnected(preservingStats: coordinator)
                }
            }
            .store(in: &cancellables)

        coordinator.objectWillChange
            .throttle(for: .milliseconds(250), scheduler: DispatchQueue.main, latest: true)
            .sink { [weak self, weak coordinator] _ in
                guard let self, let coordinator else { return }
                updateActivity(from: coordinator)
            }
            .store(in: &cancellables)
    }

    private func startStatsTimer(coordinator: SensorCoordinator) {
        statsUpdateTimer?.invalidate()
        statsUpdateTimer = Timer.scheduledTimer(withTimeInterval: 2.0, repeats: true) { [weak self, weak coordinator] _ in
            Task { @MainActor in
                guard let self, let coordinator, coordinator.isConnected else { return }
                self.updateActivity(from: coordinator)
            }
        }
    }

    private func updateActivity(from coordinator: SensorCoordinator) {
        guard let activity = currentActivity else { return }

        let newState = makeContentState(from: coordinator)
        Task {
            await activity.update(.init(state: newState, staleDate: nil))
        }
    }

    private func updateActivityToDisconnected(preservingStats coordinator: SensorCoordinator) {
        guard let activity = currentActivity else { return }

        let disconnectedState = StreamingActivityAttributes.ContentState(
            isConnected: false,
            cameraActive: false,
            lidarActive: false,
            imuActive: false,
            gpsActive: false,
            environmentActive: false,
            messageRate: 0,
            bytesSent: coordinator.stats.values.reduce(0) { $0 + $1.bytesPublished }
        )

        Task {
            await activity.update(.init(state: disconnectedState, staleDate: Date()))
        }
    }

    private func makeContentState(from coordinator: SensorCoordinator) -> StreamingActivityAttributes.ContentState {
        StreamingActivityAttributes.ContentState(
            isConnected: coordinator.isConnected,
            cameraActive: coordinator.cameraManager.state == .running,
            lidarActive: coordinator.lidarManager.state == .running,
            imuActive: coordinator.motionManager.state == .running,
            gpsActive: coordinator.locationManager.state == .running,
            environmentActive: coordinator.environmentManager.state == .running,
            messageRate: coordinator.stats.values.reduce(0) { $0 + $1.averageHz },
            bytesSent: coordinator.stats.values.reduce(0) { $0 + $1.bytesPublished }
        )
    }
}
