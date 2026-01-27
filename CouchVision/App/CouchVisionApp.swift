import SwiftUI

@main
struct CouchVisionApp: App {
    @StateObject private var coordinator: SensorCoordinator
    @Environment(\.scenePhase) private var scenePhase

    init() {
        let coord = SensorCoordinator()
        coord.setPublisher(ZenohPublisher())
        _coordinator = StateObject(wrappedValue: coord)
    }

    var body: some Scene {
        WindowGroup {
            ContentView()
                .environmentObject(coordinator)
        }
        .onChange(of: scenePhase) { newPhase in
            switch newPhase {
            case .background:
                coordinator.enterBackground()
            case .active:
                coordinator.enterForeground()
            default:
                break
            }
        }
    }
}
