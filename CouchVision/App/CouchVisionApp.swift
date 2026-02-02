import SwiftUI
import UIKit

@main
struct CouchVisionApp: App {
    @UIApplicationDelegateAdaptor(AppDelegate.self) var appDelegate
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

final class AppDelegate: NSObject, UIApplicationDelegate {
    func applicationWillTerminate(_ application: UIApplication) {
        LiveActivityManager.shared.endActivityImmediately()
    }
}
