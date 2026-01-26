import SwiftUI

@main
struct CouchVisionApp: App {
    @StateObject private var coordinator = SensorCoordinator()

    var body: some Scene {
        WindowGroup {
            ContentView()
                .environmentObject(coordinator)
        }
    }
}
