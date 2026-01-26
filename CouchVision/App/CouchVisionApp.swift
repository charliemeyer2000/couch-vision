import SwiftUI

@main
struct CouchVisionApp: App {
    @StateObject private var coordinator: SensorCoordinator

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
    }
}
