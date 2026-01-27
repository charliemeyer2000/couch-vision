import Combine
import CoreLocation
import Foundation

public struct LocationConfig {
    public let desiredAccuracy: CLLocationAccuracy
    public let distanceFilter: CLLocationDistance
    public let publishVelocity: Bool

    public init(
        desiredAccuracy: CLLocationAccuracy = kCLLocationAccuracyBest,
        distanceFilter: CLLocationDistance = kCLDistanceFilterNone,
        publishVelocity: Bool = true
    ) {
        self.desiredAccuracy = desiredAccuracy
        self.distanceFilter = distanceFilter
        self.publishVelocity = publishVelocity
    }
}

public final class LocationManager: NSObject, ObservableObject {

    @Published public private(set) var state: SensorState = .unknown
    @Published public var isEnabled: Bool = false

    private let locationSubject = PassthroughSubject<TimestampedData<NavSatFix>, Never>()
    public var locationPublisher: AnyPublisher<TimestampedData<NavSatFix>, Never> {
        locationSubject.eraseToAnyPublisher()
    }

    private let velocitySubject = PassthroughSubject<TimestampedData<TwistStamped>, Never>()
    public var velocityPublisher: AnyPublisher<TimestampedData<TwistStamped>, Never> {
        velocitySubject.eraseToAnyPublisher()
    }

    private let headingSubject = PassthroughSubject<TimestampedData<Float64Msg>, Never>()
    public var headingPublisher: AnyPublisher<TimestampedData<Float64Msg>, Never> {
        headingSubject.eraseToAnyPublisher()
    }

    public var config = LocationConfig() {
        didSet { if state == .running { reconfigure() } }
    }

    private let locationManager = CLLocationManager()
    private let frameId = "iphone_gps"

    public let sensorId = "location"
    public let displayName = "GPS Location"

    override public init() {
        super.init()
        locationManager.delegate = self
        checkAvailability()
    }

    @discardableResult
    public func checkAvailability() -> Bool {
        guard CLLocationManager.locationServicesEnabled() else {
            state = .unavailable
            return false
        }

        updateStateFromAuthStatus(locationManager.authorizationStatus)
        return state == .ready || state == .running
    }

    public func requestPermissions() async -> Bool {
        guard CLLocationManager.locationServicesEnabled() else {
            await MainActor.run { state = .unavailable }
            return false
        }

        let status = locationManager.authorizationStatus
        if status == .notDetermined {
            return await withCheckedContinuation { continuation in
                self.continuationLock.lock()
                self.permissionContinuation = continuation
                self.continuationLock.unlock()
                self.locationManager.requestAlwaysAuthorization()
            }
        }

        let granted = status == .authorizedAlways || status == .authorizedWhenInUse
        await MainActor.run { state = granted ? .ready : .unauthorized }
        return granted
    }

    private var permissionContinuation: CheckedContinuation<Bool, Never>?
    private let continuationLock = NSLock()

    public func start() throws {
        let status = locationManager.authorizationStatus
        guard status == .authorizedAlways || status == .authorizedWhenInUse else {
            throw LocationError.notAuthorized
        }

        locationManager.desiredAccuracy = config.desiredAccuracy
        locationManager.distanceFilter = config.distanceFilter
        locationManager.allowsBackgroundLocationUpdates = true
        locationManager.pausesLocationUpdatesAutomatically = false
        locationManager.startUpdatingLocation()
        locationManager.startUpdatingHeading()

        DispatchQueue.main.async { [weak self] in self?.state = .running }
    }

    public func stop() {
        state = .ready
        locationManager.stopUpdatingLocation()
        locationManager.stopUpdatingHeading()
    }

    private func reconfigure() {
        stop()
        try? start()
    }

    private func updateStateFromAuthStatus(_ status: CLAuthorizationStatus) {
        switch status {
        case .notDetermined:
            state = .unknown
        case .restricted,
             .denied:
            state = .unauthorized
        case .authorizedAlways,
             .authorizedWhenInUse:
            state = .ready
        @unknown default:
            state = .unknown
        }
    }

    private func processLocation(_ location: CLLocation) {
        let timestamp = location.timestamp.timeIntervalSince1970

        let header = ROSHeader(timeInterval: timestamp, frameId: frameId)
        let navSatFix = NavSatFix(
            header: header,
            latitude: location.coordinate.latitude,
            longitude: location.coordinate.longitude,
            altitude: location.altitude,
            horizontalAccuracy: max(location.horizontalAccuracy, 0),
            verticalAccuracy: max(location.verticalAccuracy, 0)
        )

        locationSubject.send(TimestampedData(data: navSatFix, timestamp: timestamp, frameId: frameId))

        if config.publishVelocity, location.speed >= 0 {
            let speedMps = location.speed
            let courseDeg = location.course >= 0 ? location.course : 0
            let courseRad = courseDeg * .pi / 180.0

            // Convert speed + course to velocity components (ENU frame)
            let vx = speedMps * sin(courseRad) // East
            let vy = speedMps * cos(courseRad) // North
            let vz = 0.0 // Up (not available from GPS)

            let twist = Twist(linear: Vector3(x: vx, y: vy, z: vz))
            let twistStamped = TwistStamped(header: header, twist: twist)
            velocitySubject.send(TimestampedData(data: twistStamped, timestamp: timestamp, frameId: frameId))
        }
    }
}

extension LocationManager: CLLocationManagerDelegate {
    public func locationManager(_: CLLocationManager, didUpdateLocations locations: [CLLocation]) {
        guard let location = locations.last else { return }
        processLocation(location)
    }

    public func locationManager(_: CLLocationManager, didFailWithError error: Error) {
        Log.sensor.error("Location error: \(error.localizedDescription)")
    }

    public func locationManager(_: CLLocationManager, didUpdateHeading newHeading: CLHeading) {
        let timestamp = Date().timeIntervalSince1970
        // Use true heading if available, otherwise magnetic heading
        let heading = newHeading.trueHeading >= 0 ? newHeading.trueHeading : newHeading.magneticHeading
        let msg = Float64Msg(data: heading)
        headingSubject.send(TimestampedData(data: msg, timestamp: timestamp, frameId: frameId))
    }

    public func locationManagerDidChangeAuthorization(_ manager: CLLocationManager) {
        let status = manager.authorizationStatus
        updateStateFromAuthStatus(status)

        continuationLock.lock()
        let continuation = permissionContinuation
        permissionContinuation = nil
        continuationLock.unlock()

        if let continuation {
            let granted = status == .authorizedAlways || status == .authorizedWhenInUse
            continuation.resume(returning: granted)
        }
    }
}

public enum LocationError: Error, LocalizedError {
    case notAvailable
    case notAuthorized

    public var errorDescription: String? {
        switch self {
        case .notAvailable: "Location services not available"
        case .notAuthorized: "Location access not authorized"
        }
    }
}
