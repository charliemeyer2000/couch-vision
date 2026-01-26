import Foundation

/// UserDefaults-backed settings persistence
enum SettingsStorage {
    private static let defaults = UserDefaults.standard

    // MARK: - Keys

    private enum Key: String {
        case endpoint
        case topicPrefix
        case cameraResolution
        case cameraFrameRate
        case jpegQuality
        case generatePointCloud
        case publishConfidence
        case pointCloudDownsample
        case imuRate
    }

    // MARK: - Connection

    static var endpoint: String {
        get { defaults.string(forKey: Key.endpoint.rawValue) ?? "tcp://192.168.1.1:7447" }
        set { defaults.set(newValue, forKey: Key.endpoint.rawValue) }
    }

    static var topicPrefix: String {
        get { defaults.string(forKey: Key.topicPrefix.rawValue) ?? "/iphone" }
        set { defaults.set(newValue, forKey: Key.topicPrefix.rawValue) }
    }

    // MARK: - Camera

    static var cameraResolution: CameraConfig.Resolution {
        get {
            guard let rawValue = defaults.string(forKey: Key.cameraResolution.rawValue),
                  let resolution = CameraConfig.Resolution(rawValue: rawValue) else {
                return .p720
            }
            return resolution
        }
        set { defaults.set(newValue.rawValue, forKey: Key.cameraResolution.rawValue) }
    }

    static var cameraFrameRate: Int {
        get {
            let value = defaults.integer(forKey: Key.cameraFrameRate.rawValue)
            return value > 0 ? value : 30
        }
        set { defaults.set(newValue, forKey: Key.cameraFrameRate.rawValue) }
    }

    static var jpegQuality: Double {
        get {
            let value = defaults.double(forKey: Key.jpegQuality.rawValue)
            return value > 0 ? value : 80
        }
        set { defaults.set(newValue, forKey: Key.jpegQuality.rawValue) }
    }

    // MARK: - LiDAR

    static var generatePointCloud: Bool {
        get { defaults.object(forKey: Key.generatePointCloud.rawValue) as? Bool ?? true }
        set { defaults.set(newValue, forKey: Key.generatePointCloud.rawValue) }
    }

    static var publishConfidence: Bool {
        get { defaults.bool(forKey: Key.publishConfidence.rawValue) }
        set { defaults.set(newValue, forKey: Key.publishConfidence.rawValue) }
    }

    static var pointCloudDownsample: Int {
        get {
            let value = defaults.integer(forKey: Key.pointCloudDownsample.rawValue)
            return value > 0 ? value : 4
        }
        set { defaults.set(newValue, forKey: Key.pointCloudDownsample.rawValue) }
    }

    // MARK: - IMU

    static var imuRate: Int {
        get {
            let value = defaults.integer(forKey: Key.imuRate.rawValue)
            return value > 0 ? value : 100
        }
        set { defaults.set(newValue, forKey: Key.imuRate.rawValue) }
    }
}
