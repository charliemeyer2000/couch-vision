import os.log

enum Log {
    private static let subsystem = "com.couchvision.CouchVision"

    static let sensor = Logger(subsystem: subsystem, category: "sensor")
    static let network = Logger(subsystem: subsystem, category: "network")
    static let ros = Logger(subsystem: subsystem, category: "ros")
    static let general = Logger(subsystem: subsystem, category: "general")
    static let app = Logger(subsystem: subsystem, category: "app")
}
