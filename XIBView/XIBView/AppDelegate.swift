import Cocoa

@NSApplicationMain
class AppDelegate: NSObject, NSApplicationDelegate {
    static let defaultRes = simd_int2(400, 300)

    // Scene fields
    let eye = simd_float3(0.0, 0.0, 0.1)
    let lookAt = simd_float3(0.0, 0.0, -1.0)
    let up = simd_float3(0.0, 1.0, 0.0)
    let res = simd_int2(defaultRes.x, defaultRes.y)
    var model :Model?

    // Image fields
    let objFile = "/Users/kylefung/Downloads/bunny.obj"
    var outImage = [simd_uchar4](repeating: simd_uchar4(), count: Int(defaultRes.x * defaultRes.y))

    func applicationDidFinishLaunching(_ aNotification: Notification) {
        // Insert code here to initialize your application
    }

    func applicationWillTerminate(_ aNotification: Notification) {
        // Insert code here to tear down your application
    }
}

