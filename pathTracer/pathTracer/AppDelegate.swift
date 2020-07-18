import Cocoa
import SwiftUI
import PathTracerCore

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

    var window: NSWindow!

    func applicationDidFinishLaunching(_ aNotification: Notification) {
        model = loadModel(file: objFile)
        if var model = model {
            model.transform.translation = -model.centroid
            calculateRadiance(model, UnsafeMutablePointer<simd_uchar4>(mutating: outImage), res, eye, lookAt, up)
        }

        let contentView = ContentView(pixels: outImage, res: res)

        // Create the window and set the content view. 
        window = NSWindow(
            contentRect: NSRect(x: 0, y: 0, width: 480, height: 300),
            styleMask: [.titled, .closable, .miniaturizable, .resizable, .fullSizeContentView],
            backing: .buffered, defer: false)
        window.center()
        window.setFrameAutosaveName("Main Window")
        window.contentView = NSHostingView(rootView: contentView)
        window.makeKeyAndOrderFront(nil)
    }

    func applicationWillTerminate(_ aNotification: Notification) {
        // Insert code here to tear down your application
    }
}

