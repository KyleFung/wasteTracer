import Cocoa
import PathTracerCore
import simd

class ViewController: NSViewController {
    var pathTracer = PathTracer()

    @IBOutlet weak var imageView: NSImageView!

    override func viewDidLoad() {
        super.viewDidLoad()
        pathTracer.delegate = self

        pathTracer.setModelCentered(model: loadModel(file: pathTracer.objFile))
        pathTracer.calculate1Sample()
    }

    override var representedObject: Any? {
        didSet {
        // Update the view, if already loaded.
        }
    }
}

extension ViewController: PathTracerProtocol {
    func resultsReady(_ pathTracer: PathTracer) {
        // Update the image view of this view
        let cgImage = imageFromRGBA32Bitmap(pixels: pathTracer.pixels,
                                            width: Int(pathTracer.res.x),
                                            height: Int(pathTracer.res.y))
        if let cgImage = cgImage {
            let imageSize = NSSize(width: cgImage.width, height: cgImage.height)
            imageView.image = NSImage(cgImage: cgImage, size: imageSize)
        }
    }
}
