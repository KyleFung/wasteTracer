import Cocoa
import PathTracerCore
import simd

class ViewController: NSViewController {
    var pathTracer = PathTracer()

    @IBAction func computeSamplesButton(_ sender: NSButton) {
        let operation = AddRadianceSamples(pathTracer, sampleCount: 128)
        let queue = OperationQueue()
        queue.addOperation(operation)
    }

    @IBAction func loadObjButton(_ sender: NSButton) {
        let filePicker = NSOpenPanel()
        filePicker.allowsMultipleSelection = false
        filePicker.canChooseDirectories = false
        filePicker.canChooseFiles = true
        filePicker.allowedFileTypes = ["obj"]

        filePicker.runModal()
        if let chosenFile = filePicker.url {
            pathTracer.setModelCentered(model: loadModel(file: chosenFile.absoluteString))
            pathTracer.calculateSamples(numSamples: 1)
        }
    }

    @IBOutlet weak var imageView: NSImageView!

    override func viewDidLoad() {
        super.viewDidLoad()
        pathTracer.delegate = self
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
            DispatchQueue.main.async {
                let imageSize = NSSize(width: cgImage.width, height: cgImage.height)
                self.imageView.image = NSImage(cgImage: cgImage, size: imageSize)
                self.imageView.needsDisplay = true
            }
        }
    }
}
