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

    @IBAction func exportPngButton(_ sender: NSButton) {
        if let image = imageView.image {
            let cgImage = image.cgImage(forProposedRect: nil, context: nil, hints: nil)

            if let cgImage = cgImage {
                let filePicker = NSSavePanel()
                filePicker.showsHiddenFiles = false
                filePicker.canCreateDirectories = true
                filePicker.allowedFileTypes = ["png"]
                filePicker.runModal()

                if let chosenFile = filePicker.url {
                    guard let destination = CGImageDestinationCreateWithURL(chosenFile as CFURL,
                                                                            kUTTypePNG, 1, nil) else { return }
                    CGImageDestinationAddImage(destination, cgImage, nil)
                    CGImageDestinationFinalize(destination)
                }
            }
        }
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
            var startTime = CFAbsoluteTimeGetCurrent()
            pathTracer.calculateSamples(numSamples: 1, inPlace: false)
            var timeElapsed = CFAbsoluteTimeGetCurrent() - startTime
            print("Recursion: ", timeElapsed)

            startTime = CFAbsoluteTimeGetCurrent()
            pathTracer.calculateSamples(numSamples: 1, inPlace: true)
            timeElapsed = CFAbsoluteTimeGetCurrent() - startTime
            print("Stackless: ", timeElapsed)
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
