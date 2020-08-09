import Cocoa
import PathTracerCore
import simd

class ViewController: NSViewController {
    var pathTracer = PathTracer()

    @IBAction func computeSamplesButton(_ sender: NSButton) {
        let operation = AddRadianceSamples(pathTracer, sampleCount: 64)
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
            pathTracer.createSceneForModel(path: chosenFile.absoluteString);
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

        do {
            try doGPUStuff()
        } catch { }
    }

    func doGPUStuff() throws {
        let device = MTLCreateSystemDefaultDevice()!
        let commandQueue = device.makeCommandQueue()!
        let library = try device.makeLibrary(filepath: "Kernels.metallib")

        let commandBuffer = commandQueue.makeCommandBuffer()!
        let encoder = commandBuffer.makeComputeCommandEncoder()!
        encoder.setComputePipelineState(try device.makeComputePipelineState(function: library.makeFunction(name: "uvKernel")!))

        let outputDesc = MTLTextureDescriptor.texture2DDescriptor(pixelFormat: .rgba16Float, width: 800, height: 600, mipmapped: false)
        let outputTexture = device.makeTexture(descriptor: outputDesc)
        encoder.setTexture(outputTexture, index: 0)

        let numThreadgroups = MTLSize(width: 800, height: 600, depth: 1)
        let threadsPerThreadgroup = MTLSize(width: 8, height: 8, depth: 1)
        encoder.dispatchThreadgroups(numThreadgroups, threadsPerThreadgroup: threadsPerThreadgroup)
        encoder.endEncoding()

        commandBuffer.commit()
        commandBuffer.waitUntilCompleted()
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
