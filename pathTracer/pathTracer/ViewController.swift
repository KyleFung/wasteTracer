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
            pathTracer.calculateSamples(numSamples: 1, inPlace: false)
        }
    }

    @IBOutlet weak var imageView: NSImageView!

    override func viewDidLoad() {
        super.viewDidLoad()
        pathTracer.delegate = self

        do {
            try setupGPUResources()
        } catch { }
    }

    func setupGPUResources() throws {
        let device = MTLCreateSystemDefaultDevice()!
        let commandQueue = device.makeCommandQueue()!

        let libPath = Bundle.main.privateFrameworksPath! + "/Kernels.metallib"
        let library = try device.makeLibrary(filepath: libPath)
        let kernel = library.makeFunction(name: "intersectionKernel")!

        let commandBuffer = commandQueue.makeCommandBuffer()!
        let encoder = commandBuffer.makeComputeCommandEncoder()
        let pipelineState = try device.makeComputePipelineState(function: kernel)
        encoder!.setComputePipelineState(pipelineState)

        let radianceDesc = MTLTextureDescriptor.texture2DDescriptor(pixelFormat: .rgba16Float,
                                                                    width: 800, height: 600, mipmapped: false)
        radianceDesc.usage = .unknown
        let radianceTexture0 = device.makeTexture(descriptor: radianceDesc)
        let radianceTexture1 = device.makeTexture(descriptor: radianceDesc)

        // Store these objects for later use
        pathTracer.device = device
        pathTracer.commandQueue = commandQueue
        pathTracer.commandBuffer = commandBuffer
        pathTracer.radianceTexture0 = radianceTexture0
        pathTracer.radianceTexture1 = radianceTexture1
        pathTracer.encoder = encoder
    }

    override var representedObject: Any? {
        didSet {
        // Update the view, if already loaded.
        }
    }
}

extension ViewController: PathTracerProtocol {
    func resultsReady(_ pathTracer: PathTracer) {
        let context = CIContext()
        let radianceTexture = pathTracer.radianceState == 1 ? pathTracer.radianceTexture0 : pathTracer.radianceTexture1
        let cImg = CIImage(mtlTexture: radianceTexture!, options: nil)!
        let cgImg = context.createCGImage(cImg, from: cImg.extent)!

        DispatchQueue.main.async {
            let imageSize = NSSize(width: cgImg.width, height: cgImg.height)
            self.imageView.image = NSImage(cgImage: cgImg, size: imageSize)
            self.imageView.needsDisplay = true
        }
    }
}
