import Foundation
import Metal
import PathTracerCore

class PathTracer {
    static let defaultRes = simd_int2(800, 600)

    // Scene fields
    static let defaultEyePos = simd_float3(0.0, 0.5, 0.85)
    static let defaultLookAt = -simd_float3(0.0, 0.0, 1.0)
    static let defaultUpDir  = simd_cross(simd_float3(1.0, 0.0, 0.0), PathTracer.defaultLookAt)

    let eye = PathTracer.defaultEyePos
    let lookAt = defaultLookAt
    let up = PathTracer.defaultUpDir
    let res = simd_int2(defaultRes.x, defaultRes.y)
    var scene: Scene?
    var model: Model? // So scene doesn't have a dangling ref to model

    var device: MTLDevice?
    var pipelineState: MTLComputePipelineState?
    var commandQueue: MTLCommandQueue?

    var radianceState = 0
    var radianceTexture0: MTLTexture?
    var radianceTexture1: MTLTexture?
    var sceneBuffer: MTLBuffer?
    var instanceBuffer: MTLBuffer?
    var modelBuffer: MTLBuffer?
    var cameraBuffer: MTLBuffer?
    var nodeBuffer: MTLBuffer?
    var leafBuffer: MTLBuffer?
    var faceBuffer: MTLBuffer?
    var vertexBuffer: MTLBuffer?
    var numSamplesBuffer: MTLBuffer?

    // Image fields
    var numIterations = UInt32(0)
    var radiance = [simd_float4](repeating: simd_float4(), count: Int(defaultRes.x * defaultRes.y))
    var pixels = [simd_uchar4](repeating: simd_uchar4(), count: Int(defaultRes.x * defaultRes.y))

    func createSceneForModel(path: String) {
        model = loadModel(file: path)
        if let model = model {
            scene = buildBasicScene(model)

            if let scene = scene {
                var sceneGPU = SceneGPU.init(aabb: scene.aabb,
                                             modelCount: scene.modelCount,
                                             instanceStart: 0,
                                             instanceCount: scene.instanceCount)
                var modelGPU = ModelGPU.init(faceStart: 0, vertexStart: 0,
                                             faceCount: model.faceCount, vertCount: model.vertCount,
                                             centroid: model.centroid, aabb: model.aabb,
                                             kdNodeStart: 0, kdLeafStart: 0)
                var camera = Camera.init(pos: eye, lookAt: lookAt, up: up)

                // Create buffer objects for scene
                sceneBuffer = device!.makeBuffer(bytes: &sceneGPU, length: MemoryLayout<SceneGPU>.stride, options: [])
                instanceBuffer = device!.makeBuffer(bytes: scene.instances,
                                                    length: MemoryLayout<Instance>.stride * Int(scene.instanceCount), options: [])
                modelBuffer = device!.makeBuffer(bytes: &modelGPU, length: MemoryLayout<ModelGPU>.stride, options: [])
                cameraBuffer = device!.makeBuffer(bytes: &camera, length: MemoryLayout<Camera>.stride, options: [])
                nodeBuffer = device!.makeBuffer(bytes: model.kdNodes,
                                                length: MemoryLayout<KDNode>.stride * Int(model.nodeCount), options: [])
                leafBuffer = device!.makeBuffer(bytes: model.kdLeaves,
                                                length: MemoryLayout<UInt32>.stride * Int(model.leafCount), options: [])
                faceBuffer = device!.makeBuffer(bytes: model.faces,
                                                length: MemoryLayout<Triangle>.stride * Int(model.faceCount), options: [])
                vertexBuffer = device!.makeBuffer(bytes: model.vertices,
                                                  length: MemoryLayout<Vertex>.stride * Int(model.vertCount), options: [])
            }
        }
    }

    func kickOffSamples(numSamples: uint32) {
        let commandBuffer = commandQueue!.makeCommandBuffer()
        let encoder = commandBuffer!.makeComputeCommandEncoder()
        encoder!.setComputePipelineState(pipelineState!)

        if radianceState == 0 {
            encoder!.setTexture(radianceTexture0!, index: 0)
            encoder!.setTexture(radianceTexture1!, index: 1)
            radianceState = 1
        } else {
            encoder!.setTexture(radianceTexture1!, index: 0)
            encoder!.setTexture(radianceTexture0!, index: 1)
            radianceState = 0
        }

        encoder!.setBuffer(sceneBuffer, offset: 0, index: 0)
        encoder!.setBuffer(instanceBuffer, offset: 0, index: 1)
        encoder!.setBuffer(modelBuffer, offset: 0, index: 2)
        encoder!.setBuffer(cameraBuffer, offset: 0, index: 3)
        encoder!.setBuffer(nodeBuffer, offset: 0, index: 4)
        encoder!.setBuffer(leafBuffer, offset: 0, index: 5)
        encoder!.setBuffer(faceBuffer, offset: 0, index: 6)
        encoder!.setBuffer(vertexBuffer, offset: 0, index: 7)

        // Increment number of samples
        encoder!.setBytes(&numIterations, length: MemoryLayout<UInt32>.stride, index: 8)
        var mutNumSamples = numSamples
        encoder!.setBytes(&mutNumSamples, length: MemoryLayout<UInt32>.stride, index: 9)
        numIterations += numSamples

        let numThreadgroups = MTLSize(width: 800, height: 600, depth: 1)
        let threadsPerThreadgroup = MTLSize(width: 8, height: 8, depth: 1)
        encoder!.dispatchThreadgroups(numThreadgroups, threadsPerThreadgroup: threadsPerThreadgroup)
        encoder!.endEncoding()

        commandBuffer!.commit()
        commandBuffer!.waitUntilCompleted()

        if let delegate = delegate {
            delegate.resultsReady(self)
        }
    }

    func calculateSamples(numSamples: uint32, inPlace: Bool) {
        let numFullBatches = numSamples / 8
        if numFullBatches > 0 {
            for _ in 1...numFullBatches {
                kickOffSamples(numSamples: 8)
            }
        }
        let remainder = numSamples % 8
        if remainder != 0 {
            kickOffSamples(numSamples: remainder)
        }
    }

    var delegate: PathTracerProtocol?
}

class AddRadianceSamples: Operation {
    let pathTracer: PathTracer
    let sampleCount: Int

    init(_ pathTracer: PathTracer, sampleCount: Int) {
        self.pathTracer = pathTracer
        self.sampleCount = sampleCount
    }

    override func main () {
        if isCancelled {
            return
        }

        let startTime = CFAbsoluteTimeGetCurrent()
        pathTracer.calculateSamples(numSamples: uint32(sampleCount), inPlace: true)
        let timeElapsed = CFAbsoluteTimeGetCurrent() - startTime
        print(sampleCount, "samples took", timeElapsed, "seconds.")
    }
}

protocol PathTracerProtocol {
    func resultsReady(_ pathTracer: PathTracer)
}
