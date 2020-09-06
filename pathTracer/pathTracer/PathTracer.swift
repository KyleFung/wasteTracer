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
    var noiseTexture: MTLTexture?
    var sceneBuffer: MTLBuffer?
    var instanceBuffer: MTLBuffer?
    var modelBuffer: MTLBuffer?
    var cameraBuffer: MTLBuffer?
    var nodeBuffer: MTLBuffer?
    var leafBuffer: MTLBuffer?
    var faceBuffer: MTLBuffer?
    var vertexBuffer: MTLBuffer?
    var numSamplesBuffer: MTLBuffer?
    var materialLUTBuffer: MTLBuffer?

    // Image fields
    var numIterations = UInt32(0)
    var radiance = [simd_float4](repeating: simd_float4(), count: Int(defaultRes.x * defaultRes.y))
    var pixels = [simd_uchar4](repeating: simd_uchar4(), count: Int(defaultRes.x * defaultRes.y))

    func generate64x64WhiteNoise() {
        var noise = Array<Float32>.init(repeating: Float32(0.0), count: 64 * 64)
        for i in 0..<64*64 {
            noise[i] = Float(Int.random(in: 0...1000)) / Float32(1000.0)
        }
        let noiseDesc = MTLTextureDescriptor.texture2DDescriptor(pixelFormat: .r32Float,
                                                                 width: 64, height: 64, mipmapped: false)
        noiseDesc.usage = .shaderRead
        noiseTexture = device!.makeTexture(descriptor: noiseDesc)
        let region = MTLRegionMake2D(0, 0, 64, 64)
        noiseTexture?.replace(region: region, mipmapLevel: 0, withBytes: noise, bytesPerRow: 64 * 4)
    }

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
                                             kdNodeStart: 0, kdLeafStart: 0,
                                             materialLUTStart: 0, materialCount: model.matCount)
                var camera = Camera.init(pos: eye, lookAt: lookAt, up: up)

                // Initialize the material LUT with the model's materials
                var materialLUT: [MaterialLookup] = []
                for i in 0..<model.matCount {
                    materialLUT.append(model.materialLUT![Int(i)])
                }
                // Model assumes at least one material
                if materialLUT.isEmpty {
                    let defaultMaterial: Material = .init(diffColor: simd_float3(repeating: 0.7),
                                                          specColor: simd_float3(repeating: 0.1),
                                                          specPower: Float(2.0))
                    materialLUT.append(.init(startFace: 0, numFaces: model.faceCount,
                                             material: defaultMaterial))
                }

                var instances: [InstanceGPU] = []
                for i in 0..<scene.instanceCount {
                    let instance = scene.instances![Int(i)]
                    let primitive = instance.primitive
                    var primitiveGPU: PrimitiveGPU = .init()
                    primitiveGPU.type = primitive.type
                    switch primitive.type {
                    case 0:
                        primitiveGPU.modelRef = primitive.modelRef
                    case 1:
                        primitiveGPU.box = .init(dimensions: primitive.box.dimensions,
                                                 materialLUTStart: UInt32(materialLUT.count))
                        materialLUT.append(primitive.box.material)
                    case 2:
                        primitiveGPU.sphere = .init(radius: primitive.sphere.radius,
                                                    materialLUTStart: UInt32(materialLUT.count))
                        materialLUT.append(primitive.sphere.material)
                    default:
                        break
                    }
                    let instanceGPU: InstanceGPU = .init(aabb: instance.aabb,
                                                         transform: instance.transform,
                                                         primitive: primitiveGPU)
                    instances.append(instanceGPU)
                }

                // Create buffer objects for scene
                sceneBuffer = device!.makeBuffer(bytes: &sceneGPU, length: MemoryLayout<SceneGPU>.stride)
                instanceBuffer = device!.makeBuffer(bytes: instances,
                                                    length: MemoryLayout<InstanceGPU>.stride * instances.count)
                modelBuffer = device!.makeBuffer(bytes: &modelGPU, length: MemoryLayout<ModelGPU>.stride)
                cameraBuffer = device!.makeBuffer(bytes: &camera, length: MemoryLayout<Camera>.stride)
                nodeBuffer = device!.makeBuffer(bytes: model.kdNodes, length: MemoryLayout<KDNode>.stride * Int(model.nodeCount))
                leafBuffer = device!.makeBuffer(bytes: model.kdLeaves, length: MemoryLayout<UInt32>.stride * Int(model.leafCount))
                faceBuffer = device!.makeBuffer(bytes: model.faces, length: MemoryLayout<Triangle>.stride * Int(model.faceCount))
                vertexBuffer = device!.makeBuffer(bytes: model.vertices, length: MemoryLayout<Vertex>.stride * Int(model.vertCount))
                materialLUTBuffer = device!.makeBuffer(bytes: materialLUT, length: MemoryLayout<MaterialLookup>.stride * materialLUT.count)

                // Generate white noise
                generate64x64WhiteNoise()
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
        encoder!.setTexture(noiseTexture!, index: 2)

        encoder!.setBuffer(sceneBuffer, offset: 0, index: 0)
        encoder!.setBuffer(instanceBuffer, offset: 0, index: 1)
        encoder!.setBuffer(modelBuffer, offset: 0, index: 2)
        encoder!.setBuffer(cameraBuffer, offset: 0, index: 3)
        encoder!.setBuffer(nodeBuffer, offset: 0, index: 4)
        encoder!.setBuffer(leafBuffer, offset: 0, index: 5)
        encoder!.setBuffer(faceBuffer, offset: 0, index: 6)
        encoder!.setBuffer(vertexBuffer, offset: 0, index: 7)
        encoder!.setBuffer(materialLUTBuffer, offset: 0, index: 10)

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
        let samplesPerBatch = uint32(1)
        let numFullBatches = numSamples / samplesPerBatch
        if numFullBatches > 0 {
            for _ in 1...numFullBatches {
                kickOffSamples(numSamples: samplesPerBatch)
            }
        }
        let remainder = numSamples % samplesPerBatch
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
