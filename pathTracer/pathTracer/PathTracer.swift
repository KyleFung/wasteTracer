import Foundation
import PathTracerCore

class PathTracer {
    static let defaultRes = simd_int2(800, 600)

    // Scene fields
    static let defaultEyePos = simd_float3(0.0, 0.5, 1.75)
    static let defaultLookAt = -simd_float3(0.0, 0.0, 1.0)
    static let defaultUpDir  = simd_cross(simd_float3(1.0, 0.0, 0.0), PathTracer.defaultLookAt)

    let eye = PathTracer.defaultEyePos
    let lookAt = defaultLookAt
    let up = PathTracer.defaultUpDir
    let res = simd_int2(defaultRes.x, defaultRes.y)
    var scene: Scene?
    var model: Model? // So scene doesn't have a dangling ref to model

    // Image fields
    var numIterations = 0
    var radiance = [simd_float4](repeating: simd_float4(), count: Int(defaultRes.x * defaultRes.y))
    var pixels = [simd_uchar4](repeating: simd_uchar4(), count: Int(defaultRes.x * defaultRes.y))

    func createSceneForModel(path: String) {
        model = loadModel(file: path)
        if let model = model {
            scene = buildBasicScene(model)
        }
    }

    func calculateSamples(numSamples: uint32, inPlace: Bool) {
        for _ in 1...numSamples {
            if let scene = scene {
                addRadianceSample(scene, uint32(Int.random(in: 0...10000)), Int32(numIterations),
                                  UnsafeMutablePointer<simd_float4>(mutating: radiance),
                                  UnsafeMutablePointer<simd_uchar4>(mutating: pixels),
                                  res, eye, lookAt, up, inPlace)
                numIterations += 1
            }
            if let delegate = delegate {
                delegate.resultsReady(self)
            }
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
