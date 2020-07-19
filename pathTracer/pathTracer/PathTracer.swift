import Foundation
import PathTracerCore

class PathTracer {
    static let defaultRes = simd_int2(400, 300)

    // Scene fields
    let eye = simd_float3(0.0, 0.0, 0.1)
    let lookAt = simd_float3(0.0, 0.0, -1.0)
    let up = simd_float3(0.0, 1.0, 0.0)
    let res = simd_int2(defaultRes.x, defaultRes.y)
    var model: Model?

    // Image fields
    var numIterations = 0
    var radiance = [simd_float4](repeating: simd_float4(), count: Int(defaultRes.x * defaultRes.y))
    var pixels = [simd_uchar4](repeating: simd_uchar4(), count: Int(defaultRes.x * defaultRes.y))

    func setModelCentered(model: Model) {
        self.model = model
        self.model?.transform.translation = -model.centroid
    }

    func calculateSamples(numSamples: uint32) {
        for _ in 1...numSamples {
            if let model = model {
                addRadianceSample(model, uint32(Int.random(in: 0...10000)), Int32(numIterations),
                                  UnsafeMutablePointer<simd_float4>(mutating: radiance),
                                  UnsafeMutablePointer<simd_uchar4>(mutating: pixels),
                                  res, eye, lookAt, up)
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

        pathTracer.calculateSamples(numSamples: uint32(sampleCount))
    }
}

protocol PathTracerProtocol {
    func resultsReady(_ pathTracer: PathTracer)
}
