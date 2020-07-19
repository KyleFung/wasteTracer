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
    let objFile = "/Users/kylefung/Downloads/bunny.obj"
    var pixels = [simd_uchar4](repeating: simd_uchar4(), count: Int(defaultRes.x * defaultRes.y))

    func setModelCentered(model: Model) {
        self.model = model
        self.model?.transform.translation = -model.centroid
    }

    func calculate1Sample() {
        if let model = model {
            calculateRadiance(model, UnsafeMutablePointer<simd_uchar4>(mutating: pixels),
                              res, eye, lookAt, up)
        }
        if let delegate = delegate {
            delegate.resultsReady(self)
        }
    }

    var delegate: PathTracerProtocol?
}

protocol PathTracerProtocol {
    func resultsReady(_ pathTracer: PathTracer)
}
