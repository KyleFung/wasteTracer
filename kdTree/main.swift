import Foundation

runTests()
 
let objFile = "/Users/kylefung/Downloads/Cartman/Cartman.obj"
let dest = URL(fileURLWithPath: "/Users/kylefung/blah.png")

// Load in model and podium mesh
var model = loadModel(file: objFile)

// Center the model at the origin
model.transform.translation = -model.centroid

// Ray trace
let eye = simd_float3(0.0, 0.0, 2.0)
let lookAt = simd_float3(0.0, 0.0, -1.0)
let up = simd_float3(0.0, 1.0, 0.0)
let res = simd_int2(800, 600)
var outImage = [simd_uchar4](repeating: simd_uchar4(), count: Int(res.x * res.y))

calculateRadiance(model, UnsafeMutablePointer<simd_uchar4>(mutating: outImage), res, eye, lookAt, up)

let cgImage = imageFromRGBA32Bitmap(pixels: outImage, width: Int(res.x), height: Int(res.y))
writeCGImage(cgImage!, to: dest)
