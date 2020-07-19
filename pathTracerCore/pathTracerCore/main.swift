import Foundation

runTests()
 
let objFile = "/Users/kylefung/Downloads/bunny.obj"
let dest = URL(fileURLWithPath: "/Users/kylefung/blah.png")

// Load in model and podium mesh
var model = loadModel(file: objFile)

// Center the model at the origin
model.transform.translation = -model.centroid

// Ray trace
let eye = simd_float3(0.0, 0.0, 0.1)
let lookAt = simd_float3(0.0, 0.0, -1.0)
let up = simd_float3(0.0, 1.0, 0.0)
let res = simd_int2(400, 300)
var radiance = [simd_float4](repeating: simd_float4(), count: Int(res.x * res.y))
var outImage = [simd_uchar4](repeating: simd_uchar4(), count: Int(res.x * res.y))

addRadianceSample(model, 0xdeadbeef, 0,
                  UnsafeMutablePointer<simd_float4>(mutating: radiance),
                  UnsafeMutablePointer<simd_uchar4>(mutating: outImage),
                  res, eye, lookAt, up)

let cgImage = imageFromRGBA32Bitmap(pixels: outImage, width: Int(res.x), height: Int(res.y))
writeCGImage(cgImage!, to: dest)
