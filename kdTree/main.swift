import Foundation
 
let objFile = "/Users/kylefung/Downloads/Cartman/Cartman.obj"
let dest = URL(fileURLWithPath: "/Users/kylefung/blah.png")

// Load in model and podium mesh
var model = loadModel(file: objFile)

// Ray trace
let eye = simd_float3(0.0, 0.0, 2.0)
let lookAt = simd_float3(0.0, 0.0, -1.0)
let up = simd_float3(0.0, 1.0, 0.0)
let res = simd_int2(400, 300)
var imagePixels = [Pixel](repeating: Pixel(), count: Int(res.x * res.y))

for x in 0..<res.x {
    for y in 0..<res.y {
        let uv = simd_float2(Float(x),Float(res.y - y)) / simd_float2(Float(res.x), Float(res.y))
        let ray = primaryRay(uv, simd_float2(res), eye, lookAt, up)

        imagePixels[Int(y * res.x + x)] = Pixel(0.5)
        let intersection = intersectionModel(model, ray)
        if isHit(intersection) {
            let lighting = max(0.0, dot(simd_float3(0.0, 1.0, 0.0), intersection.normal))
            imagePixels[Int(y * res.x + x)] = Pixel(lighting)
        }
    }
}

let cgImage = imageFromRGBA32Bitmap(pixels: imagePixels, width: Int(res.x), height: Int(res.y))
writeCGImage(cgImage!, to: dest)
