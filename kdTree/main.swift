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
var imagePixels = [Pixel](repeating: Pixel(), count: Int(res.x * res.y))

for x in 0..<res.x {
    for y in 0..<res.y {
        let uv = simd_float2(Float(x),Float(res.y - y)) / simd_float2(Float(res.x), Float(res.y))
        let ray = primaryRay(uv, simd_float2(res), eye, lookAt, up)

        // Clear color
        imagePixels[Int(y * res.x + x)] = Pixel(0.2, 0.6, 0.9)

        // Ray trace
        let intersection = intersectionModel(model, ray)
        if isHit(intersection) {
            let pointLight = simd_float3(3.0, 3.0, 3.0)
            let lDir = normalize(pointLight - intersection.pos)
            let lighting = min(1.0, max(0.0, dot(lDir, intersection.normal)))

            let shadowRay: Ray = .init(pos: intersection.pos + 0.01 * intersection.normal, dir: lDir)
            let shadowIntersection = intersectionModel(model, shadowRay)
            if isHit(shadowIntersection) {
                imagePixels[Int(y * res.x + x)] = Pixel(0.0)
            } else {
                imagePixels[Int(y * res.x + x)] = Pixel(lighting)
            }
        }
    }
}

let cgImage = imageFromRGBA32Bitmap(pixels: imagePixels, width: Int(res.x), height: Int(res.y))
writeCGImage(cgImage!, to: dest)
