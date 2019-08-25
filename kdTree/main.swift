import Foundation
 
let objFile = "/Users/kylefung/Downloads/Cartman/Cartman.obj"
let dest = URL(fileURLWithPath: "/Users/kylefung/blah.png")
let res = simd_int2(400, 300)

func primaryRay(pxUV: simd_float2, eye: simd_float3, lookAt: simd_float3, up: simd_float3) -> simd_float3
{
    // Camera properties
    let focal: Float = 1.0
    let ar = Float(res.x) / Float(res.y)
    let screenHeight: Float = 2.0
    let screenWidth: Float = ar * screenHeight

    // Calculate eye directions
    let right: simd_float3 = cross(lookAt, up)

    // Calculate this particular pixel's normalized coordinates
    // on the virtual screen in [-1,1] x [-1,1]
    let screenX: Float = (2.0 * pxUV.x) - 1.0
    let screenY: Float = (2.0 * pxUV.y) - 1.0

    //Calculate the direction that the ray through this pixel goes
    let u: simd_float3 = screenX * normalize(right) * screenWidth * 0.5
    let v: simd_float3 = screenY * normalize(up) * screenHeight * 0.5
    let dir: simd_float3 = normalize(focal * normalize(lookAt) + u + v)
    return dir;
}

// Ray trace
let eye = simd_float3(0.0, 1.0, 2.0)
let lookAt = simd_float3(0.0, 0.0, -1.0)
let up = simd_float3(0.0, 1.0, 0.0)
var imagePixels = [Pixel](repeating: Pixel(), count: Int(res.x * res.y))

let (faces, vertices) = getTrianglesFrom(file: objFile)
let rawVertices: UnsafeMutablePointer<Vertex> = UnsafeMutablePointer(mutating: vertices)

for x in 0..<res.x {
    for y in 0..<res.y {
        let uv = simd_float2(Float(x),Float(res.y - y)) / simd_float2(Float(res.x), Float(res.y))
        let dir = primaryRay(pxUV: uv, eye: eye, lookAt: lookAt, up: up)
        let ray = Ray(pos: eye, dir: dir)

        imagePixels[Int(y * res.x + x)] = Pixel(128)
        // Try to search for an intersection
        for t in faces {
            if intersectionTriangle(t, rawVertices, ray) > 0.0 {
                imagePixels[Int(y * res.x + x)] = Pixel(255)
                break
            }
        }
    }
}

let cgImage = imageFromRGBA32Bitmap(pixels: imagePixels, width: Int(res.x), height: Int(res.y))
writeCGImage(cgImage!, to: dest)
