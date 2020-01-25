import Foundation

struct Pixel {
    var r: UInt8
    var g: UInt8
    var b: UInt8
    var a: UInt8

    public init() {
        r = 0
        g = 0
        b = 0
        a = 0
    }

    public init(_ repeating: UInt8) {
        r = repeating
        g = repeating
        b = repeating
        a = 255
    }

    public init(_ repeating: Float) {
        r = UInt8(repeating * 255)
        g = UInt8(repeating * 255)
        b = UInt8(repeating * 255)
        a = 255
    }

    public init(_ r: UInt8, _ g: UInt8, _ b: UInt8) {
        self.r = r
        self.g = g
        self.b = b
        self.a = 255
    }

    public init(_ r: Float, _ g: Float, _ b: Float) {
        self.r = UInt8(r * 255)
        self.g = UInt8(g * 255)
        self.b = UInt8(b * 255)
        self.a = 255
    }

    public init(_ rgb: simd_float3) {
        self.r = UInt8(rgb.x * 255)
        self.g = UInt8(rgb.y * 255)
        self.b = UInt8(rgb.z * 255)
        self.a = 255
    }
}

func imageFromRGBA32Bitmap(pixels: [Pixel], width: Int, height: Int) -> CGImage? {
    guard width > 0 && height > 0 else { return nil }
    guard pixels.count == width * height else { return nil }

    let rgbColorSpace = CGColorSpaceCreateDeviceRGB()
    let bitmapInfo = CGBitmapInfo(rawValue: CGImageAlphaInfo.premultipliedFirst.rawValue)
    let bitsPerComponent = 8
    let bitsPerPixel = 32

    var data = pixels // Copy to mutable []
    guard let providerRef = CGDataProvider(data: NSData(bytes: &data,
                                                        length: data.count * MemoryLayout<Pixel>.size)
        )
        else { return nil }

    guard let cgim = CGImage(
        width: width,
        height: height,
        bitsPerComponent: bitsPerComponent,
        bitsPerPixel: bitsPerPixel,
        bytesPerRow: width * MemoryLayout<Pixel>.size,
        space: rgbColorSpace,
        bitmapInfo: bitmapInfo,
        provider: providerRef,
        decode: nil,
        shouldInterpolate: true,
        intent: .defaultIntent
        )
        else { return nil }

    return cgim
}

@discardableResult func writeCGImage(_ image: CGImage, to destinationURL: URL) -> Bool {
    guard let destination = CGImageDestinationCreateWithURL(destinationURL as CFURL, kUTTypePNG, 1, nil) else { return false }
    CGImageDestinationAddImage(destination, image, nil)
    return CGImageDestinationFinalize(destination)
}
