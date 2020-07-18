import SwiftUI
import PathTracerCore

struct ContentView: View {
    var pixels :[simd_uchar4]?
    var res :simd_int2?

    public init(pixels: [simd_uchar4], res: simd_int2) {
        self.pixels = pixels
        self.res = res
    }

    public init() {}

    var body: some View {
        if let pixels = pixels, let res = res {
            let cgImage = imageFromRGBA32Bitmap(pixels: pixels, width: Int(res.x), height: Int(res.y))
            return Image(decorative: cgImage!, scale: 1.0)
        } else {
            let defaultRes = simd_int2(200, 200)
            let defaultPixels = [simd_uchar4](repeating: simd_uchar4(0xff, 0x00, 0xff, 0xff), count: Int(defaultRes.x * defaultRes.y))
            let cgImage = imageFromRGBA32Bitmap(pixels: defaultPixels, width: Int(defaultRes.x), height: Int(defaultRes.y))
            return Image(decorative: cgImage!, scale: 1.0)
        }
    }
}

struct ContentView_Previews: PreviewProvider {
    static var previews: some View {
        ContentView()
    }
}
