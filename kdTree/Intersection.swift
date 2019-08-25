import Foundation

extension Triangle {
    init(_ v0: UInt32, _ v1: UInt32, _ v2: UInt32) {
        self.init()
        self.v = (v0, v1, v2)
    }
}

func getTrianglesFrom(file: String) -> ([Triangle], [Vertex]) {
    var vertexCount: Int = 0
    var faceCount: Int = 0

    if let aStreamReader = StreamReader(path: objFile) {
        defer {
            aStreamReader.close()
        }
        while let line = aStreamReader.nextLine() {
            if (line[line.startIndex] == "v") {
                vertexCount += 1
            }
            if (line[line.startIndex] == "f") {
                faceCount += 1
            }
        }
    } else {
        print("Failed to load %s", objFile)
    }

    // Collect vertices
    var vertices = [Vertex](repeating: Vertex(), count: vertexCount)
    vertexCount = 0
    if let aStreamReader = StreamReader(path: objFile) {
        defer {
            aStreamReader.close()
        }
        while let line = aStreamReader.nextLine() {
            if (line.hasPrefix("v") && !line.hasPrefix("vt")) {
                let vertex = line.trimmingCharacters(in: .whitespacesAndNewlines).components(separatedBy: " ")
                    let x: Float = Float(vertex[1])!
                    let y: Float = Float(vertex[2])!
                    let z: Float = Float(vertex[3])!
                    vertices[vertexCount].pos = simd_float3(x, y, z)
                    vertexCount += 1
            }
        }
    }

    // Collect faces
    var faces = [Triangle](repeating: Triangle(), count: faceCount)
    faceCount = 0
    if let aStreamReader = StreamReader(path: objFile) {
        defer {
            aStreamReader.close()
        }
        let separation = CharacterSet(charactersIn: " /")
        while let line = aStreamReader.nextLine() {
            if (line[line.startIndex] == "f") {
                let indices = line.trimmingCharacters(in: .whitespacesAndNewlines).components(separatedBy: separation)
                faces[faceCount] = Triangle(UInt32(indices[1])! - 1,
                                            UInt32(indices[3])! - 1,
                                            UInt32(indices[5])! - 1)
                faceCount += 1
            }
        }
    }

    return (faces, vertices)
}
