import Foundation

extension Triangle {
    init(_ v0: UInt32, _ v1: UInt32, _ v2: UInt32) {
        self.init()
        self.v = (v0, v1, v2)
    }
}

extension Material {
    init(_ name: String) {
        self.init()
        self.materialName = UnsafeMutablePointer<Int8>(mutating: (name as NSString).utf8String)
    }
}

func loadModel(file: String) -> ([Triangle], [Vertex], [Material]) {
    let dirPath = URL(string: file)!.deletingLastPathComponent().absoluteString
    var vertexCount: Int = 0
    var faceCount: Int = 0
    var materials: [Material] = []

    if let aStreamReader = StreamReader(path: file) {
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
            if (line.hasPrefix("mtllib")) {
                // Load in a material library
                let fileName = line.trimmingCharacters(in: .whitespacesAndNewlines).components(separatedBy: " ")[1]
                loadMaterials(file: dirPath + fileName, materials: &materials)
            }
        }
    } else {
        print("Failed to load %s", file)
    }

    // Collect vertices
    var vertices = [Vertex](repeating: Vertex(), count: vertexCount)
    vertexCount = 0
    if let aStreamReader = StreamReader(path: file) {
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

    return (faces, vertices, materials)
}

func loadMaterials(file: String, materials: inout [Material]) {
    if let aStreamReader = StreamReader(path: file) {
        defer {
            aStreamReader.close()
        }
        var newMaterial: Material?
        while let line = aStreamReader.nextLine() {
            let trimmedLine = line.trimmingCharacters(in: .whitespacesAndNewlines)
            if trimmedLine.hasPrefix("#") || trimmedLine.isEmpty {
                continue
            } else if trimmedLine.hasPrefix("newmtl ") {
                if let newMaterial = newMaterial {
                    materials.append(newMaterial)
                }
                newMaterial = Material(trimmedLine.components(separatedBy: " ")[1])
                // Initialize object... I hate C
                newMaterial?.materialName = nil
                newMaterial?.diffColor = simd_float3(repeating: 0.0)
                newMaterial?.specColor = simd_float3(repeating: 0.0)
                newMaterial?.specPower = 0.0
                newMaterial?.diffMapName = nil
                newMaterial?.diffMapIndex = uint32(0)
            } else if trimmedLine.hasPrefix("Ns ") {
                newMaterial?.specPower = Float(trimmedLine.components(separatedBy: " ")[1])!
            } else if trimmedLine.hasPrefix("Kd ") {
                let vec = trimmedLine.components(separatedBy: " ")
                newMaterial?.diffColor = simd_float3(Float(vec[1])!, Float(vec[2])!, Float(vec[3])!)
            } else if trimmedLine.hasPrefix("Ks ") {
                let vec = trimmedLine.components(separatedBy: " ")
                newMaterial?.specColor = simd_float3(Float(vec[1])!, Float(vec[2])!, Float(vec[3])!)
            } else if trimmedLine.hasPrefix("Ka ") {
                continue
            } else if trimmedLine.hasPrefix("Ni ") {
                continue
            } else if trimmedLine.hasPrefix("illum ") {
                continue
            } else if trimmedLine.hasPrefix("d ") {
                continue
            } else {
                print("Unknown material attribute: ", trimmedLine)
            }
        }

        // Collect the last material
        if let newMaterial = newMaterial {
            materials.append(newMaterial)
        }
    } else {
        print("Failed to load %s", file)
    }
}
