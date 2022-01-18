import Foundation
import AppKit

extension Triangle {
    init(_ v0: UInt32, _ v1: UInt32, _ v2: UInt32) {
        self.init()
        self.v = (v0, v1, v2)
    }
}

extension Texture {
    init(_ filePath: String) {
        self.init()
        self.filePath = toUnsafe(filePath)
    }
}

func toUnsafe(_ string: String) -> UnsafeMutablePointer<Int8> {
    return UnsafeMutablePointer<Int8>(mutating: (string as NSString).utf8String!)
}

public func loadModel(file: String) -> Model {
    let fullPath = URL(string: file)!
    let dirPath = fullPath.deletingLastPathComponent().relativePath
    var vertexCount: Int = 0
    var faceCount: Int = 0
    var materials = [String : Material]()
    var textures: [Texture] = []

    if let aStreamReader = StreamReader(path: fullPath) {
        defer {
            aStreamReader.close()
        }
        while let line = aStreamReader.nextLine() {
            if line.isEmpty {
                continue
            } else if line[line.startIndex] == "v" {
                vertexCount += 1
            } else if line[line.startIndex] == "f" {
                faceCount += 1
            } else if line.hasPrefix("mtllib") {
                // Load in a material library
                let fileName = line.trimmingCharacters(in: .whitespacesAndNewlines).components(separatedBy: " ")[1]
                loadMaterials(file: dirPath + "/" + fileName, materials: &materials, textures: &textures)
            }
        }
    } else {
        print("Failed to load %s", file)
    }

    // Collect geometry and material mappings
    var vertices = [Vertex](repeating: Vertex(), count: vertexCount)
    vertexCount = 0
    var aabbMin = simd_float3(repeating: Float.infinity)
    var aabbMax = simd_float3(repeating: -Float.infinity)
    var vertAttributes: Set<String> = .init()
    var faces = [Triangle](repeating: Triangle(), count: faceCount)
    var materialLUT: [MaterialLookup] = []
    faceCount = 0
    if let aStreamReader = StreamReader(path: fullPath) {
        defer {
            aStreamReader.close()
        }
        let separation = CharacterSet(charactersIn: " /")
        while let line = aStreamReader.nextLine() {
            if line.isEmpty {
                continue
            } else if line.hasPrefix("v") && !line.hasPrefix("vt") && !line.hasPrefix("vn") {
                vertAttributes.insert("v")
                let vertex = line.trimmingCharacters(in: .whitespacesAndNewlines).components(separatedBy: " ")
                let x: Float = Float(vertex[1])!
                let y: Float = Float(vertex[2])!
                let z: Float = Float(vertex[3])!

                let v = simd_float3(x, y, z)
                aabbMin = min(aabbMin, v)
                aabbMax = max(aabbMax, v)
                vertices[vertexCount].pos = v
                vertexCount += 1
            } else if line.hasPrefix("vt") {
                vertAttributes.insert("vt")
            } else if line.hasPrefix("vn") {
                vertAttributes.insert("vn")
            } else if line[line.startIndex] == "f" {
                let indices = line.trimmingCharacters(in: .whitespacesAndNewlines).components(separatedBy: separation)
                faces[faceCount] = Triangle(UInt32(indices[0 * vertAttributes.count + 1])! - 1,
                                            UInt32(indices[1 * vertAttributes.count + 1])! - 1,
                                            UInt32(indices[2 * vertAttributes.count + 1])! - 1)
                faceCount += 1
            } else if line.hasPrefix("usemtl") {
                let materialName = line.trimmingCharacters(in: .whitespacesAndNewlines).components(separatedBy: " ")[1]
                if let material = materials[materialName] {
                    let firstIndex = UInt32(faceCount)
                    let lookup = MaterialLookup(startFace: firstIndex, numFaces: 0, material: material)
                    if let prevLookup = materialLUT.last {
                        materialLUT[materialLUT.count - 1].numFaces = firstIndex - prevLookup.startFace
                    }
                    materialLUT.append(lookup)
                }
            }
        }
    }

    // Patch up last material lookup
    if let lastLookup = materialLUT.last {
        materialLUT[materialLUT.count - 1].numFaces = UInt32(faceCount) - lastLookup.startFace
    }

    let cFaces = UnsafeMutablePointer<Triangle>.allocate(capacity: faces.count)
    cFaces.initialize(from: UnsafeMutablePointer<Triangle>(mutating: faces), count: faces.count)

    let cVertices = UnsafeMutablePointer<Vertex>.allocate(capacity: vertices.count)
    cVertices.initialize(from: UnsafeMutablePointer<Vertex>(mutating: vertices), count: vertices.count)

    let cMaterialLUT = UnsafeMutablePointer<MaterialLookup>.allocate(capacity: materialLUT.count)
    cMaterialLUT.initialize(from: UnsafeMutablePointer<MaterialLookup>(mutating: materialLUT), count: materialLUT.count)

    var model = Model(faces: cFaces,
                      vertices: cVertices,
                      faceCount: UInt32(faceCount),
                      vertCount: UInt32(vertexCount),
                      centroid: (aabbMax + aabbMin) * 0.5,
                      aabb: AABB(max: aabbMax, min: aabbMin),
                      kdNodes: nil,
                      nodeCount: 0,
                      kdLeaves: nil,
                      leafCount: 0,
                      materialLUT: cMaterialLUT,
                      matCount: UInt32(materialLUT.count))

    // Create kd tree for this model
    partitionModel(&model)

    return model
}

func loadMaterials(file: String, materials: inout [String : Material], textures: inout [Texture]) {
    let fullPath = URL(string: file)!
    let dirPath = fullPath.deletingLastPathComponent().relativePath
    if let aStreamReader = StreamReader(path: fullPath) {
        defer {
            aStreamReader.close()
        }
        var materialName: String?
        while let line = aStreamReader.nextLine() {
            let trimmedLine = line.trimmingCharacters(in: .whitespacesAndNewlines)
            if trimmedLine.hasPrefix("#") || trimmedLine.isEmpty {
                continue
            } else if trimmedLine.hasPrefix("newmtl ") {
                materialName = trimmedLine.components(separatedBy: " ")[1]
                if let materialName = materialName {
                    materials[materialName] = Material()
                }
            } else if trimmedLine.hasPrefix("Ns "), let materialName = materialName {
                materials[materialName]?.specPower = Float(trimmedLine.split(separator: " ")[1])!
            } else if trimmedLine.hasPrefix("Kd "), let materialName = materialName {
                let vec = trimmedLine.split(separator: " ")
                materials[materialName]?.diffColor = simd_float3(Float(vec[1])!, Float(vec[2])!, Float(vec[3])!)
            } else if trimmedLine.hasPrefix("Ks "), let materialName = materialName {
                let vec = trimmedLine.split(separator: " ")
                materials[materialName]?.specColor = simd_float3(Float(vec[1])!, Float(vec[2])!, Float(vec[3])!)
            } else if trimmedLine.hasPrefix("Ka ") {
                continue
            } else if trimmedLine.hasPrefix("Ni ") {
                continue
            } else if trimmedLine.hasPrefix("illum ") {
                continue
            } else if trimmedLine.hasPrefix("d ") {
                continue
            } else if trimmedLine.hasPrefix("map_Kd ") {
                let textureName = trimmedLine.components(separatedBy: " ")[1]
                if !loadTexture(file: dirPath + "/" + textureName, textureTable: &textures) {
                    print("Diffuse texture load failed!")
                }
            } else {
                print("Unknown material attribute: ", trimmedLine)
            }
        }
    } else {
        print("Failed to load %s", file)
    }
}

func loadTexture(file: String, textureTable: inout [Texture]) -> Bool {
    if let (size) = getImageProperties(file: file) {
        var texture = Texture(file)
        texture.width = UInt32(size.width)
        texture.height = UInt32(size.height)
        texture.index = UInt32(textureTable.count)

        textureTable.append(texture)
        return true
    }

    print("Texture loading failed ")
    return false
}

private func getImageProperties(file: String) -> NSSize? {
    let img = NSImage(contentsOfFile: file)
    if let img = img {
        return img.size
    }
    print("Error loading image file ", file)
    return nil
}
