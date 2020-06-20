import Foundation
import AppKit

extension Triangle {
    init(_ v0: UInt32, _ v1: UInt32, _ v2: UInt32) {
        self.init()
        self.v = (v0, v1, v2)
    }
}

extension Material {
    init(_ name: String) {
        self.init()
        self.materialName = toUnsafe(name)
    }
}

extension Texture {
    init(_ name: String) {
        self.init()
        self.textureName = toUnsafe(name)
    }
}

func toUnsafe(_ string: String) -> UnsafeMutablePointer<Int8> {
    return UnsafeMutablePointer<Int8>(mutating: (string as NSString).utf8String!)
}

func loadModel(file: String) -> Model {
    let dirPath = URL(string: file)!.deletingLastPathComponent().absoluteString
    var vertexCount: Int = 0
    var faceCount: Int = 0
    var materials: [Material] = []
    var textures: [Texture] = []

    if let aStreamReader = StreamReader(path: file) {
        defer {
            aStreamReader.close()
        }
        while let line = aStreamReader.nextLine() {
            if line.isEmpty {
                continue
            }
            if line[line.startIndex] == "v" {
                vertexCount += 1
            }
            if line[line.startIndex] == "f" {
                faceCount += 1
            }
            if line.hasPrefix("mtllib") {
                // Load in a material library
                let fileName = line.trimmingCharacters(in: .whitespacesAndNewlines).components(separatedBy: " ")[1]
                loadMaterials(file: dirPath + fileName, materials: &materials, textures: &textures)
            }
        }
    } else {
        print("Failed to load %s", file)
    }

    // Collect vertices
    var vertices = [Vertex](repeating: Vertex(), count: vertexCount)
    vertexCount = 0
    var aabbMin = simd_float3(repeating: Float.infinity)
    var aabbMax = simd_float3(repeating: -Float.infinity)
    var centroid = simd_float3(repeating: 0)
    var vertAttributes: Set<String> = .init()
    if let aStreamReader = StreamReader(path: file) {
        defer {
            aStreamReader.close()
        }
        while let line = aStreamReader.nextLine() {
            if line.hasPrefix("v") && !line.hasPrefix("vt") {
                vertAttributes.insert("v")
                let vertex = line.trimmingCharacters(in: .whitespacesAndNewlines).components(separatedBy: " ")
                let x: Float = Float(vertex[1])!
                let y: Float = Float(vertex[2])!
                let z: Float = Float(vertex[3])!

                let v = simd_float3(x, y, z)
                aabbMin = min(aabbMin, v)
                aabbMax = max(aabbMax, v)
                centroid += v
                vertices[vertexCount].pos = v
                vertexCount += 1
            } else if line.hasPrefix("vt") {
                vertAttributes.insert("vt")
            } else if line.hasPrefix("vn") {
                vertAttributes.insert("vn")
            }
        }
        centroid /= Float(vertexCount)
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
            if line.isEmpty {
                continue
            }
            if line[line.startIndex] == "f" {
                let indices = line.trimmingCharacters(in: .whitespacesAndNewlines).components(separatedBy: separation)
                faces[faceCount] = Triangle(UInt32(indices[0 * vertAttributes.count + 1])! - 1,
                                            UInt32(indices[1 * vertAttributes.count + 1])! - 1,
                                            UInt32(indices[2 * vertAttributes.count + 1])! - 1)
                faceCount += 1
            }
        }
    }

    let cFaces = UnsafeMutablePointer<Triangle>.allocate(capacity: faces.count)
    cFaces.initialize(from: UnsafeMutablePointer<Triangle>(mutating: faces), count: faces.count)

    let cVertices = UnsafeMutablePointer<Vertex>.allocate(capacity: vertices.count)
    cVertices.initialize(from: UnsafeMutablePointer<Vertex>(mutating: vertices), count: vertices.count)

    let cMaterials = UnsafeMutablePointer<Material>.allocate(capacity: materials.count)
    cMaterials.initialize(from: UnsafeMutablePointer<Material>(mutating: materials), count: materials.count)

    var model = Model(faces: cFaces,
                      vertices: cVertices,
                      materials: cMaterials,
                      faceCount: UInt32(faceCount),
                      vertCount: UInt32(vertexCount),
                      matCount: UInt32(materials.count),
                      transform: Transform(scale: simd_float3(repeating: 1.0),
                                           rotation: simd_float3x3.init(diagonal: simd_float3(repeating: 1.0)),
                                           translation: simd_float3(repeating: 0.0)),
                      centroid: centroid,
                      aabb: AABB(max: aabbMax, min: aabbMin),
                      kdNodes: ByteArray(),
                      kdLeaves: ByteArray())

    // Create kd tree for this model
    partitionModel(&model)

    return model
}

func loadMaterials(file: String, materials: inout [Material], textures: inout [Texture]) {
    let dirPath = URL(string: file)!.deletingLastPathComponent().absoluteString
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
            } else if trimmedLine.hasPrefix("map_Kd ") {
                /*let textureName = trimmedLine.components(separatedBy: " ")[1]
                if let texture = loadTexture(file: dirPath + textureName) {
                    newMaterial?.diffMapName = toUnsafe(textureName)
                    newMaterial?.diffMapIndex = UInt32(textures.count)
                    textures.append(texture)
                }*/
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

func loadTexture(file: String) -> Texture? {
    var width = 0
    var height = 0
    var pixelValues: [UInt8]?
    if let imageRef = loadImage(file: file) {
        width = imageRef.width
        height = imageRef.height
        let bitsPerComponent = imageRef.bitsPerComponent
        let bytesPerRow = imageRef.bytesPerRow
        let totalBytes = height * bytesPerRow

        let colorSpace = CGColorSpaceCreateDeviceRGB()
        pixelValues = [UInt8](repeating: 0, count: totalBytes)

        let contextRef = CGContext(data: &pixelValues, width: width, height: height, bitsPerComponent: bitsPerComponent, bytesPerRow: bytesPerRow, space: colorSpace, bitmapInfo: CGImageAlphaInfo.noneSkipLast.rawValue)
        contextRef?.draw(imageRef, in: CGRect(x: 0.0, y: 0.0, width: CGFloat(width), height: CGFloat(height)))

        if let pixels = pixelValues {
            var texture = Texture(file)
            texture.width = UInt32(width)
            texture.height = UInt32(height)
            texture.data = UnsafeMutablePointer<UInt8>(mutating: pixels)
            return texture
        }
    }

    print("Texture loading failed ")
    return nil
}

private func loadImage(file: String) -> CGImage? {
    let img = NSImage(contentsOfFile: file)
    if let img = img {
        var imageRect = CGRect(x: 0, y: 0, width: img.size.width, height: img.size.height)
        return img.cgImage(forProposedRect: &imageRect, context: nil, hints: nil)
    }
    print("Error loading image file ", file)
    return nil
}
