#include "kdNode.h"

#include <assert.h>
#include <stdio.h> // for printf
#include <string.h>

float max(float a, float b) {return a > b ? a : b;}
float min(float a, float b) {return a < b ? a : b;}

// ByteArray: helpers

Vertex *getVertexFromArray(ByteArray vertices, unsigned int index) {
    assert(strcmp(vertices.typeName, "Vertex") == 0);
    return &(((Vertex *)vertices.data)[index]);
}

Triangle *getFaceFromArray(ByteArray faces, unsigned int index) {
    assert(strcmp(faces.typeName, "Triangle") == 0);
    return &(((Triangle *)faces.data)[index]);
}

unsigned int *getIndexFromArray(ByteArray indices, unsigned int index) {
    assert(strcmp(indices.typeName, "UnsignedInt") == 0);
    return &(((unsigned int *)indices.data)[index]);
}

KDNode *getNodeFromArray(ByteArray nodes, unsigned int index) {
    assert(strcmp(nodes.typeName, "KDNode") == 0);
    return &(((KDNode *)nodes.data)[index]);
}

// ByteArray: external

void deinitByteArray(ByteArray *byteArray) {
    free(byteArray->data);
    byteArray->size = 0;
    byteArray->count = 0;
}

ByteArray initByteArray(const char *typeName, unsigned int elementCount, unsigned int elementSize) {
    ByteArray result;
    result.typeName = typeName;
    result.elementSize = elementSize;
    result.size = elementCount * elementSize;
    result.data = result.size ? malloc(result.size) : NULL;
    result.count = elementCount;
    return result;
}

void resizeByteArray(ByteArray *byteArray, unsigned int newCount) {
    if (newCount == 0) {
        deinitByteArray(byteArray);
        return;
    }

    unsigned int newSize = newCount * byteArray->elementSize;
    void *newData = malloc(newSize);
    memcpy(newData, byteArray->data, min(newSize, byteArray->size));
    free(byteArray->data);
    byteArray->data = newData;
    byteArray->size = newSize;
    byteArray->count = newCount;
}

// Intersection: helpers

float intersectionBox(AABB b, Ray r) {
    double tmin = -INFINITY, tmax = INFINITY;

    for (int i = 0; i < 3; ++i) {
        double t1 = (b.min[i] - r.pos[i]) / r.dir[i];
        double t2 = (b.max[i] - r.pos[i]) / r.dir[i];

        tmin = max(tmin, min(t1, t2));
        tmax = min(tmax, max(t1, t2));
    }

    if (tmax > max(tmin, 0.0)) {
        if (tmin < 0.0) {
            return tmax;
        } else {
            return tmin;
        }
    }
    return INFINITY;
}

bool intersectsBox(AABB b, Ray r) {
    float intersection = intersectionBox(b, r);
    return isfinite(intersection) && intersection > 0.0;
}

Intersection makeIntersection(float distance, simd_float3 normal, simd_float3 pos) {
    Intersection intersection = { distance, normal, pos };
    return intersection;
}

simd_float3 normalOf(ExplicitTriangle t) {
    return simd_normalize(simd_cross(t.v1 - t.v0, t.v2 - t.v0));
}

Intersection intersectionTriangle(ExplicitTriangle t, Ray r) {
    // Triangle plane
    simd_float3 nor = normalOf(t);
    float d = simd_dot(nor, t.v0);

    // Plane intersection
    float hit = (d - simd_dot(nor, r.pos)) / simd_dot(nor, r.dir);
    simd_float3 hitPos = hit * r.dir + r.pos;

    // Plane intersection behind ray (miss)
    if (hit <= 0) {
        return missedIntersection();
    }

    // Inside outside test
    float side0 = simd_dot(nor, simd_cross(hitPos - t.v0, t.v1 - t.v0));
    float side1 = simd_dot(nor, simd_cross(hitPos - t.v1, t.v2 - t.v1));
    float side2 = simd_dot(nor, simd_cross(hitPos - t.v2, t.v0 - t.v2));

    if (side0 * side1 > 0 && side1 * side2 > 0) {
        // Intersection
        return makeIntersection(hit, nor, hitPos);
    } else {
        // Not inside triangle (miss)
        return missedIntersection();
    }
}

Intersection intersectionTree(const KDNode *nodes, const unsigned int *leaves,
                              const Triangle *faces, const Vertex *vertices, Ray r) {
    const KDNode root = nodes[0];
    // Split node
    if (root.type <= 2) {
        const KDSplitNode split = root.split;
        if (!intersectsBox(split.aabb, r)) {
            return missedIntersection();
        }

        Intersection lIntersection = intersectionTree(nodes + split.left,  leaves, faces, vertices, r);
        Intersection rIntersection = intersectionTree(nodes + split.right, leaves, faces, vertices, r);
        return closestIntersection(lIntersection, rIntersection);
    } else {
        const KDLeafNode leaf = root.leaf;
        const unsigned int leafCount = root.type >> 2;

        Intersection intersection = missedIntersection();

        const unsigned int staticLeafCount = min(leafCount, MAX_STATIC_FACES);
        for (int i = 0; i < staticLeafCount; i++) {
            const unsigned int faceIndex = leaf.staticList[i];
            const Triangle triangle = faces[faceIndex];
            ExplicitTriangle t;
            t.v0 = vertices[triangle.v[0]].pos;
            t.v1 = vertices[triangle.v[1]].pos;
            t.v2 = vertices[triangle.v[2]].pos;

            Intersection triIntersect = intersectionTriangle(t, r);
            intersection = closestIntersection(intersection, triIntersect);
        }

        const unsigned int dynamicLeafCount = max(0, leafCount - staticLeafCount);
        for (int i = 0; i < dynamicLeafCount; i++) {
            const unsigned int faceIndex = leaves[leaf.dynamicListStart + i];
            const Triangle triangle = faces[faceIndex];
            ExplicitTriangle t;
            t.v0 = vertices[triangle.v[0]].pos;
            t.v1 = vertices[triangle.v[1]].pos;
            t.v2 = vertices[triangle.v[2]].pos;

            Intersection triIntersect = intersectionTriangle(t, r);
            intersection = closestIntersection(intersection, triIntersect);
        }
        return intersection;
    }
}

// Intersection: external

bool isHit(Intersection intersection) {
    return !isnan(intersection.distance);
}

Intersection closestIntersection(Intersection a, Intersection b) {
    if (!isHit(a) && !isHit(b)) {
        return missedIntersection();
    }
    else if (isHit(a) && (!isHit(b) || a.distance < b.distance)) {
        return a;
    } else {
        return b;
    }
}

Intersection missedIntersection() {
    Intersection miss = { NAN, simd_make_float3(NAN), simd_make_float3(NAN) };
    return miss;
}

Intersection applyTransform(Intersection intersection, Transform transform) {
    Intersection result = intersection;
    result.pos *= transform.scale;
    result.pos = simd_mul(result.pos, transform.rotation);
    result.pos += transform.translation;
    result.normal = simd_mul(result.normal, transform.rotation);
    return result;
}

Intersection intersectionModel(Model model, Ray r) {
    // Perform model transform on the model (by inverting the transform on the ray)
    r.pos -= model.transform.translation;
    r.pos = simd_mul(r.pos, simd_transpose(model.transform.rotation));
    r.pos /= model.transform.scale;
    r.dir = simd_mul(r.dir, simd_transpose(model.transform.rotation));

    Intersection modelIntersection = intersectionTree((KDNode *)model.kdNodes.data,
                                                      (unsigned int *)model.kdLeaves.data,
                                                      model.faces, model.vertices, r);

    return applyTransform(modelIntersection, model.transform);
}

// Partitioning: helpers

bool isEmpty(const AABB box) {
    return simd_any(box.max < box.min);
}

AABB emptyBox() {
    AABB box = { simd_make_float3(-INFINITY), simd_make_float3(INFINITY) };
    return box;
}

bool inBox(simd_float3 p, const AABB box) {
    return simd_all(p < box.max && p > box.min);
}

AABB unionBox(simd_float3 p, const AABB box) {
    AABB result = box;
    if (isEmpty(box)) {
        result.max = p;
        result.min = p;
        return result;
    }

    result.max = simd_max(p, result.max);
    result.min = simd_min(p, result.min);
    return result;
}

void splitAABB(const AABB aabb, const short splitAxis, const float splitPos,
               AABB *left, AABB *right /* out */) {
    assert(aabb.min[splitAxis] <= splitPos && splitPos <= aabb.max[splitAxis]);
    *left = aabb;
    left->max[splitAxis] = splitPos;
    *right = aabb;
    right->min[splitAxis] = splitPos;
}

typedef struct Polygon {
    // 9 because that's the max number of vertices a triangle clipped by an AABB can have
    simd_float3 verts[9];
    uint8_t vertCount;
} Polygon;

typedef struct Plane {
    simd_float4 plane;
} Plane;

Plane makePlane(const simd_float4 normAndConst) {
    Plane p;
    p.plane = normAndConst;
    return p;
}

float signedDistToPlane(const simd_float3 p, const Plane plane) {
    return simd_dot(plane.plane, simd_make_float4(p, 1.0f));
}

Polygon clipPolygon(const Plane p, const Polygon polygon) {
    Polygon clipped;
    clipped.vertCount = 0;
    for (int vert = 0; vert < polygon.vertCount; vert++) {
        const int nextVert = (vert + 1 == polygon.vertCount) ? 0 : vert + 1; // Wrap around
        const simd_float3 currPos = polygon.verts[vert];
        const simd_float3 nextPos = polygon.verts[nextVert];

        const float currDist = signedDistToPlane(currPos, p);
        const float nextDist = signedDistToPlane(nextPos, p);
        const bool currInside = currDist <= 0;
        const bool nextInside = nextDist <= 0;

        // Do not add this edge as it is completely outside
        if (!currInside && !nextInside) {
            continue;
        }
        // Add this edge as it is completely inside
        if (currInside && nextInside) {
            clipped.verts[clipped.vertCount] = nextPos;
            clipped.vertCount++;
        }
        // Replace a straddling edge with a clipped version of itself
        if (currInside != nextInside) {
            const float alpha = nextDist / (nextDist - currDist);
            assert(fabs(nextDist - currDist) > 0.001); // Numerical stability
            const simd_float3 intersection = alpha * currPos + (1.0f - alpha) * nextPos;

            if (currInside) {
                clipped.verts[clipped.vertCount] = intersection;
                clipped.vertCount++;
            } else {
                clipped.verts[clipped.vertCount] = intersection;
                clipped.verts[clipped.vertCount + 1] = nextPos;
                clipped.vertCount += 2;
            }
        }
    }
    // Assume that plane creation can only add at most one vertex
    assert(clipped.vertCount - polygon.vertCount <= 1);
    return clipped;
}

AABB clipTriangle(const ExplicitTriangle t, const AABB clipBox) {
    int numInsideVertices = 0;
    if (inBox(t.v0, clipBox)) { numInsideVertices++; }
    if (inBox(t.v1, clipBox)) { numInsideVertices++; }
    if (inBox(t.v2, clipBox)) { numInsideVertices++; }

    if (numInsideVertices == 3) {
        AABB result = emptyBox();
        result = unionBox(t.v0, result);
        result = unionBox(t.v1, result);
        result = unionBox(t.v2, result);
        return result;
    } else {
        AABB result = emptyBox();
        Polygon p;
        p.vertCount = 3;
        p.verts[0] = t.v0;
        p.verts[1] = t.v1;
        p.verts[2] = t.v2;

        p = clipPolygon(makePlane(simd_make_float4( 1.0f,  0.0f,  0.0f, -clipBox.max.x)), p);
        p = clipPolygon(makePlane(simd_make_float4(-1.0f,  0.0f,  0.0f,  clipBox.min.x)), p);
        p = clipPolygon(makePlane(simd_make_float4( 0.0f,  1.0f,  0.0f, -clipBox.max.y)), p);
        p = clipPolygon(makePlane(simd_make_float4( 0.0f, -1.0f,  0.0f,  clipBox.min.y)), p);
        p = clipPolygon(makePlane(simd_make_float4( 0.0f,  0.0f,  1.0f, -clipBox.max.z)), p);
        p = clipPolygon(makePlane(simd_make_float4( 0.0f,  0.0f, -1.0f,  clipBox.min.z)), p);

        for (int i = 0; i < p.vertCount; i++) {
            result = unionBox(p.verts[i], result);
        }
        return result;
    }
}

typedef struct SplitInfo {
    uint32_t lCount;
    uint32_t rCount;
    uint32_t splitAxis;
    float splitPos;
} SplitInfo;

SplitInfo getOptimalPartitionMedianSplit(const AABB aabb,
                                         const ByteArray faceIndices,
                                         const ByteArray faces,
                                         const ByteArray vertices) {
    SplitInfo split = { 0, 0, 0, NAN };

    const int faceCount = faceIndices.count;
    const int verticesPerFace = 3;

    // Find mean for split
    simd_float3 vertexMin = simd_make_float3(INFINITY, INFINITY, INFINITY);
    simd_float3 vertexMax = -vertexMin;
    {
        for (int f = 0; f < faceCount; f++) {
            Triangle t = *getFaceFromArray(faces, *getIndexFromArray(faceIndices, f));
            for (int v = 0; v < verticesPerFace; v++) {
                const Vertex *vertex = getVertexFromArray(vertices, t.v[v]);
                vertexMin = simd_min(vertex->pos, vertexMin);
                vertexMax = simd_max(vertex->pos, vertexMax);
            }
        }
    }
    simd_float3 median = (vertexMin + vertexMax) * 0.5f;

    // Determine memory needed for separate partitions
    {
        // Partition the faces
        simd_int3 lCount = simd_make_int3(0, 0, 0);
        simd_int3 rCount = simd_make_int3(0, 0, 0);
        for (int f = 0; f < faceCount; f++) {
            Triangle t = *getFaceFromArray(faces, *getIndexFromArray(faceIndices, f));
            simd_char3 left = simd_make_char3(false, false, false);
            simd_char3 right = simd_make_char3(false, false, false);
            for (int v = 0; v < verticesPerFace; v++) {
                for (int split = 0; split < 3; split++) {
                    Vertex *vertex = getVertexFromArray(vertices, t.v[v]);
                    if (vertex->pos[split] < median[split]) {
                        left[split] = true;
                    } else {
                        right[split] = true;
                    }
                }
            }
            for (int split = 0; split < 3; split++) {
                if (left[split]) {
                    lCount[split]++;
                }
                if (right[split]) {
                    rCount[split]++;
                }
            }
        }

        // Analyze split ratios to find optimal split
        int32_t idealSplit = faceCount / 2;
        simd_int3 overLap = simd_abs(lCount - idealSplit) + simd_abs(rCount - idealSplit);
        split.splitAxis = (overLap.x < overLap.y && overLap.x < overLap.z) ? 0 :
                          (overLap.y < overLap.z) ? 1 : 2;
        split.splitPos = median[split.splitAxis];
        split.lCount = lCount[split.splitAxis];
        split.rCount = rCount[split.splitAxis];
    }

    return split;
}

static int leafCount = 0;
void partitionSerialKD(const AABB aabb,
                       const ByteArray faceIndices,
                       const ByteArray faces,
                       const ByteArray vertices,
                       ByteArray *nodes,
                       ByteArray *leaves) {
    const int faceCount = faceIndices.count;
    const int verticesPerFace = 3;

    SplitInfo split = getOptimalPartitionMedianSplit(aabb, faceIndices, faces, vertices);

    // Cut failed if either lists are the same size as the original
    if (split.lCount == faceCount || split.rCount == faceCount) {
        // Shove all faces into a leaf node
        *nodes = initByteArray("KDNode", 1, sizeof(KDNode));
        KDNode *leaf = getNodeFromArray(*nodes, 0);
        leaf->type = (faceCount << 2) | 3;
        leaf->leaf.dynamicListStart = leaves->count;
        int staticFaceCount = min(faceCount, MAX_STATIC_FACES);
        int dynamicFaceCount = max(0, faceCount - staticFaceCount);
        assert(staticFaceCount + dynamicFaceCount == faceCount);
        if (dynamicFaceCount > 0) {
            resizeByteArray(leaves, leaves->count + dynamicFaceCount);
        }
        leafCount += faceCount;

        for (int f = 0; f < staticFaceCount; f++) {
            leaf->leaf.staticList[f] = *getIndexFromArray(faceIndices, f);
        }
        unsigned int *dynamicFaces = ((unsigned int *)leaves->data) + leaf->leaf.dynamicListStart;
        for (unsigned int f = staticFaceCount; f < faceCount; f++) {
            unsigned int index = f - 11;
            static_assert(MAX_STATIC_FACES == 11, "Offset is invalid");
            dynamicFaces[index] = *getIndexFromArray(faceIndices, f);
        }
        return;
    }

    // Allocate memory for partitions
    ByteArray lIndices = initByteArray("UnsignedInt", split.lCount, sizeof(unsigned int));
    ByteArray rIndices = initByteArray("UnsignedInt", split.rCount, sizeof(unsigned int));

    // Partition the faces
    {
        unsigned int leftCounter = 0;
        unsigned int rightCounter = 0;
        for (int f = 0; f < faceCount; f++) {
            const unsigned int faceIndex = *getIndexFromArray(faceIndices, f);
            Triangle t = *getFaceFromArray(faces, faceIndex);
            bool left = false, right = false;
            for (int v = 0; v < verticesPerFace; v++) {
                Vertex *vertex = getVertexFromArray(vertices, t.v[v]);
                if (vertex->pos[split.splitAxis] < split.splitPos) {
                    left = true;
                } else {
                    right = true;
                }
            }
            if (left) {
                *getIndexFromArray(lIndices, leftCounter) = faceIndex;
                leftCounter++;
            }
            if (right) {
                *getIndexFromArray(rIndices, rightCounter) = faceIndex;
                rightCounter++;
            }
        }
        assert(leftCounter == split.lCount);
        assert(rightCounter == split.rCount);
    }

    // Subdivide the AABB
    AABB leftBox, rightBox;
    splitAABB(aabb, split.splitAxis, split.splitPos, &leftBox, &rightBox);

    // Recurse
    ByteArray leftResult = initByteArray("KDNode", 0, sizeof(KDNode));
    ByteArray rightResult = initByteArray("KDNode", 0, sizeof(KDNode));
    partitionSerialKD(leftBox,  lIndices, faces, vertices, &leftResult, leaves);
    partitionSerialKD(rightBox, rIndices, faces, vertices, &rightResult, leaves);

    KDNode splitNode;
    splitNode.type = split.splitAxis;
    splitNode.split.aabb = aabb;
    splitNode.split.left = 1;
    splitNode.split.right = 1 + leftResult.count;
    splitNode.split.split = split.splitPos;

    // Write out result
    unsigned int nodeCount = 1 + leftResult.count + rightResult.count;
    *nodes = initByteArray("KDNode", nodeCount, sizeof(KDNode));
    *getNodeFromArray(*nodes, 0) = splitNode;
    memcpy(((KDNode *)nodes->data) + 1, leftResult.data, sizeof(KDNode) * leftResult.count);
    memcpy(((KDNode *)nodes->data) + 1 + leftResult.count, rightResult.data, sizeof(KDNode) * rightResult.count);

    // Clean up dangling resources
    deinitByteArray(&lIndices);
    deinitByteArray(&rIndices);
    deinitByteArray(&leftResult);
    deinitByteArray(&rightResult);
}

void partitionSerialKDRoot(const AABB aabb,
                           const ByteArray faces,
                           const ByteArray vertices,
                           ByteArray *nodes,
                           ByteArray *leaves) {
    const unsigned int faceCount = faces.size / faces.elementSize;
    ByteArray faceIndices = initByteArray("UnsignedInt", faceCount, sizeof(unsigned int));
    for (int f = 0; f < faceCount; f++) {
        ((unsigned int *)faceIndices.data)[f] = f;
    }
    partitionSerialKD(aabb, faceIndices, faces, vertices, nodes, leaves);
    deinitByteArray(&faceIndices);

    printf("Brute force index structure takes %u bytes\n", faceIndices.size);
    printf("Tree structure takes %u bytes\n", nodes->size);
    printf("Total leaf polygons %d from %d faces\n", leafCount, faceCount);
}

// Partitioning: external

void partitionModel(Model *model) {
    ByteArray faces = initByteArray("Triangle", 0, sizeof(Triangle));
    faces.count = model->faceCount;
    faces.size = faces.count * faces.elementSize;
    faces.data = (void *) model->faces;

    ByteArray vertices = initByteArray("Vertex", 0, sizeof(Vertex));
    vertices.count = model->vertCount;
    vertices.size = vertices.count * vertices.elementSize;
    vertices.data = (void *) model->vertices;

    // Allocate memory for the first node
    model->kdNodes = initByteArray("KDNode", 0, sizeof(KDNode));
    model->kdLeaves = initByteArray("UnsignedInt", 0, sizeof(unsigned int));

    partitionSerialKDRoot(model->aabb, faces, vertices, &model->kdNodes, &model->kdLeaves);
}

// Tracing: external

Ray primaryRay(simd_float2 uv, simd_float2 res, simd_float3 eye, simd_float3 lookAt, simd_float3 up) {
    const float focal = 1.0f;
    const float ar = res.x / res.y;
    const float screenHeight = 2.0f;
    const float screenWidth = ar * screenHeight;

    simd_float3 right = simd_cross(lookAt, up);

    float screenX = (2.0f * uv.x) - 1.0f;
    float screenY = (2.0f * uv.y) - 1.0f;

    simd_float3 u = screenX * simd_normalize(right) * screenWidth * 0.5f;
    simd_float3 v = screenY * simd_normalize(up) * screenHeight * 0.5f;
    simd_float3 dir = simd_normalize(focal * simd_normalize(lookAt) + u + v);

    Ray ray;
    ray.dir = dir;
    ray.pos = eye;
    return ray;
}

// Testing: external

void runTests() {
    {
        AABB box;
        box.min = simd_make_float3(-1, -1, -1);
        box.max = simd_make_float3( 1,  1,  1);

        ExplicitTriangle t;
        t.v0 = simd_make_float3(0.0, 100.0,  100.0);
        t.v1 = simd_make_float3(0.0, 100.0, -100.0);
        t.v2 = simd_make_float3(0.0, -100.0, 0.0);

        AABB clipped = clipTriangle(t, box);
        assert(!isEmpty(clipped));
    }
    {
        AABB box;
        box.min = simd_make_float3(-1, -1, -1);
        box.max = simd_make_float3( 1,  1,  1);

        ExplicitTriangle t;
        t.v0 = simd_make_float3(3.0, 100.0,  100.0);
        t.v1 = simd_make_float3(3.0, 100.0, -100.0);
        t.v2 = simd_make_float3(3.0, -100.0, 0.0);

        AABB clipped = clipTriangle(t, box);
        assert(isEmpty(clipped));
    }
}
