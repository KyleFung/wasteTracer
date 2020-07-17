#include "kdNode.h"

#include <assert.h>
#include <stdio.h> // for printf
#include <stdlib.h> // for rand
#include <string.h>

float max(float a, float b) {return a > b ? a : b;}
float min(float a, float b) {return a < b ? a : b;}

// ByteArray

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

// Intersection

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

Intersection missedIntersection() {
    Intersection miss = { NAN, simd_make_float3(NAN), simd_make_float3(NAN) };
    return miss;
}

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

    if (side0 * side1 >= 0 && side1 * side2 >= 0) {
        // Intersection
        return makeIntersection(hit, nor, hitPos);
    } else {
        // Not inside triangle (miss)
        return missedIntersection();
    }
}

ExplicitTriangle makeExplicitFace(const Vertex vertices[],
                                  const Triangle t) {
    ExplicitTriangle explicitFace;
    explicitFace.v0 = vertices[t.v[0]].pos;
    explicitFace.v1 = vertices[t.v[1]].pos;
    explicitFace.v2 = vertices[t.v[2]].pos;
    return explicitFace;
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
            ExplicitTriangle t = makeExplicitFace(vertices, triangle);

            Intersection triIntersect = intersectionTriangle(t, r);
            intersection = closestIntersection(intersection, triIntersect);
        }

        const unsigned int dynamicLeafCount = max(0, leafCount - staticLeafCount);
        for (int i = 0; i < dynamicLeafCount; i++) {
            const unsigned int faceIndex = leaves[leaf.dynamicListStart + i];
            const Triangle triangle = faces[faceIndex];
            ExplicitTriangle t = makeExplicitFace(vertices, triangle);

            Intersection triIntersect = intersectionTriangle(t, r);
            intersection = closestIntersection(intersection, triIntersect);
        }
        return intersection;
    }
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

// Partitioning

bool isEmpty(const AABB box) {
    return simd_any(box.max < box.min);
}

float surfaceArea(const AABB box) {
    simd_float3 diff = box.max - box.min;
    return 2.0f * simd_dot(diff.xyz, diff.yzx);
}

AABB emptyBox() {
    AABB box = { simd_make_float3(-INFINITY), simd_make_float3(INFINITY) };
    return box;
}

bool inBox(simd_float3 p, const AABB box) {
    return simd_all(p < box.max && p > box.min);
}

AABB commonBox(const AABB a, const AABB b) {
    AABB result;
    result.min = simd_max(a.min, b.min);
    result.max = simd_min(a.max, b.max);
    return result;
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
            assert(fabs(nextDist - currDist) > 0.00000000000001); // Numerical stability
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

        // For numerical stability return intersection of result and input box.
        if (p.vertCount) {
            result = commonBox(result, clipBox);
        }
        return result;
    }
}

#define PLANAR_TOLERANCE 0.00001f
typedef struct SplitInfo {
    uint32_t lCount; // includes planar count if planarToLeft
    uint32_t rCount; // includes planar count if !planarToLeft
    uint32_t pCount;
    uint32_t splitAxis;
    float splitPos;
    bool planarToLeft;
    float cost;
} SplitInfo;

typedef struct PartitionCategory {
    bool left : 1;
    bool right : 1;
    bool planar : 1;
} PartitionCategory;

// 0th bit = left, 1st bit = right, 2nd bit = planar
PartitionCategory categorizeTriangle(const ExplicitTriangle t,
                                     const int splitAxis,
                                     const float splitPos) {
    PartitionCategory result = { false, false, true };
    for (int v = 0; v < 3; v++) {
        if (fabs(t.verts[v][splitAxis] - splitPos) >= PLANAR_TOLERANCE) {
            if (t.verts[v][splitAxis] < splitPos) {
                result.left = true;
                result.planar = false;
            } else {
                result.right = true;
                result.planar = false;
            }
        }
    }
    if (result.planar) {
        assert(!result.left && !result.right);
    }
    return result;
}

void countPartitions(const unsigned int partitionCount,
                     const float partitions[],
                     const unsigned int partitionTypes[],
                     const ByteArray faceIndices,
                     const ByteArray faces,
                     const ByteArray vertices,
                     unsigned int lCount[],
                     unsigned int rCount[],
                     unsigned int pCount[]) {
    for (int p = 0; p < partitionCount; p++) {
        lCount[p] = 0;
        rCount[p] = 0;
        pCount[p] = 0;
    }
    for (int f = 0; f < faceIndices.count; f++) {
        Triangle face = *getFaceFromArray(faces, *getIndexFromArray(faceIndices, f));
        ExplicitTriangle t = makeExplicitFace(getVertexFromArray(vertices, 0), face);
        for (int p = 0; p < partitionCount; p++) {
            unsigned int split = partitionTypes[p];
            PartitionCategory category = categorizeTriangle(t, split, partitions[p]);
            if (category.left) lCount[p]++;
            if (category.right) rCount[p]++;
            if (category.planar) pCount[p]++;
        }
    }
}

SplitInfo getOptimalPartitionSAH(const AABB aabb,
                                 const ByteArray faceIndices,
                                 const ByteArray faces,
                                 const ByteArray vertices) {
    const int faceCount = faceIndices.count;

    SplitInfo optimalSplit = { 0, 0, 0, 3, NAN, false, INFINITY };
    for (int f = 0; f < faceCount; f++) {
        Triangle face = *getFaceFromArray(faces, *getIndexFromArray(faceIndices, f));
        ExplicitTriangle t = makeExplicitFace(getVertexFromArray(vertices, 0), face);
        AABB clippedBox = clipTriangle(t, aabb);
        if (!isEmpty(clippedBox)) {
            unsigned int lCount[6], rCount[6], pCount[6];
            const unsigned int splitTypes[6] = { 0, 1, 2, 0, 1, 2 };
            const float splitCandidates[6] = { clippedBox.min.x, clippedBox.min.y, clippedBox.min.z,
                                               clippedBox.max.x, clippedBox.max.y, clippedBox.max.z };
            countPartitions(6, splitCandidates, splitTypes, faceIndices, faces, vertices,
                            lCount, rCount, pCount);
            // Evaluate each of the split candidates for the best one
            for (int i = 0; i < 6; i++) {
                AABB left, right;
                splitAABB(aabb, splitTypes[i], splitCandidates[i], &left, &right);
                const float saL = surfaceArea(left);
                const float saR = surfaceArea(right);
                const bool partitionLeft = saL < saR;
                const float cost = saL * (lCount[i] + partitionLeft * pCount[i]) +
                                   saR * (rCount[i] + (1 - partitionLeft) * pCount[i]);
                if (cost < optimalSplit.cost) {
                    optimalSplit.cost = cost;
                    optimalSplit.lCount = lCount[i] + partitionLeft * pCount[i];
                    optimalSplit.rCount = rCount[i] + (1 - partitionLeft) * pCount[i];
                    optimalSplit.pCount = pCount[i];
                    optimalSplit.splitAxis = splitTypes[i];
                    optimalSplit.splitPos = splitCandidates[i];
                    optimalSplit.planarToLeft = partitionLeft;
                    assert(optimalSplit.lCount + optimalSplit.rCount + optimalSplit.pCount >= faceCount);
                }
            }
        }
    }

    return optimalSplit;
}

SplitInfo getOptimalPartitionMedianSplit(const AABB aabb,
                                         const ByteArray faceIndices,
                                         const ByteArray faces,
                                         const ByteArray vertices) {
    SplitInfo optimalSplit = { 0, 0, 0, 3, NAN, false, INFINITY };

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
    simd_float3 vecMed = (vertexMin + vertexMax) * 0.5f;
    const float median[3] = { vecMed.x, vecMed.y, vecMed.z };

    // Count partition sizes
    unsigned int lCount[3], rCount[3], pCount[3];
    unsigned int splitTypes[3] = { 0, 1, 2 };
    countPartitions(3, median, splitTypes, faceIndices, faces, vertices,
                    lCount, rCount, pCount);

    // Analyze split ratios to find optimal split
    int32_t idealSplit = faceCount / 2;
    simd_int3 lVec = simd_make_int3(lCount[0], lCount[1], lCount[2]);
    simd_int3 rVec = simd_make_int3(rCount[0], rCount[1], rCount[2]);
    simd_int3 overLap = simd_abs(lVec - idealSplit) + simd_abs(rVec - idealSplit);
    optimalSplit.splitAxis = (overLap.x < overLap.y && overLap.x < overLap.z) ? 0 :
                             (overLap.y < overLap.z) ? 1 : 2;

    // Populate return struct
    optimalSplit.splitPos = median[optimalSplit.splitAxis];
    optimalSplit.lCount = lCount[optimalSplit.splitAxis];
    optimalSplit.rCount = rCount[optimalSplit.splitAxis];
    optimalSplit.pCount = pCount[optimalSplit.splitAxis];
    if (optimalSplit.lCount < optimalSplit.rCount) {
        optimalSplit.planarToLeft = true;
        optimalSplit.lCount += optimalSplit.pCount;
    } else {
        optimalSplit.planarToLeft = false;
        optimalSplit.rCount += optimalSplit.pCount;
    }

    return optimalSplit;
}

bool shouldTerminate(const SplitInfo split, const AABB aabb, const int faceCount) {
    if (split.lCount == faceCount || split.rCount == faceCount) {
        return true;
    }

    if (split.lCount + split.rCount + split.pCount == 0) {
        return true;
    }

    if (faceCount <= MAX_STATIC_FACES) {
        return true;
    }

    const float leafCost = surfaceArea(aabb) * faceCount;
    if (leafCost < split.cost) {
        return true;
    }

    return false;
}

static int leafCount = 0;
void partitionSerialKD(const AABB aabb,
                       const ByteArray faceIndices,
                       const ByteArray faces,
                       const ByteArray vertices,
                       ByteArray *nodes,
                       ByteArray *leaves) {
    const int faceCount = faceIndices.count;

    // Get an optimal splitting plane for these polygons
    SplitInfo split = getOptimalPartitionSAH(aabb, faceIndices, faces, vertices);

    // Build a leaf node if we hit termination condition
    if (shouldTerminate(split, aabb, faceCount)) {
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
            unsigned int index = f - MAX_STATIC_FACES;
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
        unsigned int planarCounter = 0;
        for (int f = 0; f < faceCount; f++) {
            const unsigned int faceIndex = *getIndexFromArray(faceIndices, f);
            Triangle face = *getFaceFromArray(faces, faceIndex);
            ExplicitTriangle t = makeExplicitFace(getVertexFromArray(vertices, 0), face);
            PartitionCategory category = categorizeTriangle(t, split.splitAxis, split.splitPos);
            if (category.planar) {
                planarCounter++;
            }
            if (category.left || (category.planar && split.planarToLeft)) {
                *getIndexFromArray(lIndices, leftCounter) = faceIndex;
                leftCounter++;
            }
            if (category.right || (category.planar && !split.planarToLeft)) {
                *getIndexFromArray(rIndices, rightCounter) = faceIndex;
                rightCounter++;
            }
        }
        assert(leftCounter == split.lCount);
        assert(rightCounter == split.rCount);
        assert(planarCounter == split.pCount);
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

// Tracing

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

float fract(float x) {
    float whole;
    return modff(x, &whole);
}

float hash(float seed) {
    return fract(sin(seed) * 43758.5453);
}

float goldenSequence(float seed) {
    return fract(seed + 1.61803398875f);
}

float sqrt2Sequence(float seed) {
    return fract(seed + 1.41421356237f);
}

simd_float3 cosineDirection(float seed, simd_float3 nor) {
    simd_float3 tc = simd_make_float3(1.0 + nor.z - nor.xy * nor.xy, -nor.x * nor.y) / (1.0 + nor.z);
    simd_float3 uu = simd_make_float3(tc.x, tc.z, -nor.x);
    simd_float3 vv = simd_make_float3(tc.z, tc.y, -nor.y);

    float u = fabs(hash(78.233 + seed));
    float v = fabs(hash(10.873 + seed));
    float a = 6.283185 * v;

    return sqrt(u) * (cos(a) * uu + sin(a) * vv) + sqrt(1.0 - u) * nor;
}

simd_float4 pathTraceKernel(simd_int2 threadID, float seed, Model model, simd_int2 res,
                            simd_float3 eye, simd_float3 lookAt, simd_float3 up) {
    simd_float2 uv = simd_make_float2(threadID.x, threadID.y) / simd_make_float2(res.x, res.y);

    simd_float3 result = simd_make_float3(0.0, 0.0, 0.0);
    for (unsigned int iteration = 0; iteration < 32; iteration++) {
        simd_float3 sumOfPaths = simd_make_float3(0.0f, 0.0f, 0.0f);
        simd_float3 throughPut = simd_make_float3(1.0f, 1.0f, 1.0f);
        float pdf = 1.0f;

        Ray ray = primaryRay(uv, simd_make_float2(res.x, res.y), eye, lookAt, up);
        for (int rayDepth = 0; rayDepth < 2; rayDepth++) {
            const Intersection intersection = intersectionModel(model, ray);
            if (isHit(intersection)) {
                // Hit geometry (continue tracing)
                ray.pos = intersection.pos + 0.01 * intersection.normal;
                seed = goldenSequence(seed);
                ray.dir = simd_normalize(cosineDirection(seed, intersection.normal));

                // Increment the ray (assuming that no geometry is emissive)
                const float pi_inverse = 1.0f / 3.14159f;
                const float brdf = 0.95f * pi_inverse;
                const float geometry = simd_dot(ray.dir, intersection.normal);
                pdf *= pi_inverse;
                throughPut *= brdf * geometry;
            } else {
                // Hit the skybox (uniform radiance)
                sumOfPaths += throughPut * simd_make_float3(1.0f, 1.0f, 1.0f);
                break;
            }
        }
        const simd_float3 estimate = sumOfPaths / pdf;
        result *= ((float) iteration) / (iteration + 1);
        result += estimate / (iteration + 1);
    }

    return simd_make_float4(result, 1.0f);
}

// Tracing: external

void calculateRadianceSubImage(Model model, float seed, simd_uchar4 *pixels,
                               simd_int2 res, simd_int2 start, simd_int2 end,
                               simd_float3 eye, simd_float3 lookAt, simd_float3 up) {
    float runningSeed = seed;
    for (int y = start.y; y < end.y; y++) {
        for (int x = start.x; x < end.x; x++) {
            simd_int2 threadID = simd_make_int2(x, y);
            runningSeed = sqrt2Sequence(runningSeed);
            simd_float4 floatValue = pathTraceKernel(threadID, runningSeed, model, res, eye, lookAt, up);
            simd_float4 clampedValue = simd_clamp(floatValue, 0.0, 1.0);
            simd_uchar4 charValue = simd_make_uchar4(clampedValue.x * 255,
                                                     clampedValue.y * 255,
                                                     clampedValue.z * 255,
                                                     clampedValue.w * 255);
            pixels[(res.y - y - 1) * res.x + x] = charValue;
        }
    }
}

void calculateRadiance(Model model, simd_uchar4 *pixels, simd_int2 res,
                       simd_float3 eye, simd_float3 lookAt, simd_float3 up) {
    srand(0xdeadbeef);
    // Divide the image into a number of threads
    const int numThreads = 8;
    for (int i = 0; i < numThreads; i++) {
        simd_int2 start = simd_make_int2(0, (res.y * i) / 8);
        simd_int2 end = simd_make_int2(res.x, min(res.y, (res.y * (i + 1)) / 8));
        float seed = (float) rand() / (float)(RAND_MAX);
        calculateRadianceSubImage(model, seed, pixels, res, start, end, eye, lookAt, up);
    }
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
