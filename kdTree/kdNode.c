#include "kdNode.h"

#include <stdio.h> // for printf

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
    memcpy(newData, byteArray->data, fmin(newSize, byteArray->size));
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
            assert(!isHit(triIntersect) || fabsf(simd_length(triIntersect.normal) - 1.0f) < 0.1);
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
            assert(!isHit(triIntersect) || fabsf(simd_length(triIntersect.normal) - 1.0f) < 0.1);
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

Intersection intersectionModel(Model model, Ray r) {
    // Perform model transform on the model (by inverting the transform on the ray)
    r.pos -= model.transform.translation;
    r.pos = simd_mul(r.pos, simd_transpose(model.transform.rotation));
    r.pos /= model.transform.scale;
    r.dir = simd_mul(r.dir, simd_transpose(model.transform.rotation));

    // Transform ray such that the centroid of the model is at the origin
    r.pos += model.centroid;

    return intersectionTree((KDNode *)model.kdNodes.data, (unsigned int *)model.kdLeaves.data,
                            model.faces, model.vertices, r);
}

// Partitioning: helpers

AABB emptyBox() {
    AABB box = { simd_make_float3(NAN), simd_make_float3(NAN) };
    return box;
}

void splitAABB(const AABB aabb, const short splitAxis, const float splitPos,
               AABB *left, AABB *right /* out */) {
    assert(aabb.min[splitAxis] <= splitPos && splitPos <= aabb.max[splitAxis]);
    *left = aabb;
    left->max[splitAxis] = splitPos;
    *right = aabb;
    right->min[splitAxis] = splitPos;
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
    unsigned int leftCount = 0, rightCount = 0;
    unsigned int optimalSplit = -1;
    float optimalMedian = NAN;
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
        optimalSplit = (overLap.x < overLap.y && overLap.x < overLap.z) ? 0 :
                       (overLap.y < overLap.z) ? 1 : 2;
        optimalMedian = median[optimalSplit];
        leftCount = lCount[optimalSplit];
        rightCount = rCount[optimalSplit];
    }

    // Cut failed if either lists are the same size as the original
    if (leftCount == faceCount || rightCount == faceCount) {
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
        for (int f = staticFaceCount; f < faceCount; f++) {
            dynamicFaces[f - MAX_STATIC_FACES] = *getIndexFromArray(faceIndices, f);
        }
        return;
    }

    // Allocate memory for partitions
    ByteArray lIndices = initByteArray("UnsignedInt", leftCount, sizeof(unsigned int));
    ByteArray rIndices = initByteArray("UnsignedInt", rightCount, sizeof(unsigned int));

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
                if (vertex->pos[optimalSplit] < median[optimalSplit]) {
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
        assert(leftCounter == leftCount);
        assert(rightCounter == rightCount);
    }

    // Subdivide the AABB
    AABB leftBox, rightBox;
    splitAABB(aabb, optimalSplit, median[optimalSplit], &leftBox, &rightBox);

    // Recurse
    ByteArray leftResult = initByteArray("KDNode", 0, sizeof(KDNode));
    ByteArray rightResult = initByteArray("KDNode", 0, sizeof(KDNode));
    partitionSerialKD(leftBox,  lIndices, faces, vertices, &leftResult, leaves);
    partitionSerialKD(rightBox, rIndices, faces, vertices, &rightResult, leaves);

    KDNode splitNode;
    splitNode.type = optimalSplit;
    splitNode.split.aabb = aabb;
    splitNode.split.left = 1;
    splitNode.split.right = 1 + leftResult.count;
    splitNode.split.split = optimalMedian;

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
