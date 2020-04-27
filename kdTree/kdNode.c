#include "kdNode.h"

#include <stdio.h> // for printf

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

float max(float a, float b) {return a > b ? a : b;}
float min(float a, float b) {return a < b ? a : b;}

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

Intersection intersectionModel(Model model, Ray r) {
    Intersection result = missedIntersection();

    // Perform model transform on the model (by inverting the transform on the ray)
    r.pos -= model.transform.translation;
    r.pos = simd_mul(r.pos, simd_transpose(model.transform.rotation));
    r.pos /= model.transform.scale;
    r.dir = simd_mul(r.dir, simd_transpose(model.transform.rotation));

    // Transform ray such that the centroid of the model is at the origin
    r.pos += model.centroid;

    // Try the bounding box first
    float boxIntersection = intersectionBox(model.aabb, r);
    if (!isfinite(boxIntersection) || boxIntersection <= 0.0) {
        return result;
    }
    for (int f = 0; f < model.faceCount; f++) {
        Triangle triangle = model.faces[f];
        ExplicitTriangle t;
        t.v0 = model.vertices[triangle.v[0]].pos;
        t.v1 = model.vertices[triangle.v[1]].pos;
        t.v2 = model.vertices[triangle.v[2]].pos;

        Intersection intersection = intersectionTriangle(t, r);
        if (isHit(intersection) && (intersection.distance < result.distance || !isHit(result))) {
            result = intersection;
        }
    }
    return result;
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
        return makeIntersection(hit, hitPos, nor);
    } else {
        // Not inside triangle (miss)
        return missedIntersection();
    }
}

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

// Struct helpers
simd_float3 normalOf(ExplicitTriangle t) {
    return simd_normalize(simd_cross(t.v1 - t.v0, t.v2 - t.v0));
}

bool isHit(Intersection intersection) {
    return !isnan(intersection.distance);
}

Intersection makeIntersection(float distance, simd_float3 normal, simd_float3 pos) {
    Intersection intersection = { distance, normal, pos };
    return intersection;
}

Intersection missedIntersection() {
    Intersection miss = { NAN, simd_make_float3(NAN), simd_make_float3(NAN) };
    return miss;
}

AABB emptyBox() {
    AABB box = { simd_make_float3(NAN), simd_make_float3(NAN) };
    return box;
}

static int leafCount = 0;
void partitionSerialKDRelative(const ByteArray faceIndices,
                               const ByteArray faces,
                               const ByteArray vertices,
                               ByteArray *nodes) {
    const int faceCount = faceIndices.count;
    const int verticesPerFace = 3;

    // Find mean for split
    simd_float3 min = simd_make_float3(INFINITY, INFINITY, INFINITY);
    simd_float3 max = -min;
    {
        for (int f = 0; f < faceCount; f++) {
            Triangle t = *getFaceFromArray(faces, *getIndexFromArray(faceIndices, f));
            for (int v = 0; v < verticesPerFace; v++) {
                const Vertex *vertex = getVertexFromArray(vertices, t.v[v]);
                min = simd_min(vertex->pos, min);
                max = simd_max(vertex->pos, max);
            }
        }
    }
    simd_float3 median = (min + max) * 0.5f;

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
        unsigned int staticFaceCount = 0;
        leafCount += faceCount;
        for (int f = 0; f < faceCount; f++) {
            if (f < MAX_STATIC_FACES) {
                leaf->leaf.staticList[staticFaceCount] = *getIndexFromArray(faceIndices, f);
                staticFaceCount++;
            } else {
                printf("Warning: Maximum static face count exceeded: %d\n", faceCount);
            }
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

    // Recurse
    ByteArray leftResult = initByteArray("KDNode", 0, sizeof(KDNode));
    ByteArray rightResult = initByteArray("KDNode", 0, sizeof(KDNode));
    partitionSerialKDRelative(lIndices, faces, vertices, &leftResult);
    partitionSerialKDRelative(rIndices, faces, vertices, &rightResult);

    KDNode splitNode;
    splitNode.type = optimalSplit;
    // splitNode.split.aabb = ...
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

void partitionSerialKD(const ByteArray faces,
                       const ByteArray vertices,
                       ByteArray *nodes) {
    const unsigned int faceCount = faces.size / faces.elementSize;
    ByteArray faceIndices = initByteArray("UnsignedInt", faceCount, sizeof(unsigned int));
    for (int f = 0; f < faceCount; f++) {
        ((unsigned int *)faceIndices.data)[f] = f;
    }
    partitionSerialKDRelative(faceIndices, faces, vertices, nodes);
    deinitByteArray(&faceIndices);

    printf("Brute force index structure takes %u bytes\n", faceIndices.size);
    printf("Tree structure takes %u bytes\n", nodes->size);
    printf("Total leaf polygons %d from %d faces\n", leafCount, faceCount);
}

void partitionModel(Model *model) {
    ByteArray faces = initByteArray("Triangle", 0, sizeof(Triangle));
    faces.count = model->faceCount;
    faces.size = faces.count * faces.elementSize;
    faces.data = (void *) model->faces;

    ByteArray vertices = initByteArray("Vertex", 0, sizeof(Vertex));
    vertices.count = model->vertCount;
    vertices.size = vertices.count * vertices.elementSize;
    vertices.data = (void *) model->vertices;
    partitionSerialKD(faces, vertices, &model->kdNodes);
}
