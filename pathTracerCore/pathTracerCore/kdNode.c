#include "kdNode.h"

#include <assert.h>
#include <pthread.h>
#include <stdio.h> // for printf
#include <stdlib.h> // for rand
#include <string.h>

float max(float a, float b) {return a > b ? a : b;}
float min(float a, float b) {return a < b ? a : b;}

// ByteArray
typedef struct ByteArray {
    const char *typeName;
    char *data;
    uint32_t size; // in bytes
    uint32_t elementSize; // in bytes
    uint32_t count;
} ByteArray;

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

float intersectionAABB(AABB b, Ray r) {
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

float boxSDF(simd_float3 pos, simd_float3 boxDim) {
    simd_float3 q = simd_abs(pos) - boxDim;
    return simd_length(simd_max(q, 0.0)) + fmin(fmax(q.x, fmax(q.y,q.z)), 0.0);
}

bool intersectsAABB(AABB b, Ray r) {
    float intersection = intersectionAABB(b, r);
    return isfinite(intersection) && intersection > 0.0;
}

bool inBox(simd_float3 p, const AABB box) {
    return simd_all(p < box.max && p > box.min);
}

MaterialQuery emptyQuery() {
    MaterialQuery query;
    query.faceID = -1;
    query.materialCount = 0;
    query.materialLUTStart = 0;
    return query;
}

Intersection makeIntersection(float distance, simd_float3 normal, simd_float3 pos) {
    Intersection intersection = { distance, normal, pos, emptyQuery() };
    return intersection;
}

Intersection missedIntersection() {
    Intersection miss = { NAN, simd_make_float3(NAN), simd_make_float3(NAN), emptyQuery() };
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

Intersection intersectionBox(Box b, Ray r) {
    AABB aabb;
    aabb.min = -b.dimensions * 0.5f;
    aabb.max =  b.dimensions * 0.5f;

    float t = intersectionAABB(aabb, r);
    if (isfinite(t)) {
        simd_float3 pos = r.pos + t * r.dir;
        float d = boxSDF(pos, b.dimensions);
        simd_float3 eps = simd_make_float3(0.0000001f, 0.0f, 0.0f);
        simd_float3 dev = simd_make_float3(boxSDF(pos + eps.xyz, b.dimensions),
                                           boxSDF(pos + eps.yxz, b.dimensions),
                                           boxSDF(pos + eps.zyx, b.dimensions));
        simd_float3 normal = simd_normalize(dev - d);
        return makeIntersection(t, normal, pos);
    } else {
        return missedIntersection();
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

void setBit(unsigned int* x, short n, bool value) {
    const unsigned int mask = 1 << n;
    if (value) {
        *x = *x | mask;
    } else {
        *x = *x & ~mask;
    }
}

bool getBit(unsigned int x, short n) {
    const unsigned int mask = 1 << n;
    return !!(x & mask);
}

typedef struct TraversalStack {
    unsigned int nodeStack[32];
    unsigned int traversedPath;
    unsigned int leftCloser;
    short stackPointer;
} TraversalStack;

TraversalStack initStack(KDNode rootNode, Ray r) {
    TraversalStack stack;
    for (int i = 0; i < 32; i++) {
        // Set up invalid nodes.
        stack.nodeStack[i] = -1;
    }

    // Determine which of the root's children is closer to the ray.
    stack.leftCloser = 0;
    setBit(&stack.leftCloser, 0, true);
    bool leftCloser = r.pos[rootNode.type] < rootNode.split.split;
    setBit(&stack.leftCloser, 1, leftCloser);

    // Set up traversed path.
    stack.nodeStack[0] = 0;
    stack.nodeStack[1] = leftCloser ? 1 : rootNode.split.right;
    stack.traversedPath = 0;
    stack.stackPointer = 1;

    return stack;
}

void backUpTraversal(TraversalStack *stack, const KDNode *nodes, Intersection topResult) {
    // Back up stack until we find a 0 or hit the bottom
    while (stack->stackPointer > 0 &&
           (getBit(stack->traversedPath, stack->stackPointer) ||
            (isHit(topResult) &&
             inBox(topResult.pos, nodes[stack->nodeStack[stack->stackPointer]].aabb)))) {
        stack->stackPointer -= 1;
    }

    // If we hit the bottom, the traversal is over
    if (stack->stackPointer <= 0) {
        return;
    }

    // Assume the top bit is 0
    assert(getBit(stack->traversedPath, stack->stackPointer) == false);
    // Get the index of the far sibling of this closer node
    bool leftCloser = getBit(stack->leftCloser, stack->stackPointer);
    unsigned int parentIndex = stack->nodeStack[stack->stackPointer - 1];
    const KDNode *parent = &nodes[parentIndex];
    assert(parent->type <= 2);
    stack->nodeStack[stack->stackPointer] = parentIndex + (leftCloser ? parent->split.right : 1);
    // Replace the top 0 with a 1
    setBit(&stack->traversedPath, stack->stackPointer, true);
}

void traverseToCloser(TraversalStack *stack, const KDNode *nodes, Ray r) {
    // Take a fork to the closer child and push that onto the stack
    stack->stackPointer += 1;
    assert(stack->stackPointer < 32);

    const KDNode *parent = &nodes[stack->nodeStack[stack->stackPointer - 1]];
    bool leftCloser = r.pos[parent->type] < parent->split.split;
    setBit(&stack->leftCloser, stack->stackPointer, leftCloser);

    stack->nodeStack[stack->stackPointer] = stack->nodeStack[stack->stackPointer - 1] +
                                            (leftCloser ? 1 : parent->split.right);
    setBit(&stack->traversedPath, stack->stackPointer, false);
}

Intersection intersectionTreeInPlace(const KDNode *nodes, const unsigned int *leaves,
                                     const Triangle *faces, const Vertex *vertices, Ray r) {
    Intersection closest = missedIntersection();
    TraversalStack stack = initStack(nodes[0], r);

    while (stack.stackPointer > 0) {
        const KDNode currentNode = nodes[stack.nodeStack[stack.stackPointer]];
        if (!intersectsAABB(currentNode.aabb, r)) {
            // Back up
            backUpTraversal(&stack, nodes, missedIntersection());
            continue;
        }

        // Split node
        if (currentNode.type <= 2) {
            // Explore the closer side
            traverseToCloser(&stack, nodes, r);
        } else {
            const KDLeafNode leaf = currentNode.leaf;
            const unsigned int leafCount = currentNode.type >> 2;

            Intersection leafIntersection = missedIntersection();

            const unsigned int staticLeafCount = min(leafCount, MAX_STATIC_FACES);
            for (int i = 0; i < staticLeafCount; i++) {
                const unsigned int faceIndex = leaf.staticList[i];
                const Triangle triangle = faces[faceIndex];
                ExplicitTriangle t = makeExplicitFace(vertices, triangle);

                Intersection triIntersect = intersectionTriangle(t, r);
                leafIntersection = closestIntersection(leafIntersection, triIntersect);
            }

            const unsigned int dynamicLeafCount = max(0, leafCount - staticLeafCount);
            for (int i = 0; i < dynamicLeafCount; i++) {
                const unsigned int faceIndex = leaves[leaf.dynamicListStart + i];
                const Triangle triangle = faces[faceIndex];
                ExplicitTriangle t = makeExplicitFace(vertices, triangle);

                Intersection triIntersect = intersectionTriangle(t, r);
                leafIntersection = closestIntersection(leafIntersection, triIntersect);
            }

            closest = closestIntersection(closest, leafIntersection);

            // Back up
            backUpTraversal(&stack, nodes, leafIntersection);
        }
    }

    return closest;
}

Intersection intersectionTree(const KDNode *nodes, const unsigned int *leaves,
                              const Triangle *faces, const Vertex *vertices, Ray r) {
    const KDNode root = nodes[0];
    if (!intersectsAABB(root.aabb, r)) {
        return missedIntersection();
    }

    // Split node
    if (root.type <= 2) {
        const KDSplitNode split = root.split;

        // Decide if the ray r is on the left or the right side of the partition.
        bool leftSide = r.pos[root.type] < split.split;
        const KDNode *firstNode  = nodes + (leftSide ? 1 : split.right);
        const KDNode *secondNode = nodes + (leftSide ? split.right : 1);

        Intersection firstIntersection = intersectionTree(firstNode, leaves, faces, vertices, r);
        if (isHit(firstIntersection) && inBox(firstIntersection.pos, firstNode->aabb)) {
            return firstIntersection;
        } else {
            Intersection secondIntersection = intersectionTree(secondNode, leaves, faces, vertices, r);
            return closestIntersection(firstIntersection, secondIntersection);
        }
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

Intersection intersectionModel(Model model, Ray r, bool inPlace) {
    Intersection modelIntersection;

    if (!inPlace) {
        modelIntersection = intersectionTree(model.kdNodes, model.kdLeaves,
                                             model.faces, model.vertices, r);
    } else {
        modelIntersection = intersectionTreeInPlace(model.kdNodes, model.kdLeaves,
                                                    model.faces, model.vertices, r);
    }

    return modelIntersection;
}

Intersection intersectionInstance(Instance instance, Model *modelArray, Ray r, bool inPlace) {
    simd_float3 oldPos = r.pos;

    // Perform instance inverse transform on the ray
    r.pos -= instance.transform.translation;
    r.pos = simd_mul(r.pos, simd_transpose(instance.transform.rotation));
    r.pos /= instance.transform.scale;
    r.dir = simd_mul(r.dir, simd_transpose(instance.transform.rotation));

    Intersection intersection = missedIntersection();
    switch (instance.primitive.type) {
        case 0: {
            uint32_t modelIndex = instance.primitive.modelRef.modelIndex;
            intersection = intersectionModel(modelArray[modelIndex], r, inPlace);
            break;
        }
        case 1: {
            Box box = instance.primitive.box;
            intersection = intersectionBox(box, r);
            break;
        }
        default: {
            intersection = missedIntersection();
            break;
        }
    }

    if (isHit(intersection)) {
        intersection = applyTransform(intersection, instance.transform);
        intersection.distance = simd_length(intersection.pos - oldPos);
        return intersection;
    } else {
        return missedIntersection();
    }
}

Intersection intersectionScene(Scene scene, Ray r, bool inPlace) {
    if (!intersectsAABB(scene.aabb, r)) {
        return missedIntersection();
    }

    Intersection closest = missedIntersection();
    for (int i = 0; i < scene.instanceCount; i++) {
        if (intersectsAABB(scene.instances[i].aabb, r)) {
            Intersection instanceIntersection = intersectionInstance(scene.instances[i],
                                                                     scene.models,
                                                                     r, inPlace);
            closest = closestIntersection(closest, instanceIntersection);
        }
    }

    return closest;
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

AABB commonBox(const AABB a, const AABB b) {
    AABB result;
    result.min = simd_max(a.min, b.min);
    result.max = simd_min(a.max, b.max);
    return result;
}

AABB unionAABB(const AABB a, const AABB b) {
    AABB result;
    result.min = simd_min(a.min, b.min);
    result.max = simd_max(a.max, b.max);
    return result;
}

AABB unionPointAndAABB(simd_float3 p, const AABB box) {
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
            simd_float3 intersection;
            if (fabs(nextDist - currDist) < 0.0000001) {
                intersection = 0.5 * (currPos + nextPos);
            } else {
                const float alpha = nextDist / (nextDist - currDist);
                intersection = alpha * currPos + (1.0f - alpha) * nextPos;
            }

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

Polygon clipTriangle(const ExplicitTriangle t, const AABB aabb) {
    Polygon p;
    p.vertCount = 3;
    p.verts[0] = t.v0;
    p.verts[1] = t.v1;
    p.verts[2] = t.v2;

    int numInsideVertices = 0;
    if (inBox(t.v0, aabb)) { numInsideVertices++; }
    if (inBox(t.v1, aabb)) { numInsideVertices++; }
    if (inBox(t.v2, aabb)) { numInsideVertices++; }

    if (numInsideVertices == 3) {
        AABB result = emptyBox();
        result = unionPointAndAABB(t.v0, result);
        result = unionPointAndAABB(t.v1, result);
        result = unionPointAndAABB(t.v2, result);
        return p;
    }

    p = clipPolygon(makePlane(simd_make_float4( 1.0f,  0.0f,  0.0f, -aabb.max.x)), p);
    p = clipPolygon(makePlane(simd_make_float4(-1.0f,  0.0f,  0.0f,  aabb.min.x)), p);
    p = clipPolygon(makePlane(simd_make_float4( 0.0f,  1.0f,  0.0f, -aabb.max.y)), p);
    p = clipPolygon(makePlane(simd_make_float4( 0.0f, -1.0f,  0.0f,  aabb.min.y)), p);
    p = clipPolygon(makePlane(simd_make_float4( 0.0f,  0.0f,  1.0f, -aabb.max.z)), p);
    p = clipPolygon(makePlane(simd_make_float4( 0.0f,  0.0f, -1.0f,  aabb.min.z)), p);

    // Snap to the box
    for (int v = 0; v < p.vertCount; v++) {
        for (int i = 0; i < 3; i++) {
            if (fabs(p.verts[v][i] - aabb.min[i]) < 0.000001) {
                p.verts[v][i] = aabb.min[i];
            } else if (fabs(p.verts[v][i] - aabb.max[i]) < 0.000001) {
                p.verts[v][i] = aabb.max[i];
            }
        }
    }

    return p;
}

AABB clipBoxOfTriangle(const ExplicitTriangle t, const AABB clipBox) {
    int numInsideVertices = 0;
    if (inBox(t.v0, clipBox)) { numInsideVertices++; }
    if (inBox(t.v1, clipBox)) { numInsideVertices++; }
    if (inBox(t.v2, clipBox)) { numInsideVertices++; }

    if (numInsideVertices == 3) {
        AABB result = emptyBox();
        result = unionPointAndAABB(t.v0, result);
        result = unionPointAndAABB(t.v1, result);
        result = unionPointAndAABB(t.v2, result);
        return result;
    } else {
        AABB result = emptyBox();
        Polygon p = clipTriangle(t, clipBox);

        for (int i = 0; i < p.vertCount; i++) {
            result = unionPointAndAABB(p.verts[i], result);
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
                                     const AABB aabb,
                                     const int splitAxis,
                                     const float splitPos) {
    Polygon p;
    p = clipTriangle(t, aabb);

    PartitionCategory result = { false, false, true };
    for (int v = 0; v < p.vertCount; v++) {
        if (p.verts[v][splitAxis] != splitPos) {
            if (p.verts[v][splitAxis] < splitPos) {
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
                     const AABB aabb,
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
        unsigned int faceIndex = *getIndexFromArray(faceIndices, f);
        Triangle face = *getFaceFromArray(faces, faceIndex);
        ExplicitTriangle t = makeExplicitFace(getVertexFromArray(vertices, 0), face);
        for (int p = 0; p < partitionCount; p++) {
            unsigned int split = partitionTypes[p];
            PartitionCategory category = categorizeTriangle(t, aabb, split, partitions[p]);
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
        AABB clippedBox = clipBoxOfTriangle(t, aabb);
        if (!isEmpty(clippedBox)) {
            unsigned int lCount[6], rCount[6], pCount[6];
            const unsigned int splitTypes[6] = { 0, 0, 1, 1, 2, 2 };
            const float splitCandidates[6] = { clippedBox.min.x, clippedBox.max.x,
                                               clippedBox.min.y, clippedBox.max.y,
                                               clippedBox.min.z, clippedBox.max.z };
            countPartitions(6, aabb, splitCandidates, splitTypes, faceIndices, faces, vertices,
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

enum EventType {
    endEvent = 0,
    planarEvent = 1,
    startEvent = 2
};

typedef struct SplitEvent {
    float position;
    enum EventType eventType; // 0 end, 1 planar, 2 start
} SplitEvent;

// -1 : b was bigger, +1 : a was bigger, 0 equal

int eventCmp(const void *a, const void *b) {
    const SplitEvent ea = *((SplitEvent *) a);
    const SplitEvent eb = *((SplitEvent *) b);

    bool bGreater = (ea.position < eb.position) || (ea.position == eb.position && ea.eventType < eb.eventType);
    if (bGreater) { return -1; }
    return 1;
}

SplitInfo getOptimalPartitionSAHFaster1Dim(const AABB aabb,
                                           const ByteArray faceIndices,
                                           const ByteArray faces,
                                           const ByteArray vertices,
                                           const unsigned int dim) {
    const int faceCount = faceIndices.count;

    // Allocate an event for min and max candidates for face
    const unsigned int eventCount = faceCount * 2;
    SplitEvent *eventBuffer = malloc(eventCount * sizeof(SplitEvent));

    unsigned int eventIndex = 0;
    // Generate events for the faces
    for (int f = 0; f < faceCount; f++) {
        unsigned int faceIndex = *getIndexFromArray(faceIndices, f);
        Triangle face = *getFaceFromArray(faces, faceIndex);
        ExplicitTriangle t = makeExplicitFace(getVertexFromArray(vertices, 0), face);
        AABB clippedBox = clipBoxOfTriangle(t, aabb);
        if (isEmpty(clippedBox)) {
            continue;
        }
        if (clippedBox.max[dim] == clippedBox.min[dim]) {
            SplitEvent event;
            event.eventType = planarEvent;
            event.position = clippedBox.max[dim];
            eventBuffer[eventIndex++] = event;
        } else {
            SplitEvent start;
            start.eventType = startEvent;
            start.position = clippedBox.min[dim];
            eventBuffer[eventIndex++] = start;

            SplitEvent end;
            end.eventType = endEvent;
            end.position = clippedBox.max[dim];
            eventBuffer[eventIndex++] = end;
        }
    }

    // Sort events
    qsort(eventBuffer, eventIndex, sizeof(SplitEvent), eventCmp);

    // Initialize counters
    unsigned int nL = 0;
    unsigned int nR = faceCount;
    unsigned int nP = 0;

    // Sweep through all events
    SplitInfo optimalSplit = { 0, 0, 0, 3, NAN, false, INFINITY };
    unsigned int sweepIndex = 0;

    while (sweepIndex < eventIndex) {
        // Properties of the current plane
        float planePos = eventBuffer[sweepIndex].position;
        unsigned int p0 = 0;
        unsigned int pp = 0;
        unsigned int pn = 0;

        // Sweep through all events with the same plane pos
        while (sweepIndex < eventIndex && planePos == eventBuffer[sweepIndex].position) {
            SplitEvent event = eventBuffer[sweepIndex];
            if (event.eventType == 2) {
                // Start event
                pp++;
            } else if (event.eventType == 1) {
                // Planar event
                p0++;
            } else {
                // End event
                pn++;
            }
            sweepIndex++;
        }

        nP = p0;
        nR -= pn + p0;

        AABB left, right;
        splitAABB(aabb, dim, planePos, &left, &right);
        const float saL = surfaceArea(left);
        const float saR = surfaceArea(right);
        const bool partitionLeft = saL < saR;
        const float cost = saL * (nL + partitionLeft * nP) + saR * (nR + (1 - partitionLeft) * nP);
        if (cost < optimalSplit.cost) {
            optimalSplit.cost = cost;
            optimalSplit.lCount = nL + partitionLeft * nP;
            optimalSplit.rCount = nR + (1 - partitionLeft) * nP;
            optimalSplit.pCount = nP;
            optimalSplit.splitAxis = dim;
            optimalSplit.splitPos = planePos;
            optimalSplit.planarToLeft = partitionLeft;
            assert(optimalSplit.lCount + optimalSplit.rCount + optimalSplit.pCount >= faceCount);
        }

        nL += pp + p0;
    }

    free(eventBuffer);

    return optimalSplit;
}

SplitInfo getOptimalPartitionSAHFaster(const AABB aabb,
                                       const ByteArray faceIndices,
                                       const ByteArray faces,
                                       const ByteArray vertices) {
    SplitInfo optimalSplit = { 0, 0, 0, 3, NAN, false, INFINITY };
    for (int i = 0; i < 3; i++) {
        SplitInfo split = getOptimalPartitionSAHFaster1Dim(aabb, faceIndices, faces, vertices, i);
        if (split.cost < optimalSplit.cost) {
            optimalSplit = split;
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
    countPartitions(3, aabb, median, splitTypes, faceIndices, faces, vertices,
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

void partitionSerialKD(const AABB aabb,
                       const ByteArray faceIndices,
                       const ByteArray faces,
                       const ByteArray vertices,
                       ByteArray *nodes,
                       ByteArray *leaves) {
    const int faceCount = faceIndices.count;

    // Get an optimal splitting plane for these polygons
    SplitInfo split = getOptimalPartitionSAHFaster(aabb, faceIndices, faces, vertices);

    // Build a leaf node if we hit termination condition
    if (shouldTerminate(split, aabb, faceCount)) {
        // Shove all faces into a leaf node
        *nodes = initByteArray("KDNode", 1, sizeof(KDNode));
        KDNode *leaf = getNodeFromArray(*nodes, 0);
        leaf->type = (faceCount << 2) | 3;
        leaf->aabb = aabb;
        leaf->leaf.dynamicListStart = leaves->count;
        int staticFaceCount = min(faceCount, MAX_STATIC_FACES);
        int dynamicFaceCount = max(0, faceCount - staticFaceCount);
        assert(staticFaceCount + dynamicFaceCount == faceCount);
        if (dynamicFaceCount > 0) {
            resizeByteArray(leaves, leaves->count + dynamicFaceCount);
        }

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
            PartitionCategory category = categorizeTriangle(t, aabb, split.splitAxis, split.splitPos);
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
    splitNode.aabb = aabb;
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

    ByteArray kdNodes = initByteArray("KDNode", 0, sizeof(KDNode));
    ByteArray kdLeaves = initByteArray("UnsignedInt", 0, sizeof(unsigned int));

    partitionSerialKDRoot(model->aabb, faces, vertices, &kdNodes, &kdLeaves);

    model->kdNodes = (KDNode *) kdNodes.data;
    model->nodeCount = kdNodes.count;
    model->kdLeaves = (unsigned int *) kdLeaves.data;
    model->leafCount = kdLeaves.count;
}

// Scene construction

simd_float3 transformPoint(simd_float3 p, Transform transform) {
    p *= transform.scale;
    p = simd_mul(p, transform.rotation);
    p += transform.translation;
    return p;
}

Transform identityTransform() {
    Transform identity;
    identity.scale = simd_make_float3(1.0f, 1.0f, 1.0f);
    identity.rotation = simd_diagonal_matrix(simd_make_float3(1.0f, 1.0f, 1.0f));
    identity.translation = simd_make_float3(0.0f, 0.0f, 0.0f);
    return identity;
}

AABB transformAABB(AABB aabb, Transform transform) {
    AABB transformed;
    simd_float3 v[8];
    v[0] = simd_make_float3(aabb.min.x, aabb.min.y, aabb.min.z);
    v[1] = simd_make_float3(aabb.min.x, aabb.min.y, aabb.max.z);
    v[2] = simd_make_float3(aabb.min.x, aabb.max.y, aabb.min.z);
    v[3] = simd_make_float3(aabb.min.x, aabb.max.y, aabb.max.z);
    v[4] = simd_make_float3(aabb.max.x, aabb.min.y, aabb.min.z);
    v[5] = simd_make_float3(aabb.max.x, aabb.min.y, aabb.max.z);
    v[6] = simd_make_float3(aabb.max.x, aabb.max.y, aabb.min.z);
    v[7] = simd_make_float3(aabb.max.x, aabb.max.y, aabb.max.z);

    transformed.min = simd_make_float3(INFINITY, INFINITY, INFINITY);
    transformed.max = simd_make_float3(-INFINITY, -INFINITY, -INFINITY);
    for (int i = 0; i < 8; i++) {
        simd_float3 transformedVert = transformPoint(v[i], transform);
        transformed = unionPointAndAABB(transformedVert, transformed);
    }
    return transformed;
}

Scene buildBasicScene(Model model) {
    Scene scene;

    AABB planeAABB;
    planeAABB.min = simd_make_float3(-2.5f, -0.01f, -2.5f);
    planeAABB.max = simd_make_float3( 2.5f,  0.01f,  2.5f);
    Box plane;
    plane.dimensions = planeAABB.max - planeAABB.min;
    plane.dimensions *= 0.99f;

    Instance floor;
    floor.primitive.type = 1;
    floor.primitive.box = plane;
    floor.primitive.box.material.material.diffColor = simd_make_float3(0.9f, 0.01f, 0.01f);
    floor.primitive.box.material.material.specColor = simd_make_float3(0.01f, 0.01f, 0.01f);
    floor.primitive.box.material.material.specPower = 1.0f;
    floor.primitive.box.material.numFaces = 1;
    floor.primitive.box.material.startFace = 0;
    floor.transform = identityTransform();
    floor.aabb = transformAABB(planeAABB, floor.transform);

    Instance wall;
    wall.primitive.type = 1;
    wall.primitive.box = plane;
    wall.primitive.box.material.material.diffColor = simd_make_float3(0.01f, 0.01f, 0.01f);
    wall.primitive.box.material.material.specColor = simd_make_float3(0.9f, 0.9f, 0.9f);
    wall.primitive.box.material.material.specPower = 100.0f;
    wall.primitive.box.material.numFaces = 1;
    wall.primitive.box.material.startFace = 0;
    wall.transform = identityTransform();
    wall.transform.rotation = simd_matrix3x3(simd_quaternion(3.1415f * 0.5f,
                                                             simd_make_float3(1.0f, 0.0f, 0.0f)));
    wall.transform.translation.z -= 0.5f;
    wall.transform.translation.y += 0.5f;
    wall.aabb = transformAABB(planeAABB, wall.transform);

    Instance modelRef;
    modelRef.primitive.type = 0;
    modelRef.primitive.modelRef.modelIndex = 0;
    modelRef.transform = identityTransform();
    float modelScale = 1.0f / simd_reduce_max(model.aabb.max - model.aabb.min);
    modelRef.transform.scale *= modelScale;
    modelRef.transform.translation -= model.centroid * modelScale;
    modelRef.transform.translation.y += 0.5 * modelScale * (model.aabb.max.y - model.aabb.min.y);
    modelRef.aabb = transformAABB(model.aabb, modelRef.transform);

    scene.aabb = unionAABB(unionAABB(wall.aabb, floor.aabb), modelRef.aabb);
    scene.instanceCount = 3;
    scene.instances = malloc(3 * sizeof(Instance));
    scene.instances[0] = wall;
    scene.instances[1] = floor;
    scene.instances[2] = modelRef;
    scene.modelCount = 1;
    scene.models = malloc(1 * sizeof(Model));
    scene.models[0] = model;

    return scene;
};

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

void pathTraceKernel(simd_int2 threadID, float seed,
                     simd_float4 *radiance /* inout */, int sampleCount,
                     Scene scene, simd_int2 res,
                     simd_float3 eye, simd_float3 lookAt, simd_float3 up, bool inPlace) {
    simd_float2 uv = simd_make_float2(threadID.x, threadID.y) / simd_make_float2(res.x, res.y);

    simd_float3 sumOfPaths = simd_make_float3(0.0f, 0.0f, 0.0f);
    simd_float3 throughPut = simd_make_float3(1.0f, 1.0f, 1.0f);
    float pdf = 1.0f;

    Ray ray = primaryRay(uv, simd_make_float2(res.x, res.y), eye, lookAt, up);
    for (int rayDepth = 0; rayDepth < 2; rayDepth++) {
        const Intersection intersection = intersectionScene(scene, ray, inPlace);
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
            const float ibl = 2.0f * fmax(0.0f, ray.dir.y);
            sumOfPaths += throughPut * ibl / pdf;
            break;
        }
    }

    simd_float3 result = (*radiance).xyz;
    result *= ((float) sampleCount) / (sampleCount + 1);
    result += sumOfPaths / (sampleCount + 1);
    *radiance = simd_make_float4(result, 1.0f);
}

// Tracing: external

typedef struct KernelArgs {
    float seed;
    Scene scene;
    simd_float4 *radiance;
    simd_uchar4 *pixels;
    int sampleCount;
    simd_int2 res;
    simd_int2 start;
    simd_int2 end;
    simd_float3 eye;
    simd_float3 lookAt;
    simd_float3 up;
    bool inPlace;
} KernelArgs;

void *addRadianceSampleSubImage(void *arguments) {
    const KernelArgs args = *(KernelArgs *) arguments;
    float seed = args.seed;
    Scene scene = args.scene;
    simd_float4 *radiance = args.radiance;
    simd_uchar4 *pixels = args.pixels;
    int sampleCount = args.sampleCount;
    simd_int2 res = args.res;
    simd_int2 start = args.start;
    simd_int2 end = args.end;
    simd_float3 eye = args.eye;
    simd_float3 lookAt = args.lookAt;
    simd_float3 up = args.up;
    bool inPlace = args.inPlace;

    float runningSeed = seed;
    for (int y = start.y; y < end.y; y++) {
        for (int x = start.x; x < end.x; x++) {
            simd_int2 threadID = simd_make_int2(x, y);
            int pixelIndex = (res.y - y - 1) * res.x + x;
            runningSeed = sqrt2Sequence(runningSeed);
            pathTraceKernel(threadID, runningSeed,
                            &radiance[pixelIndex], sampleCount,
                            scene, res, eye, lookAt, up, inPlace);

            simd_float4 clampedValue = simd_clamp(radiance[pixelIndex], 0.0, 1.0);
            simd_uchar4 charValue = simd_make_uchar4(clampedValue.x * 255,
                                                     clampedValue.y * 255,
                                                     clampedValue.z * 255,
                                                     clampedValue.w * 255);
            pixels[pixelIndex] = charValue;
        }
    }
    return NULL;
}

void addRadianceSample(Scene scene, unsigned int seed, int sampleCount,
                       simd_float4 *radiance, simd_uchar4 *pixels, simd_int2 res,
                       simd_float3 eye, simd_float3 lookAt, simd_float3 up, bool inPlace) {
    srand(seed);
    // Divide the image into a number of threads
    const int numThreads = 16;

    pthread_t threads[numThreads];
    KernelArgs args[numThreads];

    for (int i = 0; i < numThreads; i++) {
        args[i].seed = (float) rand() / (float)(RAND_MAX);
        args[i].scene = scene;
        args[i].radiance = radiance;
        args[i].pixels = pixels;
        args[i].sampleCount = sampleCount;
        args[i].res = res;
        args[i].start = simd_make_int2(0, (res.y * i) / numThreads);
        args[i].end = simd_make_int2(res.x, min(res.y, (res.y * (i + 1)) / numThreads));
        args[i].eye = eye;
        args[i].lookAt = lookAt;
        args[i].up = up;
        args[i].inPlace = inPlace;
    }

    for (int i = 0; i < numThreads; i++) {
        pthread_create(&threads[i], NULL, addRadianceSampleSubImage, (void *)&args[i]);
    }

    for (int i = 0; i < numThreads; i++) {
        pthread_join(threads[i], NULL);
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

        AABB clipped = clipBoxOfTriangle(t, box);
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

        AABB clipped = clipBoxOfTriangle(t, box);
        assert(isEmpty(clipped));
    }
}
