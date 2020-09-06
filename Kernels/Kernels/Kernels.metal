#include <metal_stdlib>

#define __METAL__ 1
#include "../../pathTracerCore/pathTracerCore/kdNode.h"
using namespace metal;

kernel void uvKernel(texture2d<half, access::write> out [[texture(0)]],
                     uint2                          gid [[thread_position_in_grid]]) {
    const uint2 textureSize = {out.get_width(), out.get_height()};
    half2 uv = static_cast<half2>(gid) / static_cast<half2>(textureSize);
    out.write(half4(uv, 0.0h, 1.0h), gid);
}

Ray primaryRay(simd_float2 uv, simd_float2 res, simd_float3 eye, simd_float3 lookAt, simd_float3 up) {
    const float focal = 1.0f;
    const float ar = res.x / res.y;
    const float screenHeight = 2.0f;
    const float screenWidth = ar * screenHeight;

    simd_float3 right = cross(lookAt, up);

    float screenX = (2.0f * uv.x) - 1.0f;
    float screenY = (2.0f * uv.y) - 1.0f;

    simd_float3 u = screenX * normalize(right) * screenWidth * 0.5f;
    simd_float3 v = screenY * normalize(up) * screenHeight * 0.5f;
    simd_float3 dir = normalize(focal * normalize(lookAt) + u + v);

    Ray ray;
    ray.dir = dir;
    ray.pos = eye;
    return ray;
}

float intersectionAABB(AABB b, Ray r) {
    float tmin = -INFINITY, tmax = INFINITY;

    for (int i = 0; i < 3; ++i) {
        float t1 = (b.min[i] - r.pos[i]) / r.dir[i];
        float t2 = (b.max[i] - r.pos[i]) / r.dir[i];

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
    simd_float3 q = abs(pos) - boxDim;
    return length(max(q, 0.0)) + min(max(q.x, max(q.y,q.z)), 0.0);
}

bool intersectsAABB(AABB b, Ray r) {
    float intersection = intersectionAABB(b, r);
    return isfinite(intersection) && intersection > 0.0;
}

bool inBox(simd_float3 p, const AABB box) {
    return all(p < box.max && p > box.min);
}

MaterialQuery emptyQuery() {
    MaterialQuery query;
    query.faceID = -1;
    query.materialCount = -1;
    query.materialLUTStart = -1;
    return query;
}

bool isEmptyQuery(MaterialQuery query) {
    return query.faceID == static_cast<unsigned int>(-1) ||
           query.materialLUTStart == static_cast<unsigned int>(-1) ||
           query.materialCount == static_cast<unsigned int>(-1);
}

Intersection makeIntersection(float distance, simd_float3 normal, simd_float3 pos, MaterialQuery materialQuery) {
    Intersection intersection = { distance, normal, pos, materialQuery };
    return intersection;
}

Intersection missedIntersection() {
    Intersection miss = { NAN, float3(NAN), float3(NAN), emptyQuery() };
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

Intersection intersectionBox(BoxGPU b, Ray r) {
    AABB aabb;
    aabb.min = -b.dimensions * 0.5f;
    aabb.max =  b.dimensions * 0.5f;

    float t = intersectionAABB(aabb, r);
    if (isfinite(t)) {
        simd_float3 pos = r.pos + t * r.dir;
        float d = boxSDF(pos, b.dimensions);
        simd_float3 eps = float3(0.0000001f, 0.0f, 0.0f);
        simd_float3 dev = float3(boxSDF(pos + eps.xyz, b.dimensions),
                                 boxSDF(pos + eps.yxz, b.dimensions),
                                 boxSDF(pos + eps.zyx, b.dimensions));
        simd_float3 normal = normalize(dev - d);

        MaterialQuery query;
        query.faceID = 0;
        query.materialLUTStart = b.materialLUTStart;
        query.materialCount = 1;

        return makeIntersection(t, normal, pos, query);
    } else {
        return missedIntersection();
    }
}

simd_float3 normalOf(ExplicitTriangle t) {
    return normalize(cross(t.v1 - t.v0, t.v2 - t.v0));
}

Intersection intersectionTriangle(ExplicitTriangle t, Ray r, MaterialQuery query) {
    // Triangle plane
    simd_float3 nor = normalOf(t);
    float d = dot(nor, t.v0);

    // Plane intersection
    float hit = (d - dot(nor, r.pos)) / dot(nor, r.dir);
    simd_float3 hitPos = hit * r.dir + r.pos;

    // Plane intersection behind ray (miss)
    if (hit <= 0) {
        return missedIntersection();
    }

    // Inside outside test
    float side0 = dot(nor, cross(hitPos - t.v0, t.v1 - t.v0));
    float side1 = dot(nor, cross(hitPos - t.v1, t.v2 - t.v1));
    float side2 = dot(nor, cross(hitPos - t.v2, t.v0 - t.v2));

    if (side0 * side1 >= 0 && side1 * side2 >= 0) {
        // Intersection
        return makeIntersection(hit, nor, hitPos, query);
    } else {
        // Not inside triangle (miss)
        return missedIntersection();
    }
}

ExplicitTriangle makeExplicitFace(constant const Vertex vertices[],
                                  const Triangle t) {
    ExplicitTriangle explicitFace;
    explicitFace.v0 = vertices[t.v[0]].pos;
    explicitFace.v1 = vertices[t.v[1]].pos;
    explicitFace.v2 = vertices[t.v[2]].pos;
    return explicitFace;
}

void setBit(thread unsigned int* x, short n, bool value) {
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

void backUpTraversal(thread TraversalStack *stack, constant const KDNode *nodes, Intersection topResult) {
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
    constant const KDNode *parent = &nodes[parentIndex];
    assert(parent->type <= 2);
    stack->nodeStack[stack->stackPointer] = parentIndex + (leftCloser ? parent->split.right : 1);
    // Replace the top 0 with a 1
    setBit(&stack->traversedPath, stack->stackPointer, true);
}

void traverseToCloser(thread TraversalStack *stack, constant const KDNode *nodes, Ray r) {
    // Take a fork to the closer child and push that onto the stack
    stack->stackPointer += 1;
    assert(stack->stackPointer < 32);

    constant const KDNode *parent = &nodes[stack->nodeStack[stack->stackPointer - 1]];
    bool leftCloser = r.pos[parent->type] < parent->split.split;
    setBit(&stack->leftCloser, stack->stackPointer, leftCloser);

    stack->nodeStack[stack->stackPointer] = stack->nodeStack[stack->stackPointer - 1] +
                                            (leftCloser ? 1 : parent->split.right);
    setBit(&stack->traversedPath, stack->stackPointer, false);
}

Intersection intersectionTreeInPlace(constant const KDNode *nodes, constant const unsigned int *leaves,
                                     constant const Triangle *faces, constant const Vertex *vertices, Ray r) {
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
            for (unsigned int i = 0; i < staticLeafCount; i++) {
                const unsigned int faceIndex = leaf.staticList[i];
                const Triangle triangle = faces[faceIndex];
                ExplicitTriangle t = makeExplicitFace(vertices, triangle);

                MaterialQuery query = emptyQuery();
                query.faceID = faceIndex;

                Intersection triIntersect = intersectionTriangle(t, r, query);
                leafIntersection = closestIntersection(leafIntersection, triIntersect);
            }

            int dynamicLeafCount = leafCount - staticLeafCount;
            if (dynamicLeafCount < 0) {
                dynamicLeafCount = 0;
            }
            for (int i = 0; i < dynamicLeafCount; i++) {
                const unsigned int faceIndex = leaves[leaf.dynamicListStart + i];
                const Triangle triangle = faces[faceIndex];
                ExplicitTriangle t = makeExplicitFace(vertices, triangle);

                MaterialQuery query = emptyQuery();
                query.faceID = faceIndex;

                Intersection triIntersect = intersectionTriangle(t, r, query);
                leafIntersection = closestIntersection(leafIntersection, triIntersect);
            }

            closest = closestIntersection(closest, leafIntersection);

            // Back up
            backUpTraversal(&stack, nodes, leafIntersection);
        }
    }

    return closest;
}

Intersection applyTransform(Intersection intersection, Transform transform) {
    Intersection result = intersection;
    result.pos *= transform.scale;
    result.pos = transform.rotation * result.pos;
    result.pos += transform.translation;
    result.normal = transform.rotation * result.normal;
    return result;
}

Intersection intersectionModel(ModelGPU model, Ray r,
                               constant const KDNode *nodes, constant const unsigned int *leaves,
                               constant const Triangle *faces, constant const Vertex *vertices) {
    Intersection modelIntersection;

    modelIntersection = intersectionTreeInPlace(nodes + model.kdNodeStart, leaves + model.kdLeafStart,
                                                faces + model.faceStart, vertices + model.vertexStart, r);
    modelIntersection.materialQuery.materialLUTStart = model.materialLUTStart;
    modelIntersection.materialQuery.materialCount = model.materialCount;

    return modelIntersection;
}

Intersection intersectionInstance(InstanceGPU instance, constant const ModelGPU *modelArray, Ray r,
                                  constant const KDNode *nodes, constant const unsigned int *leaves,
                                  constant const Triangle *faces, constant const Vertex *vertices) {
    simd_float3 oldPos = r.pos;

    // Perform instance inverse transform on the ray
    r.pos -= instance.transform.translation;
    r.pos = transpose(instance.transform.rotation) * r.pos;
    r.pos /= instance.transform.scale;
    r.dir = transpose(instance.transform.rotation) * r.dir;

    Intersection intersection = missedIntersection();
    switch (instance.primitive.type) {
        case 0: {
            uint32_t modelIndex = instance.primitive.modelRef.modelIndex;
            intersection = intersectionModel(modelArray[modelIndex], r, nodes, leaves, faces, vertices);
            break;
        }
        case 1: {
            BoxGPU box = instance.primitive.box;
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
        intersection.distance = length(intersection.pos - oldPos);
        return intersection;
    } else {
        return missedIntersection();
    }
}

Intersection intersectionScene(SceneGPU scene, Ray r,
                               constant const InstanceGPU *instances, constant const ModelGPU *models,
                               constant const KDNode *nodes, constant const unsigned int *leaves,
                               constant const Triangle *faces, constant const Vertex *vertices) {
    if (!intersectsAABB(scene.aabb, r)) {
        return missedIntersection();
    }

    Intersection closest = missedIntersection();
    for (unsigned int i = 0; i < scene.instanceCount; i++) {
        InstanceGPU instance = instances[scene.instanceStart + i];
        if (intersectsAABB(instance.aabb, r)) {
            Intersection instanceIntersection = intersectionInstance(instance, models, r,
                                                                     nodes, leaves, faces, vertices);
            closest = closestIntersection(closest, instanceIntersection);
        }
    }

    return closest;
}

Material defaultMaterial() {
    Material mat;
    mat.diffColor = float3(0.7, 0.7, 0.7);
    mat.specColor = float3(0.1, 0.1, 0.1);
    mat.specPower = 2.0f;
    return mat;
}

Material resolveMaterial(constant const MaterialLookup *materialLUT, MaterialQuery query) {
    if (isEmptyQuery(query)) {
        return defaultMaterial();
    }

    for (unsigned int i = 0; i < query.materialCount; i++) {
        unsigned int matIndex = query.materialLUTStart + i;
        MaterialLookup entry = materialLUT[matIndex];
        if (entry.startFace <= query.faceID && query.faceID < entry.startFace + entry.numFaces) {
            return entry.material;
        }
    }
    return defaultMaterial();
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
    float3 tc = float3(1.0 + nor.z - nor.xy * nor.xy, -nor.x * nor.y) / (1.0 + nor.z);
    float3 uu = float3(tc.x, tc.z, -nor.x);
    float3 vv = float3(tc.z, tc.y, -nor.y);

    float u = fabs(hash(78.233 + seed));
    float v = fabs(hash(10.873 + seed));
    float a = 6.283185 * v;

    return sqrt(u) * (cos(a) * uu + sin(a) * vv) + sqrt(1.0 - u) * nor;
}

kernel void intersectionKernel(texture2d<half, access::write> outRadiance [[texture(0)]],
                               texture2d<half, access::read>  inRadiance  [[texture(1)]],
                               constant SceneGPU&             scene       [[buffer(0)]],
                               constant InstanceGPU*          instances   [[buffer(1)]],
                               constant ModelGPU*             models      [[buffer(2)]],
                               constant Camera&               camera      [[buffer(3)]],
                               constant KDNode*               nodes       [[buffer(4)]],
                               constant unsigned int*         leaves      [[buffer(5)]],
                               constant Triangle*             faces       [[buffer(6)]],
                               constant Vertex*               vertices    [[buffer(7)]],
                               constant unsigned int&         numSamples  [[buffer(8)]],
                               constant unsigned int&         newSamples  [[buffer(9)]],
                               constant MaterialLookup*       materialLUT [[buffer(10)]],
                               uint2                          gid [[thread_position_in_grid]]) {
    const uint2 textureSize = {outRadiance.get_width(), outRadiance.get_height()};
    float2 uv = static_cast<float2>(gid) / static_cast<float2>(textureSize);
    Ray cameraRay = primaryRay(uv, float2(textureSize), camera.pos, camera.lookAt, camera.up);

    float3 result = numSamples == 0 ? float3(0) : float3(inRadiance.read(gid).xyz);

    for (unsigned int i = 0; i < newSamples; i++) {
        // Set up for generating a new sample.
        unsigned int sample = numSamples + i;
        float seed = fract((sin(dot(uv, float2(12.9898, 78.233))) * (43758.5453123)) + sample * 2.23606798);

        float3 sumOfPaths = float3(0.0f, 0.0f, 0.0f);
        float3 throughPut = float3(1.0f, 1.0f, 1.0f);
        float pdf = 1.0f;

        Ray ray = cameraRay;
        for (int rayDepth = 0; rayDepth < 2; rayDepth++) {
            const Intersection intersection = intersectionScene(scene, ray,
                                                                instances, models, nodes, leaves, faces, vertices);
            if (isHit(intersection)) {
                // Resolve the material of the hit point
                Material material = resolveMaterial(materialLUT, intersection.materialQuery);

                // Hit geometry (continue tracing)
                ray.pos = intersection.pos + 0.01 * intersection.normal;
                seed = goldenSequence(seed);
                ray.dir = normalize(cosineDirection(seed, intersection.normal));

                // Increment the ray (assuming that no geometry is emissive)
                const float pi_inverse = 1.0f / 3.14159f;
                const float3 brdf = material.diffColor * pi_inverse;
                const float geometry = dot(ray.dir, intersection.normal);
                pdf *= pi_inverse;
                throughPut *= brdf * geometry;
            } else {
                // Hit the skybox (uniform radiance)
                const float ibl = 2.0f * max(0.0f, ray.dir.y);
                sumOfPaths += throughPut * ibl / pdf;
                break;
            }
        }

        result *= ((float) sample) / (sample + 1);
        result += (sumOfPaths) / (sample + 1);
    }

    outRadiance.write(half4(half3(result), 1.0h), gid);
}
