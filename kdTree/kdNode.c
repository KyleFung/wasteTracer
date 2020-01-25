#include "kdNode.h"

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
        return makeIntersection(hit, nor);
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

Intersection makeIntersection(float distance, simd_float3 normal) {
    Intersection intersection = { distance, normal };
    return intersection;
}

Intersection missedIntersection() {
    Intersection miss = { NAN, simd_make_float3(NAN) };
    return miss;
}

AABB emptyBox() {
    AABB box = { simd_make_float3(NAN), simd_make_float3(NAN) };
    return box;
}

//float intersectTree(kdNode *tree, int index, int maxIndex, AABB box, Ray ray, int dim) {
//    // Check base case or complete miss
//    const float boxIntersection = intersectionBox(box, ray);
//    if (boxIntersection == INFINITY)
//        return boxIntersection;
//    if (index * 2 >= maxIndex) {
//        // Do primitive intersection
//    }
//
//    // Recurse
//    AABB boxL = getLBox(tree[index].median, dim, box);
//    AABB boxR = getRBox(tree[index].median, dim, box);
//
//    float intL = intersectTree(tree, 2 * index, maxIndex, boxL, ray, (dim + 1) % 3);
//    float intR = intersectTree(tree, 2 * index + 1, maxIndex, boxR, ray, (dim + 1) % 3);
//    return min(intL, intR);
//}
