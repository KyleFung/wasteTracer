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

simd_float3 normalOf(ExplicitTriangle t) {
    return simd_normalize(simd_cross(t.v1 - t.v0, t.v2 - t.v0));
}

float intersectionTriangle(Triangle triangle, Vertex vertexList[], Ray r) {
    ExplicitTriangle t;
    t.v0 = vertexList[triangle.v[0]].pos;
    t.v1 = vertexList[triangle.v[1]].pos;
    t.v2 = vertexList[triangle.v[2]].pos;

    // Triangle plane
    simd_float3 nor = normalOf(t);
    float d = simd_dot(nor, t.v0);

    // Plane intersection
    float hit = (d - simd_dot(nor, r.pos)) / simd_dot(nor, r.dir);
    simd_float3 hitPos = hit * r.dir + r.pos;

    // Plane intersection behind ray (miss)
    if (hit <= 0) {
        return -10000.0f;
    }

    // Inside outside test
    float side0 = simd_dot(nor, simd_cross(hitPos - t.v0, t.v1 - t.v0));
    float side1 = simd_dot(nor, simd_cross(hitPos - t.v1, t.v2 - t.v1));
    float side2 = simd_dot(nor, simd_cross(hitPos - t.v2, t.v0 - t.v2));

    if (side0 * side1 > 0 && side1 * side2 > 0) {
        // Intersection
        return hit;
    } else {
        // Not inside triangle (miss)
        return -10000.0f;
    }
}

//
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
