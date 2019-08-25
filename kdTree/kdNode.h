//
//  kdNode.h
//  kdTree
//
//  Created by Kyle Fung on 2019-02-10.
//

#pragma once

#include <simd/simd.h>
#include <stdbool.h>
#include <string.h>

typedef struct Triangle {
    uint32_t v[3];
} Triangle;

typedef struct ExplicitTriangle {
    simd_float3 v0;
    simd_float3 v1;
    simd_float3 v2;
} ExplicitTriangle;

typedef struct Vertex {
    simd_float3 pos;
    simd_float2 uv;
} Vertex;

typedef struct Ray {
    simd_float3 pos;
    simd_float3 dir;
} Ray;

typedef struct AABB {
    simd_float3 max;
    simd_float3 min;
} AABB;

AABB emptyBox() {
    AABB box = { simd_make_float3(NAN), simd_make_float3(NAN) };
    return box;
}

float intersectionBox(AABB b, Ray r);
float intersectionTriangle(Triangle t, Vertex v[], Ray r);
simd_float3 normalOf(ExplicitTriangle t);
