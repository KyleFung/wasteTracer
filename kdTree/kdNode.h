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

typedef struct Material {
    char *materialName;
    simd_float3 diffColor;
    simd_float3 specColor;
    float specPower;
    char *diffMapName;
    uint32_t diffMapIndex;
} Material;

typedef struct Texture {
    char *textureName;
    uint8_t *data;
    uint32_t width;
    uint32_t height;
} Texture;

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

typedef struct Model {
    Triangle *faces;
    Vertex *vertices;
    Material *materials;
    uint32_t faceCount;
    uint32_t vertCount;
    uint32_t matCount;
} Model;

AABB emptyBox() {
    AABB box = { simd_make_float3(NAN), simd_make_float3(NAN) };
    return box;
}

float intersectionBox(AABB b, Ray r);
float intersectionTriangle(Triangle t, Vertex v[], Ray r);
float intersectionModel(Model model, Ray r);
simd_float3 normalOf(ExplicitTriangle t);
