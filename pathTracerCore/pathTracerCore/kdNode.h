//
//  kdNode.h
//  kdTree
//
//  Created by Kyle Fung on 2019-02-10.
//

#pragma once

#include <simd/simd.h>

#if !__METAL__
#include <stdbool.h>
#endif

typedef struct Triangle {
    uint32_t v[3];
} Triangle;

typedef struct ExplicitTriangle {
    union {
        simd_float3 verts[3];
        struct {
            simd_float3 v0;
            simd_float3 v1;
            simd_float3 v2;
        };
    };
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

typedef struct Intersection {
    float distance; // NaN <=> miss
    simd_float3 normal;
    simd_float3 pos;
} Intersection;

typedef struct Transform {
    simd_float3 scale;
    simd_float3x3 rotation;
    simd_float3 translation;
} Transform;

typedef struct KDSplitNode {
    float split;
    // Left offset will always implicitly be 1.
    unsigned int right;
} KDSplitNode;

#define MAX_STATIC_FACES ((uint32_t)(sizeof(KDSplitNode) / sizeof(unsigned int) - 1))

typedef struct KDLeafNode {
    unsigned int staticList[MAX_STATIC_FACES];
    unsigned int dynamicListStart;
} KDLeafNode;

typedef struct KDNode {
    // First 2 bits encode node type (0 = x, 1 = y, 2 = z, 3 = triangle list).
    // Last 30 bits encode triangle count if triangle list.
    int type;
    AABB aabb;

    union {
        KDSplitNode split;
        KDLeafNode leaf;
    };
} KDNode;

typedef struct ModelRef {
    uint32_t modelIndex;
} ModelRef;

typedef struct Box {
    simd_float3 dimensions;
} Box;

typedef struct Sphere {
    float radius;
} Sphere;

typedef struct Primitive {
    uint8_t type; // 0 = Model, 1 = Box, 2 = Sphere
    union {
        ModelRef modelRef;
        Box box;
        Sphere sphere;
    };
} Primitive;

typedef struct Instance {
    AABB aabb; // AABB of the transformed AABB
    Transform transform;
    Primitive primitive;
} Instance;

struct Camera {
    simd_float3 pos;
    simd_float3 lookAt;
    simd_float3 up;
};

typedef struct ModelGPU {
    unsigned int faceStart;
    unsigned int vertexStart;
    uint32_t faceCount;
    uint32_t vertCount;
    simd_float3 centroid;
    AABB aabb;
    unsigned int kdNodeStart;
    unsigned int kdLeafStart;
} ModelGPU;

typedef struct SceneGPU {
    AABB aabb;
    uint32_t modelCount;
    unsigned int instanceStart;
    uint32_t instanceCount;
} SceneGPU;

// CPU only
#if !__METAL__
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

typedef struct Model {
    Triangle *faces;
    Vertex *vertices;
    uint32_t faceCount;
    uint32_t vertCount;
    simd_float3 centroid;
    AABB aabb;
    KDNode *kdNodes;
    unsigned int *kdLeaves;
} Model;

typedef struct Scene {
    AABB aabb;
    Model *models;
    uint32_t modelCount;
    Instance *instances;
    uint32_t instanceCount;
} Scene;

// Build scene
Scene buildBasicScene(Model model);

// Partitioning
void partitionModel(Model *model);

// Tracing
void addRadianceSample(Scene scene, unsigned int seed, int sampleCount,
                       simd_float4 *radiance, simd_uchar4 *pixels, simd_int2 res,
                       simd_float3 eye, simd_float3 lookAt, simd_float3 up, bool inPlace);

// Testing
void runTests(void);
#endif
