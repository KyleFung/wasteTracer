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
