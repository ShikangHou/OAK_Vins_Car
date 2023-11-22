#ifndef DEPTH_TRAITS_H_
#define DEPTH_TRAITS_H_

#include <iostream>
#include <cmath>

namespace depthimage_to_laserscan
{
    template <typename T>
    struct DepthTraits
    {
    };

    template <>
    struct DepthTraits<uint16_t>
    {
        static inline bool valid(uint16_t depth) { return (depth != 0); }
        static inline float toMeters(uint16_t depth) { return depth * 0.001f; }
    };

    template <>
    struct DepthTraits<float>
    {
        static inline bool valid(float depth) { return std::isfinite(depth);} //是不是有限值
        static inline float toMeters(float depth) { return depth; }
    };

}

#endif
