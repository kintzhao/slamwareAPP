#pragma once

#include "target_info.h"
#include <cstdint>
#include <algorithm>

namespace rpos { namespace system {

    template<class T>
    static inline void byte_swap_inplace(const T* p)
    {
        uint8_t *ptr = reinterpret_cast<uint8_t*>(p);
        std::reverse(ptr, ptr + sizeof(T));
    }

    template<class T>
    static inline T byte_swap(T v)
    {
        T r = v;
        byte_swap_inplace<T>(&r);
        return r;
    }

    static inline std::uint16_t byte_swap(std::uint16_t v)
    {
        return ((v & 0x00ffu) << 8) 
             | ((v & 0xff00u) >> 8);
    }

    static inline std::uint32_t byte_swap(std::uint32_t v)
    {
        return ((v & 0x000000ffu) << 24)
            | ((v & 0x0000ff00u) << 8)
            | ((v & 0x00ff0000u) >> 8)
            | ((v & 0xff000000u) >> 24);
    }

    static inline std::uint64_t byte_swap(std::uint64_t v)
    {
        return ((v & 0x00000000000000ffu) << 56)
            | ((v & 0x000000000000ff00u) << 40)
            | ((v & 0x0000000000ff0000u) << 24)
            | ((v & 0x00000000ff000000u) << 8)
            | ((v & 0x000000ff00000000u) >> 8)
            | ((v & 0x0000ff0000000000u) >> 24)
            | ((v & 0x00ff000000000000u) >> 40)
            | ((v & 0xff00000000000000u) >> 56);
    }

    template < class T >
    static inline T cpu_to_le(T v)
    {
#ifdef RPOS_TARGET_BE
        return byte_swap(v);
#else
        return v;
#endif
    }

    template < class T >
    static inline T le_to_cpu(T v)
    {
#ifdef RPOS_TARGET_BE
        return byte_swap(v);
#else
        return v;
#endif
    }

    template < class T >
    static inline T cpu_to_be(T v)
    {
#ifdef RPOS_TARGET_LE
        return byte_swap(v);
#else
        return v;
#endif
    }

    template < class T >
    static inline T be_to_cpu(T v)
    {
#ifdef RPOS_TARGET_LE
        return byte_swap(v);
#else
        return v;
#endif
    }

} }
