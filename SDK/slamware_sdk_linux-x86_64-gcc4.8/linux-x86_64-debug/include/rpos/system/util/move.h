/*
* move.h
* For those compilers doesn't support right value references, this module can help to simulate move semantics
* Spirit by boost::detail::thread_move_t<> which is used by boost::move which is used by boost::thread ^.^
*
* Created by Tony Huang (tony@slamtec.com) at 2016-4-17
* Copyright 2016 (c) Shanghai Slamtec Co., Ltd.
*/

#pragma once

#include "../../rpos_config.h"

namespace rpos { namespace system { namespace util {

    template < typename T >
    struct Move {
        T& payload;

        explicit Move(T& t)
            : payload(t)
        {}

        ~Move()
        {}

        T& operator*() const
        {
            return payload;
        }

        T* operator->() const
        {
            return &t;
        }

    private:
        Move(const Move<T>&) {}
        Move<T>& operator=(const Move<T>&) {}

#ifdef RPOS_HAS_RVALUE_REFS
        Move(Move<T>&&) {}
        Move<T>& operator=(Move<T>&&) {}
#endif
    };

    template < typename T >
    Move<T> move(T& payload)
    {
        return payload;
    }

} } }
