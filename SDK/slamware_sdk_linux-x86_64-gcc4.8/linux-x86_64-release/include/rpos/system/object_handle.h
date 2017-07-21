/*
* object_handle.h
* Object Handle
*
* Created By Tony Huang (cnwzhjs@gmail.com) at 2014-5-22
* Copyright 2014 (c) www.robopeak.com
*/

#pragma once

#include <rpos/rpos_config.h>
#include <rpos/core/rpos_core_config.h>
#include <boost/shared_ptr.hpp>

namespace rpos { namespace system {

    template <class Klass, class KlassImpl>
    class ObjectHandle {
    public:
        typedef boost::shared_ptr<KlassImpl> ImplPointer;

        ~ObjectHandle()
        {}

        Klass& operator=(const ObjectHandle<Klass, KlassImpl>& that)
        {
            impl_ = that.impl_;
            return (Klass&)*this;
        }

        Klass& operator=(const Klass& that)
        {
            impl_ = that.impl_;
            return (Klass&)*this;
        }

        operator bool() const
        {
            return (bool)impl_;
        }

#if defined(RPOS_HAS_RVALUE_REFS) && !defined(__GNUC__)
        Klass& operator=(ObjectHandle<Klass, KlassImpl>&& that)
        {
            std::swap(impl_, that.impl_);
            return (Klass&)*this;
        }
            
        Klass& operator=(Klass&& that)
        {
            std::swap(impl_, that.impl_);
            return (Klass&)*this;
        }
#endif

    protected:
        ObjectHandle()
        {}

        ObjectHandle(ImplPointer impl)
            : impl_(impl)
        {}

        ObjectHandle(const ObjectHandle<Klass, KlassImpl>& that)
            : impl_(that.impl_)
        {}

#if defined(RPOS_HAS_RVALUE_REFS) && !defined(__GNUC__)
        ObjectHandle(ObjectHandle<Klass, KlassImpl>&& that)
        {
            std::swap(impl_, that.impl_);
        }
#endif

        ImplPointer impl_;
    };

} }

#define RPOS_CORE_OBJECT_DEFAULT_CTOR_WITH_BASE(Klass, BaseKlass) \
    Klass();

#define RPOS_CORE_OBJECT_POINTER_CTOR_WITH_BASE(Klass, BaseKlass) \
    Klass(boost::shared_ptr<detail::Klass ## Impl> impl);

#define RPOS_CORE_OBJECT_CONST_REF_CTOR_WITH_BASE(Klass, BaseKlass) \
    Klass(const Klass& that);

#define RPOS_CORE_OBJECT_RVALUE_REF_CTOR_WITH_BASE(Klass, BaseKlass) \
    Klass(Klass&& that);


#define RPOS_CORE_OBJECT_DEFAULT_CTOR(Klass) \
    Klass();

#define RPOS_CORE_OBJECT_POINTER_CTOR(Klass) \
    Klass(boost::shared_ptr<detail::Klass ## Impl> impl);

#define RPOS_CORE_OBJECT_CONST_REF_CTOR(Klass) \
    Klass(const Klass& that);

#define RPOS_CORE_OBJECT_RVALUE_REF_CTOR(Klass) \
    Klass(Klass&& that);


#define RPOS_CORE_OBJECT_DEFAULT_CTOR_WITH_BASE_IMPL(Klass, BaseKlass) \
    Klass::Klass() : BaseKlass() {}

#define RPOS_CORE_OBJECT_POINTER_CTOR_WITH_BASE_IMPL(Klass, BaseKlass) \
    Klass::Klass(boost::shared_ptr<detail::Klass ## Impl> impl) : BaseKlass(impl) {}

#define RPOS_CORE_OBJECT_CONST_REF_CTOR_WITH_BASE_IMPL(Klass, BaseKlass) \
    Klass::Klass(const Klass& that) : BaseKlass(that) {}

#define RPOS_CORE_OBJECT_RVALUE_REF_CTOR_WITH_BASE_IMPL(Klass, BaseKlass) \
    Klass::Klass(Klass&& that) : BaseKlass(that) {}


#define RPOS_CORE_OBJECT_DEFAULT_CTOR_IMPL(Klass) \
    Klass::Klass() : rpos::system::ObjectHandle<Klass, detail::Klass ## Impl>() {}

#define RPOS_CORE_OBJECT_POINTER_CTOR_IMPL(Klass) \
    Klass::Klass(boost::shared_ptr<detail::Klass ## Impl> impl) : rpos::system::ObjectHandle<Klass, detail::Klass ## Impl>(impl) {}

#define RPOS_CORE_OBJECT_CONST_REF_CTOR_IMPL(Klass) \
    Klass::Klass(const Klass& that) : rpos::system::ObjectHandle<Klass, detail::Klass ## Impl>(that) {}

#define RPOS_CORE_OBJECT_RVALUE_REF_CTOR_IMPL(Klass) \
    Klass::Klass(Klass&& that) : rpos::system::ObjectHandle<Klass, detail::Klass ## Impl>(that) {}

#if defined(RPOS_HAS_RVALUE_REFS) && !defined(__GNUC__)
    #define RPOS_OBJECT_CTORS_WITH_BASE(Klass, BaseKlass) \
                RPOS_CORE_OBJECT_DEFAULT_CTOR_WITH_BASE(Klass, BaseKlass) \
                RPOS_CORE_OBJECT_CONST_REF_CTOR_WITH_BASE(Klass, BaseKlass) \
                RPOS_CORE_OBJECT_RVALUE_REF_CTOR_WITH_BASE(Klass, BaseKlass)

    #define RPOS_OBJECT_CTORS(Klass) \
                RPOS_CORE_OBJECT_DEFAULT_CTOR(Klass) \
                RPOS_CORE_OBJECT_POINTER_CTOR(Klass) \
                RPOS_CORE_OBJECT_CONST_REF_CTOR(Klass) \
                RPOS_CORE_OBJECT_RVALUE_REF_CTOR(Klass)

    #define RPOS_OBJECT_CTORS_WITH_BASE_IMPL(Klass, BaseKlass) \
                RPOS_CORE_OBJECT_DEFAULT_CTOR_WITH_BASE_IMPL(Klass, BaseKlass) \
                RPOS_CORE_OBJECT_CONST_REF_CTOR_WITH_BASE_IMPL(Klass, BaseKlass) \
                RPOS_CORE_OBJECT_RVALUE_REF_CTOR_WITH_BASE_IMPL(Klass, BaseKlass)

    #define RPOS_OBJECT_CTORS_IMPL(Klass) \
                RPOS_CORE_OBJECT_DEFAULT_CTOR_IMPL(Klass) \
                RPOS_CORE_OBJECT_POINTER_CTOR_IMPL(Klass) \
                RPOS_CORE_OBJECT_CONST_REF_CTOR_IMPL(Klass) \
                RPOS_CORE_OBJECT_RVALUE_REF_CTOR_IMPL(Klass)
#else
    #define RPOS_OBJECT_CTORS_WITH_BASE(Klass, BaseKlass) \
                RPOS_CORE_OBJECT_DEFAULT_CTOR_WITH_BASE(Klass, BaseKlass) \
                RPOS_CORE_OBJECT_CONST_REF_CTOR_WITH_BASE(Klass, BaseKlass)

    #define RPOS_OBJECT_CTORS(Klass) \
                RPOS_CORE_OBJECT_DEFAULT_CTOR(Klass) \
                RPOS_CORE_OBJECT_POINTER_CTOR(Klass) \
                RPOS_CORE_OBJECT_CONST_REF_CTOR(Klass)

    #define RPOS_OBJECT_CTORS_WITH_BASE_IMPL(Klass, BaseKlass) \
                RPOS_CORE_OBJECT_DEFAULT_CTOR_WITH_BASE_IMPL(Klass, BaseKlass) \
                RPOS_CORE_OBJECT_CONST_REF_CTOR_WITH_BASE_IMPL(Klass, BaseKlass)

    #define RPOS_OBJECT_CTORS_IMPL(Klass) \
                RPOS_CORE_OBJECT_DEFAULT_CTOR_IMPL(Klass) \
                RPOS_CORE_OBJECT_POINTER_CTOR_IMPL(Klass) \
                RPOS_CORE_OBJECT_CONST_REF_CTOR_IMPL(Klass)
#endif
