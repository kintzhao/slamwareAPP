/**
* auto_link.h
* Automatically link relative rpos library
* Just take advantage of boost/config/auto_link.hpp
*
* Created By Tony Huang @ 2014-6-12
* Copyright (c) 2014 Shanghai SlamTec Co., Ltd.
*/

/**
* USAGE
*
* RPOS_LIB_NAME: such as rpos_core
*/

#include <rpos/rpos_version.h>
#include <boost/config.hpp>

#if defined(_MSC_VER) && !defined(__MWERKS__) && !defined(__EDG_VERSION__)
//
// C language compatability (no, honestly)
//
#  define RPOS_MSVC _MSC_VER
#  define RPOS_STRINGIZE(X) RPOS_DO_STRINGIZE(X)
#  define RPOS_DO_STRINGIZE(X) #X
#endif

//
// Only include what follows for known and supported compilers:
//
#if defined(RPOS_MSVC) \
    || defined(__BORLANDC__) \
    || (defined(__MWERKS__) && defined(_WIN32) && (__MWERKS__ >= 0x3000)) \
    || (defined(__ICL) && defined(_MSC_EXTENSIONS) && (_MSC_VER >= 1200))

#ifndef RPOS_LIB_NAME
#  error "Macro RPOS_LIB_NAME not set (internal error)"
#endif

//
// error check:
//
#if defined(__MSVC_RUNTIME_CHECKS) && !defined(_DEBUG)
#  pragma message("Using the /RTC option without specifying a debug runtime will lead to linker errors")
#  pragma message("Hint: go to the code generation options and switch to one of the debugging runtimes")
#  error "Incompatible build options"
#endif
//
// select toolset if not defined already:
//
#ifndef RPOS_LIB_TOOLSET
#  if defined(RPOS_MSVC) && (RPOS_MSVC < 1200)
// Note: no compilers before 1200 are supported
#  elif defined(RPOS_MSVC) && (RPOS_MSVC < 1300)

#    ifdef UNDER_CE
// eVC4:
#      define RPOS_LIB_TOOLSET "evc4"
#    else
// vc6:
#      define RPOS_LIB_TOOLSET "vc6"
#    endif

#  elif defined(RPOS_MSVC) && (RPOS_MSVC < 1310)

// vc7:
#    define RPOS_LIB_TOOLSET "vc7"

#  elif defined(RPOS_MSVC) && (RPOS_MSVC < 1400)

// vc71:
#    define RPOS_LIB_TOOLSET "vc71"

#  elif defined(RPOS_MSVC) && (RPOS_MSVC < 1500)

// vc80:
#    define RPOS_LIB_TOOLSET "vc80"

#  elif defined(RPOS_MSVC) && (RPOS_MSVC < 1600)

// vc90:
#    define RPOS_LIB_TOOLSET "vc90"

#  elif defined(RPOS_MSVC) && (RPOS_MSVC < 1700)

// vc10:
#    define RPOS_LIB_TOOLSET "vc100"

#  elif defined(RPOS_MSVC) && (RPOS_MSVC < 1800)

// vc11:
#    define RPOS_LIB_TOOLSET "vc110"

#  elif defined(RPOS_MSVC)

// vc12:
#    define RPOS_LIB_TOOLSET "vc120"

#  elif defined(__BORLANDC__)

// CBuilder 6:
#    define RPOS_LIB_TOOLSET "bcb"

#  elif defined(__ICL)

// Intel C++, no version number:
#    define RPOS_LIB_TOOLSET "iw"

#  elif defined(__MWERKS__) && (__MWERKS__ <= 0x31FF )

// Metrowerks CodeWarrior 8.x
#    define RPOS_LIB_TOOLSET "cw8"

#  elif defined(__MWERKS__) && (__MWERKS__ <= 0x32FF )

// Metrowerks CodeWarrior 9.x
#    define RPOS_LIB_TOOLSET "cw9"

#  endif
#endif // RPOS_LIB_TOOLSET

//
// select thread opt:
//
#if defined(_MT) || defined(__MT__)
#  define RPOS_LIB_THREAD_OPT "-mt"
#else
#  define RPOS_LIB_THREAD_OPT
#endif

#if defined(_MSC_VER) || defined(__MWERKS__)

#  ifdef _DLL

#     if (defined(__SGI_STL_PORT) || defined(_STLPORT_VERSION)) && (defined(_STLP_OWN_IOSTREAMS) || defined(__STL_OWN_IOSTREAMS))

#        if defined(_DEBUG) && (defined(__STL_DEBUG) || defined(_STLP_DEBUG))\
               && defined(BOOST_DEBUG_PYTHON) && defined(BOOST_LINKING_PYTHON)
#            define RPOS_LIB_RT_OPT "-gydp"
#        elif defined(_DEBUG) && (defined(__STL_DEBUG) || defined(_STLP_DEBUG))
#            define RPOS_LIB_RT_OPT "-gdp"
#        elif defined(_DEBUG)\
               && defined(BOOST_DEBUG_PYTHON) && defined(BOOST_LINKING_PYTHON)
#            define RPOS_LIB_RT_OPT "-gydp"
#            pragma message("warning: STLport debug versions are built with /D_STLP_DEBUG=1")
#            error "Build options aren't compatible with pre-built libraries"
#        elif defined(_DEBUG)
#            define RPOS_LIB_RT_OPT "-gdp"
#            pragma message("warning: STLport debug versions are built with /D_STLP_DEBUG=1")
#            error "Build options aren't compatible with pre-built libraries"
#        else
#            define RPOS_LIB_RT_OPT "-p"
#        endif

#     elif defined(__SGI_STL_PORT) || defined(_STLPORT_VERSION)

#        if defined(_DEBUG) && (defined(__STL_DEBUG) || defined(_STLP_DEBUG))\
               && defined(BOOST_DEBUG_PYTHON) && defined(BOOST_LINKING_PYTHON)
#            define RPOS_LIB_RT_OPT "-gydpn"
#        elif defined(_DEBUG) && (defined(__STL_DEBUG) || defined(_STLP_DEBUG))
#            define RPOS_LIB_RT_OPT "-gdpn"
#        elif defined(_DEBUG)\
               && defined(BOOST_DEBUG_PYTHON) && defined(BOOST_LINKING_PYTHON)
#            define RPOS_LIB_RT_OPT "-gydpn"
#            pragma message("warning: STLport debug versions are built with /D_STLP_DEBUG=1")
#            error "Build options aren't compatible with pre-built libraries"
#        elif defined(_DEBUG)
#            define RPOS_LIB_RT_OPT "-gdpn"
#            pragma message("warning: STLport debug versions are built with /D_STLP_DEBUG=1")
#            error "Build options aren't compatible with pre-built libraries"
#        else
#            define RPOS_LIB_RT_OPT "-pn"
#        endif

#     else

#        if defined(_DEBUG) && defined(BOOST_DEBUG_PYTHON) && defined(BOOST_LINKING_PYTHON)
#            define RPOS_LIB_RT_OPT "-gyd"
#        elif defined(_DEBUG)
#            define RPOS_LIB_RT_OPT "-gd"
#        else
#            define RPOS_LIB_RT_OPT
#        endif

#     endif

#  else

#     if (defined(__SGI_STL_PORT) || defined(_STLPORT_VERSION)) && (defined(_STLP_OWN_IOSTREAMS) || defined(__STL_OWN_IOSTREAMS))

#        if defined(_DEBUG) && (defined(__STL_DEBUG) || defined(_STLP_DEBUG))\
               && defined(BOOST_DEBUG_PYTHON) && defined(BOOST_LINKING_PYTHON)
#            define RPOS_LIB_RT_OPT "-sgydp"
#        elif defined(_DEBUG) && (defined(__STL_DEBUG) || defined(_STLP_DEBUG))
#            define RPOS_LIB_RT_OPT "-sgdp"
#        elif defined(_DEBUG)\
               && defined(BOOST_DEBUG_PYTHON) && defined(BOOST_LINKING_PYTHON)
#             define RPOS_LIB_RT_OPT "-sgydp"
#            pragma message("warning: STLport debug versions are built with /D_STLP_DEBUG=1")
#            error "Build options aren't compatible with pre-built libraries"
#        elif defined(_DEBUG)
#             define RPOS_LIB_RT_OPT "-sgdp"
#            pragma message("warning: STLport debug versions are built with /D_STLP_DEBUG=1")
#            error "Build options aren't compatible with pre-built libraries"
#        else
#            define RPOS_LIB_RT_OPT "-sp"
#        endif

#     elif defined(__SGI_STL_PORT) || defined(_STLPORT_VERSION)

#        if defined(_DEBUG) && (defined(__STL_DEBUG) || defined(_STLP_DEBUG))\
               && defined(BOOST_DEBUG_PYTHON) && defined(BOOST_LINKING_PYTHON)
#            define RPOS_LIB_RT_OPT "-sgydpn"
#        elif defined(_DEBUG) && (defined(__STL_DEBUG) || defined(_STLP_DEBUG))
#            define RPOS_LIB_RT_OPT "-sgdpn"
#        elif defined(_DEBUG)\
               && defined(BOOST_DEBUG_PYTHON) && defined(BOOST_LINKING_PYTHON)
#             define RPOS_LIB_RT_OPT "-sgydpn"
#            pragma message("warning: STLport debug versions are built with /D_STLP_DEBUG=1")
#            error "Build options aren't compatible with pre-built libraries"
#        elif defined(_DEBUG)
#             define RPOS_LIB_RT_OPT "-sgdpn"
#            pragma message("warning: STLport debug versions are built with /D_STLP_DEBUG=1")
#            error "Build options aren't compatible with pre-built libraries"
#        else
#            define RPOS_LIB_RT_OPT "-spn"
#        endif

#     else

#        if defined(_DEBUG)\
               && defined(BOOST_DEBUG_PYTHON) && defined(BOOST_LINKING_PYTHON)
#             define RPOS_LIB_RT_OPT "-sgyd"
#        elif defined(_DEBUG)
#             define RPOS_LIB_RT_OPT "-sgd"
#        else
#            define RPOS_LIB_RT_OPT "-s"
#        endif

#     endif

#  endif

#elif defined(__BORLANDC__)

//
// figure out whether we want the debug builds or not:
//
#if __BORLANDC__ > 0x561
#pragma defineonoption BOOST_BORLAND_DEBUG -v
#endif
//
// sanity check:
//
#if defined(__STL_DEBUG) || defined(_STLP_DEBUG)
#error "Pre-built versions of the Boost libraries are not provided in STLport-debug form"
#endif

#  ifdef _RTLDLL

#     if defined(BOOST_BORLAND_DEBUG)\
               && defined(BOOST_DEBUG_PYTHON) && defined(BOOST_LINKING_PYTHON)
#         define RPOS_LIB_RT_OPT "-yd"
#     elif defined(BOOST_BORLAND_DEBUG)
#         define RPOS_LIB_RT_OPT "-d"
#     elif defined(BOOST_DEBUG_PYTHON) && defined(BOOST_LINKING_PYTHON)
#         define RPOS_LIB_RT_OPT -y
#     else
#         define RPOS_LIB_RT_OPT
#     endif

#  else

#     if defined(BOOST_BORLAND_DEBUG)\
               && defined(BOOST_DEBUG_PYTHON) && defined(BOOST_LINKING_PYTHON)
#         define RPOS_LIB_RT_OPT "-syd"
#     elif defined(BOOST_BORLAND_DEBUG)
#         define RPOS_LIB_RT_OPT "-sd"
#     elif defined(BOOST_DEBUG_PYTHON) && defined(BOOST_LINKING_PYTHON)
#         define RPOS_LIB_RT_OPT "-sy"
#     else
#         define RPOS_LIB_RT_OPT "-s"
#     endif

#  endif

#endif

//
// select linkage opt:
//
#if (defined(_DLL) || defined(_RTLDLL)) && defined(BOOST_ALL_DYN_LINK)
#  define RPOS_LIB_PREFIX
#elif defined(BOOST_ALL_DYN_LINK)
#  error "Mixing a dll boost library with a static runtime is a really bad idea..."
#else
#  define RPOS_LIB_PREFIX "lib"
#endif

//
// now include the lib:
//
#if defined(RPOS_LIB_NAME) \
      && defined(RPOS_LIB_PREFIX) \
      && defined(RPOS_LIB_TOOLSET) \
      && defined(RPOS_LIB_THREAD_OPT) \
      && defined(RPOS_LIB_RT_OPT) \
      && defined(RPOS_VERSION_SHORT)

#ifdef BOOST_AUTO_LINK_TAGGED
#  pragma comment(lib, RPOS_LIB_PREFIX RPOS_STRINGIZE(RPOS_LIB_NAME) RPOS_LIB_THREAD_OPT RPOS_LIB_RT_OPT ".lib")
#  ifdef BOOST_LIB_DIAGNOSTIC
#     pragma message ("Linking to lib file: " RPOS_LIB_PREFIX RPOS_STRINGIZE(RPOS_LIB_NAME) RPOS_LIB_THREAD_OPT RPOS_LIB_RT_OPT ".lib")
#  endif
#elif defined(BOOST_AUTO_LINK_NOMANGLE)
#  pragma comment(lib, RPOS_STRINGIZE(RPOS_LIB_NAME) ".lib")
#  ifdef BOOST_LIB_DIAGNOSTIC
#     pragma message ("Linking to lib file: " RPOS_STRINGIZE(RPOS_LIB_NAME) ".lib")
#  endif
#elif defined(RPOS_AUTO_LINK_NO_VERSION)
#  pragma comment(lib, RPOS_LIB_PREFIX RPOS_STRINGIZE(RPOS_LIB_NAME) "-" RPOS_LIB_TOOLSET RPOS_LIB_THREAD_OPT RPOS_LIB_RT_OPT ".lib")
#  ifdef BOOST_LIB_DIAGNOSTIC
#     pragma message ("Linking to lib file: " RPOS_LIB_PREFIX RPOS_STRINGIZE(RPOS_LIB_NAME) "-" RPOS_LIB_TOOLSET RPOS_LIB_THREAD_OPT RPOS_LIB_RT_OPT ".lib")
#  endif
#else
#  pragma comment(lib, RPOS_LIB_PREFIX RPOS_STRINGIZE(RPOS_LIB_NAME) "-" RPOS_LIB_TOOLSET RPOS_LIB_THREAD_OPT RPOS_LIB_RT_OPT "-" RPOS_VERSION_SHORT ".lib")
#  ifdef BOOST_LIB_DIAGNOSTIC
#     pragma message ("Linking to lib file: " RPOS_LIB_PREFIX RPOS_STRINGIZE(RPOS_LIB_NAME) "-" RPOS_LIB_TOOLSET RPOS_LIB_THREAD_OPT RPOS_LIB_RT_OPT "-" RPOS_VERSION_SHORT ".lib")
#  endif
#endif

#else
#  error "some required macros where not defined (internal logic error)."
#endif


#endif // _MSC_VER || __BORLANDC__

//
// finally undef any macros we may have set:
//
#ifdef RPOS_LIB_PREFIX
#  undef RPOS_LIB_PREFIX
#endif
#if defined(RPOS_LIB_NAME)
#  undef RPOS_LIB_NAME
#endif
// Don't undef this one: it can be set by the user and should be the 
// same for all libraries:
//#if defined(BOOST_LIB_TOOLSET)
//#  undef BOOST_LIB_TOOLSET
//#endif
#if defined(RPOS_LIB_THREAD_OPT)
#  undef RPOS_LIB_THREAD_OPT
#endif
#if defined(RPOS_LIB_RT_OPT)
#  undef RPOS_LIB_RT_OPT
#endif
#if defined(RPOS_LIB_LINK_OPT)
#  undef RPOS_LIB_LINK_OPT
#endif
#if defined(RPOS_LIB_DEBUG_OPT)
#  undef RPOS_LIB_DEBUG_OPT
#endif
