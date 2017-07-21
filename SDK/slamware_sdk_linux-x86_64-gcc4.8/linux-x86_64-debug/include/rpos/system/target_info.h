/*
* target_info.h
* This header defines some marcos for detecting target configuration
* 
* Created By Tony Huang @ 2015-09-17
* Copyright Shanghai Slamtec Co., Ltd.
*/

#pragma once

#if __LP64__ || _LP64 || defined(_WIN64)
#   define RPOS_TARGET_64BIT
#else
#   define RPOS_TARGET_32BIT
#endif

#if defined(_WIN32) || defined(_WIN64)
#   define RPOS_TARGET_WINDOWS
#elif defined(_AIX)
#   define RPOS_TARGET_AIX
#elif defined(__hpux)
#   define RPOS_TARGET_HPUX
#elif defined(ANDROID)
#   define RPOS_TARGET_ANDROID
#elif defined(__linux__)
#   define RPOS_TARGET_LINUX
#elif defined(__APPLE__) && defined(__MACH__)
#   define RPOS_TARGET_DARWIN
#   include <TargetConditionals.h>
#   if TARGET_IPHONE_SIMULATOR == 1
#       define RPOS_TARGET_IOS
#       define RPOS_TARGET_IOS_SIMULATOR
#   elif TARGET_OS_IPHONE == 1
#       define RPOS_TARGET_IOS
#   elif TARGET_OS_MAC == 1
#       define RPOS_TARGET_OSX
#   endif
#elif defined(__unix__)
#   define RPOS_TARGET_BSD
#elif defined(__sun) && defined(__SVR4)
#   define RPOS_TARGET_SOLARIS
#elif defined(__CYGWIN__)
#   define RPOS_TARGET_CYGWIN
#else
#   define RPOS_TARGET_UNKNOWN
#endif

#if __BIG_ENDIAN__
#   define RPOS_TARGET_BE
#else
#   define RPOS_TARGET_LE
#endif
