#pragma once

#ifdef RPOS_SLAMWARE_DLL
#   ifdef RPOS_SLAMWARE_EXPORT
#       define RPOS_SLAMWARE_API RPOS_MODULE_EXPORT
#   else
#       define RPOS_SLAMWARE_API RPOS_MODULE_IMPORT
#   endif
#else
#   define RPOS_SLAMWARE_API
#endif

#if !defined(RPOS_SLAMWARE_EXPORT) && !defined(RPOS_SLAMWARE_STATIC)
#define RPOS_LIB_NAME rpos_robotplatforms_rpslamware
#include <rpos/system/util/auto_link.h>
#undef RPOS_LIB_NAME
#endif

#define RPOS_SLAMWARE_SDK_SUPPORT_SWEEP
#define RPOS_SLAMWARE_SDK_SUPPORT_GO_HOME
