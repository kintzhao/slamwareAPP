/*
* rpos_core_config.h
* Configuration of rpos core module build
*
* Created by Tony Huang (cnwzhjs@gmail.com) at 2012-09-03
* Copyright 2012 (c) www.robopeak.com
*/

#pragma once

#include <rpos/rpos_config.h>

#ifdef RPOS_CORE_DLL
#	ifdef RPOS_CORE_EXPORT
#		define RPOS_CORE_API RPOS_MODULE_EXPORT
#	else
#		define RPOS_CORE_API RPOS_MODULE_IMPORT
#	endif
#else
#	define RPOS_CORE_API
#endif


#if !defined(RPOS_CORE_EXPORT) && !defined(RPOS_CORE_STATIC)
#define RPOS_LIB_NAME rpos_core
#include <rpos/system/util/auto_link.h>
#undef RPOS_LIB_NAME
#endif
