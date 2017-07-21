/*
 * rpos_config.h
 * Configuration of rpos build
 *
 * Created by Tony Huang (cnwzhjs@gmail.com) at 2012-09-03
 * Copyright 2012 (c) www.robopeak.com
 */

#pragma once

// Check is compiler support right-value-ref
#include <boost/config.hpp>
#include <rpos/rpos_version.h>

#ifdef BOOST_HAS_RVALUE_REFS
#	define RPOS_HAS_RVALUE_REFS
#endif

#define RPOS_SINGLE_THREAD
#define RPOS_MULTI_THREAD

#ifdef WIN32

#define RPOS_MODULE_EXPORT  __declspec(dllexport)
#define RPOS_MODULE_IMPORT  __declspec(dllimport)
#include <boost/thread/win32/mfc_thread_init.hpp>

#else

#define RPOS_MODULE_EXPORT
#define RPOS_MODULE_IMPORT

#endif
