/*
* rpos_types.h
* Configuration of rpos build
*
* Created by Tony Huang (cnwzhjs@gmail.com) at 2014-04-28
* Copyright 2012 (c) www.robopeak.com
*/

#pragma once 

#include <boost/system/error_code.hpp>
#include <boost/chrono.hpp>
#include <float.h>
#include <math.h>

//Basic types
//
#ifdef WIN32

//fake stdint.h for VC only

typedef signed   char     int8_t;
typedef unsigned char     uint8_t;

typedef __int16           int16_t;
typedef unsigned __int16  uint16_t;

typedef __int32           int32_t;
typedef unsigned __int32  uint32_t;

typedef __int64           int64_t;
typedef unsigned __int64  uint64_t;

#else

#include <stdint.h>

#define RPMODULE_EXPORT
#define RPMODULE_IMPORT

#endif

namespace rpos { namespace system { namespace types {

	typedef int8_t         _s8;
	typedef uint8_t        _u8;

	typedef int16_t        _s16;
	typedef uint16_t       _u16;

	typedef int32_t        _s32;
	typedef uint32_t       _u32;

	typedef int64_t        _s64;
	typedef uint64_t       _u64;

	// The _word_size_t uses actual data bus width of the current CPU
#ifdef _AVR_
	typedef _u8            _word_size_t;
#define THREAD_PROC    
#elif defined (WIN64)
	typedef _u64           _word_size_t;
#define THREAD_PROC    __stdcall
#elif defined (WIN32)
	typedef _u32           _word_size_t;
#define THREAD_PROC    __stdcall
#elif defined (__GNUC__)
	typedef unsigned long  _word_size_t;
#define THREAD_PROC   
#elif defined (__ICCARM__)
	typedef _u32            _word_size_t;
#define THREAD_PROC  
#endif

	// TODO remove boost::system and boost::chrono dependency
	typedef int32_t error_code;

	typedef uint64_t timestamp_t;


    static inline bool fequal(float a, float b) {
        if (fabs(a-b) <FLT_EPSILON) {
            return true;
        } else {
            return false;
        }
    }

    static inline bool fequal(double a, double b) {
        if (fabs(a-b) <FLT_EPSILON) {
            return true;
        } else {
            return false;
        }
    }

	template < class T >
	struct NumberTypeMeta;

#define RPOS_SYSTEM_TYPES_DEFINE_NUMBER_META(NumT, MoreAdvancedT, MostAdvancedT, UnsignedT, SignedT, EqCondition) \
	template < > \
	struct NumberTypeMeta<NumT> { \
		typedef MoreAdvancedT more_advanced_t; \
		typedef MostAdvancedT most_advanced_t; \
		typedef UnsignedT unsigned_t; \
		typedef SignedT signed_t; \
		static bool is_equal(NumT a, NumT b) { \
			return (EqCondition); \
		} \
	};

	RPOS_SYSTEM_TYPES_DEFINE_NUMBER_META(int8_t, int16_t, int64_t, uint8_t, int8_t, a == b);
	RPOS_SYSTEM_TYPES_DEFINE_NUMBER_META(int16_t, int32_t, int64_t, uint16_t, int16_t, a == b);
	RPOS_SYSTEM_TYPES_DEFINE_NUMBER_META(int32_t, int64_t, int64_t, uint32_t, int32_t, a == b);
	RPOS_SYSTEM_TYPES_DEFINE_NUMBER_META(int64_t, int64_t, int64_t, uint64_t, int64_t, a == b);
	RPOS_SYSTEM_TYPES_DEFINE_NUMBER_META(uint8_t, uint16_t, uint64_t, uint8_t, int8_t, a == b);
	RPOS_SYSTEM_TYPES_DEFINE_NUMBER_META(uint16_t, uint32_t, uint64_t, uint16_t, int16_t, a == b);
	RPOS_SYSTEM_TYPES_DEFINE_NUMBER_META(uint32_t, uint64_t, uint64_t, uint32_t, int32_t, a == b);
	RPOS_SYSTEM_TYPES_DEFINE_NUMBER_META(uint64_t, uint64_t, uint64_t, uint64_t, int64_t, a == b);
	RPOS_SYSTEM_TYPES_DEFINE_NUMBER_META(float, double, double, float, float, fequal(a, b));
	RPOS_SYSTEM_TYPES_DEFINE_NUMBER_META(double, double, double, double, double, fequal(a, b));

} } }
