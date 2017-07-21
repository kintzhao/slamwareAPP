#pragma once

#ifndef RPOS_VOID

#ifdef _MSC_VER
#	define RPOS_VOID void
#else
namespace rpos { namespace system { namespace util {
	struct Void {};
}}}
#	define RPOS_VOID ::rpos::system::util::Void
#endif

#endif

