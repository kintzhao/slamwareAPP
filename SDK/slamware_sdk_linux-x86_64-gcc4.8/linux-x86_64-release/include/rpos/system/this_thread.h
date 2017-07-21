/*
* this_thread.h
* Some miscellaneous functions to deal with current thread
*
* Created by Tony Huang (tony@slamtec.com)
* Copyright 2016 (c) Shanghai Slamtec Co., Ltd.
*/

#include <rpos/core/rpos_core_config.h>
#include <string>

namespace rpos { namespace system { namespace this_thread {

    /*
    * @brief Set the thread name will ease the debugging process of cpu usage
    * Notice: this function only effect on non-win32 platforms, cos it depends on pthread_setname_np & pthread_self
    */
	RPOS_CORE_API void setCurrentThreadName(const std::string& newName);

} } }

