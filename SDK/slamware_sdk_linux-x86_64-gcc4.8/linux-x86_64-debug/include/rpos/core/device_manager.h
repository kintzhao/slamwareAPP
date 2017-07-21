/**
* device_manager.h
* Device Manager
*
* Created By Tony Huang @ 2014-5-27
* Copyright (c) 2014 Shanghai SlamTec Co., Ltd.
*/

#pragma once

#include <rpos/rpos_config.h>
#include <rpos/core/rpos_core_config.h>
#include <rpos/system/object_handle.h>

namespace rpos {
	namespace core {

		namespace detail {
			class DeviceManagerImpl;
		}

		class RPOS_CORE_API DeviceManager : public rpos::system::ObjectHandle<DeviceManager, detail::DeviceManagerImpl> {
		public:
			RPOS_OBJECT_CTORS(DeviceManager);
			~DeviceManager();

		public:

		};

	}
}