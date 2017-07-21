/**
* device.h
* Device
*
* Created By Tony Huang @ 2014-5-22
* Copyright (c) 2014 Shanghai SlamTec Co., Ltd.
*/

#pragma once

#include <rpos/rpos_config.h>
#include <rpos/core/rpos_core_config.h>
#include <rpos/system/object_handle.h>

namespace rpos {
	namespace core {

		namespace detail {
			class DeviceImpl;
		}

		class RPOS_CORE_API Device : public rpos::system::ObjectHandle<Device, detail::DeviceImpl>{
		public:
			RPOS_OBJECT_CTORS(Device);
			~Device();

		public:
			std::string getSerialNumber();
			std::string getName();
		};
	}
}