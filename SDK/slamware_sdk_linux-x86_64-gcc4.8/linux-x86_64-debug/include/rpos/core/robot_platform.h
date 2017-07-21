/**
* robot_platform.h
* Robot Platform
*
* Created By Tony Huang @ 2014-5-27
* Copyright (c) 2014 Shanghai SlamTec Co., Ltd.
*/

#pragma once

#include <rpos/rpos_config.h>
#include <rpos/system/object_handle.h>
#include <rpos/core/feature.h>
#include <boost/shared_ptr.hpp>
#include <vector>

namespace rpos {
	namespace core {

		class RobotPlatform;

		namespace detail {

			class RobotPlatformImpl;
			
			template<class RobotPlatformT>
			struct robot_platform_caster {
				static RobotPlatformT cast(RobotPlatform&);
			};

		};

		class RPOS_CORE_API RobotPlatform : public rpos::system::ObjectHandle<RobotPlatform, detail::RobotPlatformImpl> {
		public:
			RPOS_OBJECT_CTORS(RobotPlatform);
			~RobotPlatform();

		public:
			std::vector<Feature> getFeatures();

			template<class RobotPlatformT>
			RobotPlatformT cast() {
				return detail::robot_platform_caster<RobotPlatformT>::cast(*this);
			}

		private:
			template<class RobotPlatformT>
			friend struct detail::robot_platform_caster;
		};
	}
}