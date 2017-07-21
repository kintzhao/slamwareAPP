/**
* path.h
* Path represents a route from somewhere to somewhere
*
* Created By Tony Huang @ 2014-5-22
* Copyright (c) 2014 Shanghai SlamTec Co., Ltd.
*/

#pragma once

#include <rpos/rpos_config.h>
#include <rpos/system/object_handle.h>
#include <rpos/core/pose.h>
#include <vector>

namespace rpos {
	namespace features {
		namespace motion_planner {

			namespace detail {
				class PathImpl;
			}

			class RPOS_CORE_API Path : public rpos::system::ObjectHandle<Path, detail::PathImpl>{
			public:
				RPOS_OBJECT_CTORS(Path);
				Path(const std::vector<core::Location>& points);

#ifdef RPOS_HAS_RVALUE_REFS
				Path(std::vector<core::Location>&& points);
#endif

				~Path();

			public:
				const std::vector<core::Location>& getPoints() const;
			};

		}
	}
}
