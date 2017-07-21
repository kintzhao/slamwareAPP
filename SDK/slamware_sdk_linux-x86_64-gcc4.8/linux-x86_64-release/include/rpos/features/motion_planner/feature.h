/**
* feature.h
* The Motion Planner feature
*
* Created By Tony Huang @ 2014-5-22
* Copyright (c) 2014 Shanghai SlamTec Co., Ltd.
*/

#pragma once

#include <rpos/rpos_config.h>
#include <rpos/core/feature.h>
#include <rpos/core/pose.h>
#include "move_action.h"

namespace rpos {
	namespace features {

		namespace detail {
			class MotionPlannerImpl;
		}

		class RPOS_CORE_API MotionPlanner : public rpos::core::Feature{
		public:
			typedef detail::MotionPlannerImpl impl_t;

			RPOS_OBJECT_CTORS_WITH_BASE(MotionPlanner, rpos::core::Feature);
			MotionPlanner(boost::shared_ptr<detail::MotionPlannerImpl> impl);
			~MotionPlanner();

		public:
			rpos::actions::MoveAction moveTo(const std::vector<rpos::core::Location>& locations, bool appending, bool isMilestone);
			rpos::actions::MoveAction moveTo(const rpos::core::Location& location, bool appending, bool isMilestone);
            rpos::actions::MoveAction moveBy(const rpos::core::Direction& direction);
			rpos::actions::MoveAction rotateTo(const rpos::core::Rotation& orientation);
            rpos::actions::MoveAction rotate(const rpos::core::Rotation& rotation);
			rpos::actions::MoveAction getCurrentAction();

			rpos::features::motion_planner::Path searchPath(const rpos::core::Location& location);
		};

	}
}
