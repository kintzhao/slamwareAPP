/**
* move_action.h
* Move action
*
* Created By Tony Huang @ 2014-5-22
* Copyright (c) 2014 Shanghai SlamTec Co., Ltd.
*/

#pragma once

#include <rpos/rpos_config.h>
#include <rpos/core/action.h>
#include "path.h"

namespace rpos {
	namespace actions {

		namespace detail {
			class MoveActionImpl;
		}

		class RPOS_CORE_API MoveAction : public rpos::core::Action{
		public:
			typedef detail::MoveActionImpl impl_t;

			RPOS_OBJECT_CTORS_WITH_BASE(MoveAction, rpos::core::Action);
			MoveAction(boost::shared_ptr<impl_t> impl);
			~MoveAction();

		public:
			rpos::features::motion_planner::Path getRemainingPath();
			rpos::features::motion_planner::Path getRemainingMilestones();
		};
	}
}

namespace rpos { namespace core { namespace detail {

    template <>
    struct action_caster < actions::MoveAction >
    {
        static actions::MoveAction cast(Action&);
    };

} } }
