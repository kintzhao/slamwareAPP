/**
* feature.h
* The Sweep Motion Planner feature
*
* Created By Tony Huang @ 2015-4-3
* Copyright (c) 2015 Shanghai Slamtec Co., Ltd.
*/

#pragma once

#include <rpos/rpos_config.h>
#include <rpos/features/motion_planner/feature.h>
#include <rpos/features/sweep_motion_planner/sweep_move_action.h>

namespace rpos { namespace features {

    namespace detail {
        class SweepMotionPlannerImpl;
    }

    class RPOS_CORE_API SweepMotionPlanner : public rpos::features::MotionPlanner {
    public:
        typedef detail::SweepMotionPlannerImpl impl_t;

        RPOS_OBJECT_CTORS_WITH_BASE(SweepMotionPlanner, rpos::features::MotionPlanner);
        SweepMotionPlanner(boost::shared_ptr<impl_t> impl);
        ~SweepMotionPlanner();

    public:
        rpos::actions::SweepMoveAction startSweep();
        rpos::actions::SweepMoveAction sweepSpot(const rpos::core::Location& location);
        rpos::actions::MoveAction goHome();
    };

} }

namespace rpos { namespace core { namespace detail {

    template <>
    struct RPOS_CORE_API feature_caster < features::SweepMotionPlanner >
    {
        static features::SweepMotionPlanner cast(Feature& v);
    };

} } }
