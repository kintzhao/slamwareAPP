/**
* sweep_move_action.h
* Sweep Move Action
*
* Created By Tony Huang @ 2015-1-8
* Copyright (c) 2015 Shanghai SlamTec Co., Ltd.
*/

#pragma once

#include "../motion_planner/move_action.h"
#include "../location_provider/map.h"

namespace rpos { namespace actions {

    namespace detail {
        class SweepMoveActionImpl;
    }

    class RPOS_CORE_API SweepMoveAction : public MoveAction {
    public:
        typedef detail::SweepMoveActionImpl impl_t;

        RPOS_OBJECT_CTORS_WITH_BASE(SweepMoveAction, MoveAction);
        SweepMoveAction(boost::shared_ptr<impl_t> impl);
        ~SweepMoveAction();

    public:
        std::vector<features::location_provider::MapType> getAvailableSweepMaps();
        features::location_provider::Map getSweepMap(features::location_provider::MapType type, core::RectangleF area);
        core::RectangleF getSweepMapArea(features::location_provider::MapType type);
    };

} }

namespace rpos { namespace core { namespace detail {

    template <>
    struct RPOS_CORE_API action_caster < actions::SweepMoveAction >
    {
        static actions::SweepMoveAction cast(Action&);
    };

} } }
