/**
* action.h
* Action is a robot operation
*
* Created By Tony Huang @ 2014-5-22
* Copyright (c) 2014 Shanghai SlamTec Co., Ltd.
*/

#pragma once

#include <rpos/rpos_config.h>
#include <rpos/system/object_handle.h>
#include <rpos/system/types.h>

namespace rpos {
	namespace core {

		typedef rpos::system::types::_u32 ActionID;

		enum ActionStatus {
			ActionStatusWaitingForStart,
			ActionStatusRunning,
			ActionStatusFinished,
			ActionStatusPaused,
			ActionStatusStopped,
			ActionStatusError
		};

		class Action;

		namespace detail {
			class ActionImpl;

			template<class ActionT>
			struct action_caster {
				static ActionT cast(Action&);
			};
		}

		class RPOS_CORE_API Action : public rpos::system::ObjectHandle<Action, detail::ActionImpl> {
		public:
			RPOS_OBJECT_CTORS(Action);
			~Action();

		public:
			ActionStatus getStatus();
			double getProgress();

			void cancel();
			ActionStatus waitUntilDone();

            bool isEmpty();
            rpos::core::ActionID getActionId();
            std::string getActionName();

			template<class ActionT>
			ActionT cast() {
				return detail::action_caster<ActionT>::cast(*this);
			}

		private:
			template<class ActionT>
			friend struct detail::action_caster;
		};

	}
}
