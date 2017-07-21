/**
* channel.h
* Channel class is an asynchronized communication tunnel transmits messages
*
* Created By Tony Huang @ 2014-4-28
* Copyright (c) 2014 Shanghai SlamTec Co., Ltd.
*/

#pragma once

#include <rpos/rpos_config.h>
#include <rpos/core/rpos_core_config.h>
#include <rpos/system/types.h>
#include <rpos/system/signal.h>
#include <rpos/system/object_handle.h>
#include <boost/shared_ptr.hpp>
#include <boost/functional.hpp>

namespace rpos {
	namespace core {

		enum ChannelStatus {
			ChannelStatusIdle,
			ChannelStatusReady,
			ChannelStatusError,
			ChannelStatusClosed
		};

		namespace detail {
			class ChannelImpl;
		}

		class Message;

		class RPOS_CORE_API Channel : public rpos::system::ObjectHandle<Channel, detail::ChannelImpl>{
		public:
			RPOS_OBJECT_CTORS(Channel);
			~Channel();

		public:
			RPOS_MULTI_THREAD ChannelStatus getStatus();

		public:
			RPOS_MULTI_THREAD void sendMessage(const Message& msg);
			RPOS_MULTI_THREAD void sendMessage(const Message& msg, boost::function<void(const rpos::system::types::error_code&)> onFinished);

		public:
			RPOS_MULTI_THREAD rpos::system::Signal<void(const Message&)>& signalMessageArrived();
			RPOS_MULTI_THREAD rpos::system::Signal<void(const rpos::system::types::error_code&)>& signalMessageCorrupted();
			RPOS_MULTI_THREAD rpos::system::Signal<void(const rpos::system::types::error_code&, const Message&)>& signalSendFailure();
			RPOS_MULTI_THREAD rpos::system::Signal<void(const Message&)>& signalSendTimeout();
			RPOS_MULTI_THREAD rpos::system::Signal<void()>& signalConnectionLost();
		};

	}
}