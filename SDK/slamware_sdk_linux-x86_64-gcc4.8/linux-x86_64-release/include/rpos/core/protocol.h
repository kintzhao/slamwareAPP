/**
* protocol.h
* Protocol is used to encode and decode messages
*
* Created By Tony Huang @ 2014-4-28
* Copyright (c) 2014 Shanghai SlamTec Co., Ltd.
*/

#pragma once

#include <rpos/rpos_config.h>
#include <rpos/core/rpos_core_config.h>
#include <rpos/system/types.h>
#include <rpos/system/object_handle.h>
#include <rpos/system/signal.h>
#include <boost/shared_ptr.hpp>
#include <boost/noncopyable.hpp>

namespace rpos {
	namespace mm {

		class Data;

	}

	namespace net {

		namespace detail {
			class ProtocolImpl;
		}

		class Message;

		class RPOS_CORE_API Protocol : public rpos::system::ObjectHandle<Protocol, detail::ProtocolImpl>{
		public:
			RPOS_OBJECT_CTORS(Protocol);
			~Protocol();

		public:
			RPOS_SINGLE_THREAD const Message& createSyncMessage();
			RPOS_SINGLE_THREAD void resetDecoder();
			RPOS_SINGLE_THREAD void decodeData(const system::types::_u8* data, size_t offset, size_t size);
			RPOS_SINGLE_THREAD void decode(system::types::_u8 data);

			RPOS_MULTI_THREAD size_t estimateLength(const Message& message);
			RPOS_MULTI_THREAD boost::shared_ptr<mm::Data> encodeData(const Message& message);

		public:
			RPOS_SINGLE_THREAD rpos::system::Signal<void(const Message&)>& signalMessageArrived();
			RPOS_SINGLE_THREAD rpos::system::Signal<void(void)>& signalMessageCorrupted();
		};

	}
}
