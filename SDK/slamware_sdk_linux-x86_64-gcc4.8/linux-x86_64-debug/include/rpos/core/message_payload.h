/*
* message_payload.h
* Message Payload
*
* Created By Tony Huang (cnwzhjs@gmail.com) at 2014-5-22
* Copyright 2014 (c) www.robopeak.com
*/

#pragma once

#include <rpos/rpos_config.h>
#include <rpos/core/rpos_core_config.h>
#include <rpos/system/object_handle.h>

namespace rpos {
	namespace net {

		namespace detail {
			class MessagePayloadImpl;
		}

		class RPOS_CORE_API MessagePayload : public rpos::system::ObjectHandle<MessagePayload, detail::MessagePayloadImpl>{
		public:
			RPOS_OBJECT_CTORS(MessagePayload);
			~MessagePayload();

		public:
			// TODO some methods
		};

	}
}