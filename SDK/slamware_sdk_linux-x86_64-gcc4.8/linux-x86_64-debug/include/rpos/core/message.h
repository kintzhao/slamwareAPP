/*
* message.h
* Message
*
* Created by Tony Huang (cnwzhjs@gmail.com) at 2014-04-28
* Copyright 2014 (c) www.robopeak.com
*/

#pragma once

#include <rpos/rpos_config.h>
#include <rpos/core/rpos_core_config.h>
#include <rpos/system/types.h>
#include <rpos/system/object_handle.h>
#include <boost/shared_ptr.hpp>

#include "message_payload.h"

namespace rpos {
	namespace net {

		namespace detail {
			class MessageImpl;
		}

		enum MessageFlag : system::types::_u8 {
			MessageFlagContd		   = 0x1 << 0,
			MessageFlagError		   = 0x1 << 1,
			MessageFlagCustom		   = 0x1 << 2,
			MessageFlagTest			   = 0x1 << 3,
			MessageFlagCheckSumEnabled = 0x1 << 4,
			MessageFlagAddressEnabled  = 0x1 << 5,
			MessageFlagLongFrame	   = 0x1 << 6,
			MessageFlagExtraFlag	   = 0x1 << 7
		};

		enum MessageExtraFlag : system::types::_u8 {
			MessageExtraFlagUser	  = 0x1 << 0,
			MessageExtraFlagExtraFlag = 0x1 << 7
		};

		typedef system::types::_u8 MessageAddress;

		class RPOS_CORE_API Message : public rpos::system::ObjectHandle<Message, detail::MessageImpl>{
		public:
			RPOS_OBJECT_CTORS(Message);
			~Message();

		public:
			system::types::_u32 getSource() const;
			void setSource(system::types::_u32 source);

			system::types::_u32 getDest() const;
			void setDest(system::types::_u32 dest);

			MessageFlag getFlags() const;
			void setFlags(MessageFlag flags);

			MessageExtraFlag getExtraFlags() const;
			void setExtraFlags(MessageExtraFlag extraFlags);

			size_t getLength() const;
			void setLength(size_t length);

			system::types::_u8 getCommand() const;
			void setCommand(system::types::_u8 command);

			system::types::_u8 getChecksum() const;
			void setChecksum(system::types::_u8 checksum);

			boost::shared_ptr<MessagePayload> getPayload();
			void setPayload(boost::shared_ptr<MessagePayload> payload);
		};

	}
}