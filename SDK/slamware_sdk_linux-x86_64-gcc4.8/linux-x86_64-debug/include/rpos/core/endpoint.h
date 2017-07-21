/**
* endpoint.h
* Endpoint
*
* Created By Tony Huang @ 2014-5-27
* Copyright (c) 2014 Shanghai SlamTec Co., Ltd.
*/

#pragma once

#include <rpos/rpos_config.h>
#include <rpos/system/object_handle.h>

namespace rpos {
	namespace core {

		namespace detail {
			class EndpointImpl;
		}

		class RPOS_CORE_API Endpoint : public rpos::system::ObjectHandle<Endpoint, detail::EndpointImpl> {
		public:
			RPOS_OBJECT_CTORS(Endpoint);
			~Endpoint();

		public:

		};

	}
}