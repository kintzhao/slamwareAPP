#pragma once
#include <vector>
#include <rpos/system/types.h>
#include <rpos/core/rpos_core_config.h>
#include "rpos/system/serialization/i_stream.h"

namespace rpos { namespace system { namespace serialization {

    RPOS_CORE_API class BufferStreamAdaptor : public IStream
    {
    public:
        RPOS_CORE_API BufferStreamAdaptor(std::vector<system::types::_u8> *buf);

        RPOS_CORE_API virtual ~BufferStreamAdaptor();

        virtual size_t read(void *buffer, size_t count);

        virtual size_t write(const void *buffer, size_t count);

        virtual bool empty();

    private:
        std::vector<system::types::_u8> *buf_;
        size_t head_;
    };

} } }