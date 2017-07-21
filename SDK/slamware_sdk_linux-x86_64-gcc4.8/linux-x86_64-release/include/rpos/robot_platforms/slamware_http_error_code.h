#pragma once

#include <boost/system/error_code.hpp>

namespace rpos { namespace robot_platforms { namespace http {

    namespace errc {

        enum http_errc_t {
            success = 0,
            timeout,
            parse_error,
            connection_fail,
            request_error,
            server_error
        };

    }

    const boost::system::error_category& http_error_category() BOOST_SYSTEM_NOEXCEPT;

} } }

namespace boost { namespace system {

    template<>
    struct is_error_condition_enum < rpos::robot_platforms::http::errc::http_errc_t >
    {
        static const bool value = true;
    };

} }
