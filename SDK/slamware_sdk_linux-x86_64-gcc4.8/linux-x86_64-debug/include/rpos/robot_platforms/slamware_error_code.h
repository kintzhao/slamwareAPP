/**
* slamware_error_code.hpp
* Error codes for Slamware
*
* Created By Tony Huang @ 2015-03-27
* Copyright (c) 2015 Shanghai SlamTec Co., Ltd.
*/

#pragma once

#include <boost/system/error_code.hpp>

namespace rpos { namespace robot_platforms { namespace slamware {

    namespace errc {

        enum errc_t {
            success = 0,
            timeout = 1,
            already_connected = 2,
            connection_lost = 3,
            unauthorized_request = 4,
            request_timeout = 5,
            path_find_fail = 6,
            unsupported_command = 7,
            connection_fail = 8,
            parse_invalid = 9,
            // slamware core platform will not use the following http exceptions.
            http_status_code_bad_request = 400,
            http_status_code_unauthorized = 401,
            http_status_code_forbidden = 403,
            http_status_code_not_found = 404,
            http_status_code_internal_server_error = 500,
            http_status_code_gateway_timeout = 504
        };

    }

    const boost::system::error_category& error_category() BOOST_SYSTEM_NOEXCEPT;

} } }

namespace boost { namespace system {

    template<>
    struct is_error_condition_enum < rpos::robot_platforms::slamware::errc::errc_t >
    {
        static const bool value = true;
    };

} }
