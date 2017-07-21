/*
* slamware_common_exception.h
* Exceptions shared by all Slamware robot platforms
*
* Modified by Tony Huang at 2015-03-26
* Originally created by Jacky Li (eveningwear@gmail.com) at 2014-08-22
*
* Copyright (c) 2014~2015 Shanghai SlamTec Co., Ltd.
*/

#pragma once

#include <rpos/rpos_config.h>
#include <rpos/system/exception.h>
#include <exception>
#include <string>

#define RPOS_SLAMWARE_DEFINE_EXCEPTION(Name) \
    class Name ## Exception : public rpos::system::detail::ExceptionBase { \
    public: \
        RP_DEFINE_EXCEPTION(Name ## Exception, rpos::system::detail::ExceptionBase); \
    };

namespace rpos { namespace robot_platforms {

    RPOS_SLAMWARE_DEFINE_EXCEPTION(ConnectionTimeOut);      // timeout              : 1
    RPOS_SLAMWARE_DEFINE_EXCEPTION(AlreadyConnected);       // already_connected    : 2
    RPOS_SLAMWARE_DEFINE_EXCEPTION(ConnectionLost);         // connection_lost      : 3
    RPOS_SLAMWARE_DEFINE_EXCEPTION(UnauthorizedRequest);    // unauthorized_request : 4
    RPOS_SLAMWARE_DEFINE_EXCEPTION(RequestTimeOut);         // request_timeout      : 5
    RPOS_SLAMWARE_DEFINE_EXCEPTION(PathFindFail);           // path_find_fail       : 6
    RPOS_SLAMWARE_DEFINE_EXCEPTION(UnsupportedCommand);     // unsupported_command  : 7
    RPOS_SLAMWARE_DEFINE_EXCEPTION(ConnectionFail);         // default              : 8
    RPOS_SLAMWARE_DEFINE_EXCEPTION(ParseInvalid);           // parse invalid        : 9
    RPOS_SLAMWARE_DEFINE_EXCEPTION(HttpBadRequest);         // http bad request     : 400
    RPOS_SLAMWARE_DEFINE_EXCEPTION(HttpUnauthorized);       // http unauthorized    : 401
    RPOS_SLAMWARE_DEFINE_EXCEPTION(HttpForbidden);          // http forbidden       : 403
    RPOS_SLAMWARE_DEFINE_EXCEPTION(HttpNotFound);           // http not found       : 404
    RPOS_SLAMWARE_DEFINE_EXCEPTION(HttpInternalServerError);// http internal server error : 500
    RPOS_SLAMWARE_DEFINE_EXCEPTION(HttpGatewayTimeout);     // http gateway timeout : 504

} }
