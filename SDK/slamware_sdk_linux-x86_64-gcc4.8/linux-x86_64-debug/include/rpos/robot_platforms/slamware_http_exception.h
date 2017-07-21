/*
* slamware_http_exception.h
* Exceptions shared by all Slamware https client
*
* Originally created by Gabrial He (ykhe@slamtec.com) at 2016-05-09
*
* Copyright (c) 2016~2016 Shanghai SlamTec Co., Ltd.
*/

#pragma once

#include <rpos/robot_platforms/slamware_common_exception.h>


namespace rpos { namespace robot_platforms { namespace http {

    class HttpException : public rpos::system::detail::ExceptionBase
    {
    public:
        RPOS_CORE_API HttpException(const int statusCode, const std::string &msg) throw()
            : statusCode_(statusCode), rpos::system::detail::ExceptionBase(msg)
        {}

        virtual ~HttpException() throw()
        {}

        RPOS_CORE_API const int getStatus()
        {
            return statusCode_;
        }

    protected:
        int statusCode_;
    };

    class RequestHttpException : public HttpException
    {
    public:
        RequestHttpException(const int statusCode, const std::string &msg) throw()
            : HttpException(statusCode, msg)
        {}

        virtual ~RequestHttpException() throw()
        {}
    };

    class ServerHttpException : public HttpException
    {
    public:
        ServerHttpException(const int statusCode, const std::string &msg) throw()
            : HttpException(statusCode, msg)
        {}

        virtual ~ServerHttpException() throw()
        {}
    };

} } }