/**
* device_health.h
* 
*
* Created   @ 2016-7-4
* Copyright (c) 2014 Shanghai SlamTec Co., Ltd.
*/

#pragma once

#include <rpos/rpos_config.h>
#include <rpos/core/rpos_core_config.h>
#include <string>
#include <vector>
#include <cstdint>

namespace rpos { namespace features { namespace system_resource {
    enum BaseErrorLevel
    {
        BaseErrorLevelWarn,
        BaseErrorLevelError,
        BaseErrorLevelFatal,
        BaseErrorLevelUnknown = 255
    };

    enum BaseErrorComponent
    {
        BaseErrorComponentUser,
        BaseErrorComponentSystem,
        BaseErrorComponentPower,
        BaseErrorComponentMotion,
        BaseErrorComponentSensor,
        BaseErrorComponentUnknown = 255
    };

    struct BaseError
    {
        BaseError()
            : id (0), errorCode(0), level(BaseErrorLevelWarn)
            , component(BaseErrorComponentSystem), componentErrorCode(0)
        {
        }

        int id;
        std::uint32_t errorCode;
        BaseErrorLevel level;
        BaseErrorComponent component;
        std::uint16_t componentErrorCode;
        std::string message;
    };

    struct BaseHealthInfo
    {
        BaseHealthInfo()
            : hasWarning(false), hasError(false), hasFatal(false)
        {}

        bool hasWarning;
        bool hasError;
        bool hasFatal;

        std::vector<BaseError> errors;
    };
} } }
