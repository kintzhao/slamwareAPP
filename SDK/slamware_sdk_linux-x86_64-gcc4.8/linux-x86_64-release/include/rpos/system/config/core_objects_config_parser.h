#pragma once

#include "config_parser.h"
#include <rpos/core/pose.h>
#include <rpos/core/geometry.h>
#include <rpos/system/util/log.h>

namespace rpos { namespace system { namespace config {

    template <>
    struct ConfigParser < core::Pose > {
        static bool parse(const Json::Value& config, core::Pose& that);
    };

    template <>
    struct ConfigParser < core::RectangleF > {
        static bool parse(const Json::Value& config, core::RectangleF& that);
    };

    template <>
    struct ConfigParser < util::LogConfig > {
        static bool parse(const Json::Value& config, util::LogConfig& that);
    };

    template <>
    struct ConfigParser < util::LogAppenderConfig > {
        static bool parse(const Json::Value& config, util::LogAppenderConfig& that);
    };

    template <>
    struct ConfigParser < util::FileLogAppenderConfig > {
        static bool parse(const Json::Value& config, util::FileLogAppenderConfig& that);
    };

    template <>
    struct ConfigParser < util::LogLevel > {
        static bool parse(const Json::Value& config, util::LogLevel& that);
    };

} } }
