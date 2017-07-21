#pragma once

#include <string>

namespace rpos { namespace robot_platforms { namespace detail { namespace objects {

    struct ScheduledTask
    {
        int id;
        int hour;
        int minute;
        int startYear;
        int startMonth;
        int startDay;
        int maxWorkMinute;
        bool enabled;
        /*ScheduledTaskRepeat*/int repeat;
        std::string task;
    };

}}}}