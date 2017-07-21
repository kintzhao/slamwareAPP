#pragma once

#include <rpos/core/pose.h>

#include <rpos/robot_platforms/objects/metadata.h>
#include <rpos/robot_platforms/objects/map_layer.h>

#include <string>

namespace rpos { namespace robot_platforms { namespace objects {

    struct Line
    {
        std::string name;
        core::Location start, end;
        Metadata metadata;
    };

    class RPOS_SLAMWARE_API LineMapLayer : public MapLayer
    {
    public:
        static const char* const Type;

    public:
        virtual void clear(void);

    public:
        const std::map<std::string, Line>& lines() const;
        std::map<std::string, Line>& lines();

    private:
        std::map<std::string, Line> lines_;
    };

}}}