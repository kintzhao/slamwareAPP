#pragma once

#include <rpos/core/geometry.h>
#include <rpos/core/pose.h>

#include <rpos/robot_platforms/objects/map_layer.h>

#include <vector>

namespace rpos { namespace robot_platforms { namespace objects {

    class RPOS_SLAMWARE_API GridMapLayer : public MapLayer
    {
    public:
        static const char* const Type;

    public:
        virtual void clear(void);

    public:
        const core::Location& getOrigin() const;
        void setOrigin(const core::Location& rcSrc);
        
        const core::Vector2i& getDimension() const;
        void setDimension(const core::Vector2i& rcSrc);

        const core::Vector2f& getResolution() const;
        void setResolution(const core::Vector2f& rcSrc);

        const std::vector<uint8_t>& mapData() const;
        std::vector<uint8_t>& mapData();

    private:
        core::Location origin_;
        core::Vector2i dimension_;
        core::Vector2f resolution_;
        std::vector<uint8_t> mapData_;
    };

}}}
