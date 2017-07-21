#pragma once

#include <rpos/core/feature.h>
#include <rpos/core/pose.h>
#include <rpos/core/laser_point.h>

#include <vector>

namespace rpos {namespace features {

    namespace detail {

        class LidarFeatureImpl;

    }

    struct LidarCapability {
        float deadZone; // dead zone in meter
        float maxDistance; // in meter
        unsigned int maxSampleRate; // in hz
        float maxScanRate; // in hz
    };

    typedef unsigned int laser_scan_id_t;

    struct LaserScan {
        laser_scan_id_t id;
        std::vector<rpos::core::LaserPoint> points;
    };

    class LidarFeature : public rpos::core::Feature {
    public:
        RPOS_OBJECT_CTORS_WITH_BASE(LidarFeature, rpos::core::Feature);
        ~LidarFeature();

    public:
        rpos::core::Pose getSensorPose();

        bool getCapability(LidarCapability* cap);

        bool getSampleRate(unsigned int* rate);
        bool getScanRate(float* rate);

        bool setSampleRate(unsigned int rate);

        bool rampup(float rate);
        bool stop();

        bool grabNewScan(LaserScan& scan, unsigned int timeout = 3000u);
        bool grabNewScan(laser_scan_id_t lastScanId, LaserScan& scan, unsigned int timeout = 3000u);
        bool getCurrentScan(LaserScan& scan, unsigned int timeout = 3000u);
    };

} }
