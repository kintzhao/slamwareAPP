/**
* laser_scan.h
* Laser Scan to store the measured laser point from lidar
*
* Created By Jacky Li @ 2014-7-20
* Copyright (c) 2014 Shanghai SlamTec Co., Ltd.
*/

#pragma once

#include <rpos/rpos_config.h>
#include <rpos/core/pose.h>
#include <rpos/system/object_handle.h>
#include <rpos/core/laser_point.h>
#include <rpos/system/types.h>
#include <vector>

namespace rpos {
    namespace features {
        namespace system_resource {

            namespace detail {
                class LaserScanImpl;
            }

            class RPOS_CORE_API LaserScan : public rpos::system::ObjectHandle<LaserScan, detail::LaserScanImpl>{
            public:
                RPOS_OBJECT_CTORS(LaserScan);
                LaserScan(const std::vector<core::LaserPoint>& laserPoints);

#ifdef RPOS_HAS_RVALUE_REFS
                LaserScan(std::vector<core::LaserPoint>&& laserPoints);
#endif

                ~LaserScan();

            public:
                void setLaserPoints(
                    const std::vector<core::LaserPoint>& data,
                    rpos::system::types::_u64 timestamp);
                const std::vector<core::LaserPoint>& getLaserPoints() const;

                void setLaserPointsPose(const rpos::core::Pose& pose);
                const rpos::core::Pose& getLaserPointsPose() const;

                void setHasPose(bool hasPose);
                bool getHasPose() const;
            };

        }
    }
}
