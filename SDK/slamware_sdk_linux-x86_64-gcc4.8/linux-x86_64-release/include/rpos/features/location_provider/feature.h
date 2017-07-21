/*
* feature.h
* Location Provider feature
*
* Created by Tony Huang (cnwzhjs@gmail.com) at 2014-05-25
* Copyright 2014 (c) www.robopeak.com
*/

#pragma once

#include <rpos/rpos_config.h>

#include <rpos/core/feature.h>
#include <rpos/core/pose.h>
#include <vector>

#include "map.h"

namespace rpos {
    namespace features {

        namespace detail {
            class LocationProviderImpl;
        }

        class RPOS_CORE_API LocationProvider : public rpos::core::Feature{
        public:
            typedef detail::LocationProviderImpl impl_t;

            RPOS_OBJECT_CTORS_WITH_BASE(LocationProvider, rpos::core::Feature);
            LocationProvider(boost::shared_ptr<detail::LocationProviderImpl> impl);
            ~LocationProvider();

        public:
            std::vector<location_provider::MapType> getAvailableMaps();
            location_provider::Map getMap(location_provider::MapType type, core::RectangleF area, location_provider::MapKind kind);
            bool setMap(const location_provider::Map& map, location_provider::MapType type, location_provider::MapKind kind, bool partially);
            core::RectangleF getKnownArea(location_provider::MapType type, location_provider::MapKind kind);
            bool clearMap();
            bool clearMap(location_provider::MapKind kind);

            core::Location getLocation();
            core::Pose getPose();
            bool setPose(const core::Pose& pose);
            bool getMapLocalization();
            bool setMapLocalization(bool localization);
            bool getMapUpdate();
            bool setMapUpdate(bool update);
            int getLocalizationQuality();
        };
        
    }
}