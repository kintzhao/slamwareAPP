/*
* map.h
* Location Provider Maps
*
* Created by Tony Huang (cnwzhjs@gmail.com) at 2014-05-25
* Copyright 2014 (c) www.robopeak.com
*/

#pragma once

#include <rpos/rpos_config.h>
#include <rpos/system/serialization/i_stream.h>
#include <rpos/system/object_handle.h>
#include <rpos/system/types.h>
#include <rpos/core/geometry.h>
#include <vector>

namespace rpos {
    namespace features {
        namespace location_provider {

            // Base Map
            enum MapType {
                MapTypeBitmap8Bit = 0
            };

            enum MapKind {
                EXPLORERMAP = 0,
                SWEEPERMAP  = 10
            };


            class Map;

            namespace detail {
                class MapImpl;

                template<class MapT>
                struct map_caster {
                    static MapT cast(Map&);
                };
            }

            class RPOS_CORE_API Map : virtual public rpos::system::serialization::ISerializable,
                public rpos::system::ObjectHandle<Map, detail::MapImpl>{
            public:
                RPOS_OBJECT_CTORS(Map);
                ~Map();

            public:
                core::RectangleF& getMapArea() const;
                core::Vector2f& getMapPosition() const;
                core::Vector2i& getMapDimension() const;
                core::Vector2f& getMapResolution() const;
                system::types::timestamp_t getMapTimestamp();
                std::vector<rpos::system::types::_u8>& getMapData() const;

                virtual bool readFromStream(rpos::system::serialization::IStream &in);
                virtual bool writeToStream(rpos::system::serialization::IStream &out) const;

                template<class MapT>
                MapT cast()
                {
                    return detail::map_caster<MapT>::cast(*this);
                }

            private:
                template<class MapT>
                friend struct detail::map_caster;
            };

            // Bitmap map
            enum BitmapMapPixelFormat {
                BitmapMapPixelFormat8Bit
            };

            namespace detail {
                class BitmapMapImpl;
            }

            class RPOS_CORE_API BitmapMap : public Map{
            public:
                typedef detail::BitmapMapImpl impl_t;

                RPOS_OBJECT_CTORS_WITH_BASE(BitmapMap, Map);
                BitmapMap(boost::shared_ptr<detail::BitmapMapImpl> impl);
                ~BitmapMap();
                static BitmapMap createMap();

            public:
                BitmapMapPixelFormat getMapFormat();
                //void* getMapRawData();
                void clear();
                void setMapData(
                    float real_x, float real_y,
                    int dimension_x, int dimension_y,
                    float resolution, const std::vector<rpos::system::types::_u8>& data,
                    rpos::system::types::_u64 timestamp = 0);
                std::vector<rpos::system::types::_u8>& getMapData() const;
            };

        }
    }
}