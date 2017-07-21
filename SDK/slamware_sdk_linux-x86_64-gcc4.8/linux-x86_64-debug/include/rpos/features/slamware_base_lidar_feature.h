#pragma once

#include <rp/slamware/features/lidar_feature.h>
#include <rp/slamware/config/config_parser.h>

namespace rp { namespace slamware { namespace features {

    namespace detail {

        class SlamwareBaseLidarFeatureImpl;

    }

    struct SlamwareBaseLidarFeatureConfig {
        float requiredFreq;
		rpos::core::Pose pose;
    };

    class SlamwareBaseLidarFeature : public LidarFeature {
    public:
        RPOS_OBJECT_CTORS_WITH_BASE(SlamwareBaseLidarFeature, LidarFeature);
        ~SlamwareBaseLidarFeature() = default;

        SlamwareBaseLidarFeature(const SlamwareBaseLidarFeatureConfig& config);
    };

} } }

namespace rp { namespace slamware { namespace config {

    template<>
    struct ConfigParser < features::SlamwareBaseLidarFeatureConfig >
    {
        static bool parse(const Json::Value& config, features::SlamwareBaseLidarFeatureConfig& that)
        {
            CONFIG_PARSE_CHILD_WITH_DEFAULT(requiredFreq, 6.0f);
			//CONFIG_PARSE_CHILD(pose);
            return true;
        }
    };

} } }
