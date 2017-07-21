#pragma once

#include <rpos/core/feature.h>
#include <rpos/core/pose.h>

#include <vector>
#include <map>
#include <cstdint>

namespace rpos {namespace features {

    namespace detail {
        
        class ImpactSensorImpl;

    }

    namespace impact_sensor {

        enum ImpactSensorType {
            ImpactSensorTypeDigital, // two value sensors, zero to FLT_EPSILON for impact, FLT_MAX for non-impact 
            ImpactSensorTypeAnalog // analog value sensors, report distance in meter
        };

		enum ImpactSensorKind {
			ImpactSensorKindBumper,
			ImpactSensorKindCliff,
			ImpactSensorKindSonar
		};

        typedef int impact_sensor_id_t;
        typedef std::uint32_t impact_sensor_timestamp_t;

        struct ImpactSensorValue {
            impact_sensor_timestamp_t time;
            float value;
        };

        struct ImpactSensorInfo {
            impact_sensor_id_t id;
            rpos::core::Pose pose;
            ImpactSensorType type;
			ImpactSensorKind kind;
            float refreshFreq;
        };

    }

    class RPOS_CORE_API ImpactSensor : public rpos::core::Feature {
    public:
        RPOS_OBJECT_CTORS_WITH_BASE(ImpactSensor, rpos::core::Feature);
        ImpactSensor(boost::shared_ptr<detail::ImpactSensorImpl> impl);
        ~ImpactSensor();

    public:
        bool getSensors(std::vector<impact_sensor::ImpactSensorInfo>& sensors);

        bool getSensorValues(std::map<impact_sensor::impact_sensor_id_t, impact_sensor::ImpactSensorValue>& values);
        bool getSensorValues(const std::vector<impact_sensor::impact_sensor_id_t>& sensorIds, std::vector<impact_sensor::ImpactSensorValue>& values);
        bool getSensorValue(impact_sensor::impact_sensor_id_t sensorId, impact_sensor::ImpactSensorValue& value);
    };

} }
