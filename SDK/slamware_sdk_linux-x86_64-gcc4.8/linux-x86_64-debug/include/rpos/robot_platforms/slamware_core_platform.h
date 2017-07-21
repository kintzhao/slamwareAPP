/**
* slamware_core_platform.h
* Slamtec Slamware(r) Core robot platform
*
* Created By Tony Huang @ 2015-3-31
* Copyright (c) 2015 Shanghai SlamTec Co., Ltd.
*/

#pragma once

#ifdef _MSC_VER
#   pragma warning(disable: 4290)
#endif

#include <rpos/rpos_config.h>
#include <rpos/core/geometry.h>
#include <rpos/core/robot_platform.h>
#include <rpos/features/artifact_provider.h>
#include <rpos/features/location_provider.h>
#include <rpos/features/motion_planner.h>
#include <rpos/features/sweep_motion_planner.h>
#include <rpos/features/system_resource.h>
#include <rpos/features/impact_sensor_feature.h>
#include <rpos/robot_platforms/objects/composite_map.h>
#include <rpos/robot_platforms/slamware_common_exception.h>
#include <rpos/robot_platforms/slamware_sdp_platform_config.h>

namespace rpos { namespace robot_platforms {

    namespace detail {
        class SlamwareCorePlatformImpl;
        class SlamwareActionFactory;
        class SlamwareTcpClient;
    }

    class RPOS_SLAMWARE_API SlamwareCorePlatform : public rpos::core::RobotPlatform {
        friend class detail::SlamwareActionFactory;
    public:
        typedef detail::SlamwareCorePlatformImpl impl_t;
        

        RPOS_OBJECT_CTORS_WITH_BASE(SlamwareCorePlatform, rpos::core::RobotPlatform);
        SlamwareCorePlatform(boost::shared_ptr<impl_t> impl);
        ~SlamwareCorePlatform();

    public:
        static SlamwareCorePlatform connect(const std::string& host, int port, int timeoutInMs = 10000)
            throw(ConnectionTimeOutException, ConnectionFailException);
        void disconnect();

    public:
        rpos::features::ArtifactProvider getArtifactProvider();
        rpos::features::LocationProvider getLocationProvider();
        rpos::features::MotionPlanner getMotionPlanner();
        rpos::features::SweepMotionPlanner getSweepMotionPlanner();
        rpos::features::SystemResource getSystemResource();
        rpos::features::ImpactSensor getImpactSensor();

    public:
        // Artifacts Provider APIs
        std::vector<core::Line> getWalls();

        bool addWall(const core::Line& wall);

        bool addWalls(const std::vector<core::Line>& walls);

        bool clearWallById(const rpos::core::SegmentID& id);

        bool clearWalls();

    public:
        // Location Provider APIs
        std::vector<rpos::features::location_provider::MapType> getAvailableMaps();

        rpos::features::location_provider::Map getMap(rpos::features::location_provider::MapType type, core::RectangleF area, rpos::features::location_provider::MapKind kind);

        bool setMap(const rpos::features::location_provider::Map& map, rpos::features::location_provider::MapType type, rpos::features::location_provider::MapKind kind, bool partially = false);
        bool setMap(const core::Pose& pose, const rpos::features::location_provider::Map& map, rpos::features::location_provider::MapType type, rpos::features::location_provider::MapKind kind, bool partially = false);

        core::RectangleF getKnownArea(rpos::features::location_provider::MapType type, rpos::features::location_provider::MapKind kind);

        bool clearMap();

        bool clearMap(rpos::features::location_provider::MapKind kind);

        core::Location getLocation();

        core::Pose getPose();

        bool setPose(const core::Pose& pose);

        bool getMapLocalization();

        bool setMapLocalization(bool localization);

        bool getMapUpdate();

        bool setMapUpdate(bool update);

        int getLocalizationQuality();

    public:
        // Motion Planner APIs
        rpos::actions::MoveAction moveTo(const std::vector<rpos::core::Location>& locations, bool appending, bool isMilestone);

        rpos::actions::MoveAction moveTo(const rpos::core::Location& location, bool appending, bool isMilestone);

        rpos::actions::MoveAction moveBy(const rpos::core::Direction& direction);

        rpos::actions::MoveAction rotateTo(const rpos::core::Rotation& orientation);

        rpos::actions::MoveAction rotate(const rpos::core::Rotation& rotation);

        rpos::actions::MoveAction getCurrentAction();

        rpos::features::motion_planner::Path searchPath(const rpos::core::Location& location);

        rpos::actions::SweepMoveAction startSweep();

		rpos::actions::SweepMoveAction sweepSpot(const rpos::core::Location& location);

        rpos::actions::MoveAction goHome();

    public:
        // System Resource APIs
        int getBatteryPercentage();

        bool getBatteryIsCharging();

        bool getDCIsConnected();

        int getBoardTemperature();

        std::string getSDPVersion();

        std::string getSDKVersion();

        rpos::features::system_resource::LaserScan getLaserScan();

        bool restartModule(rpos::features::system_resource::RestartMode mode = rpos::features::system_resource::RestartModeSoft);

        bool setSystemParameter(const std::string& param, const std::string& value);

        std::string getSystemParameter(const std::string& param);

        rpos::features::system_resource::DeviceInfo getDeviceInfo();

        void startCalibration(rpos::features::system_resource::CalibrationType type);

        void stopCalibration();


        rpos::features::system_resource::BaseHealthInfo getRobotHealth();
        void clearRobotHealth(int errorCode);


        bool configurateNetwork(rpos::features::system_resource::NetworkMode mode, const std::map<std::string, std::string>& options);

        std::map<std::string, std::string> getNetworkStatus();

    public:
        // Impact Sensor APIs
        bool getSensors(std::vector<rpos::features::impact_sensor::ImpactSensorInfo>& sensors);

        bool getSensorValues(std::map<rpos::features::impact_sensor::impact_sensor_id_t, rpos::features::impact_sensor::ImpactSensorValue>& values);

        bool getSensorValues(const std::vector<rpos::features::impact_sensor::impact_sensor_id_t>& sensorIds, std::vector<rpos::features::impact_sensor::ImpactSensorValue>& values);

        bool getSensorValue(rpos::features::impact_sensor::impact_sensor_id_t sensorId, rpos::features::impact_sensor::ImpactSensorValue& value);

    public:
        // Composite Map APIs
        rpos::robot_platforms::objects::CompositeMap getCompositeMap();

        void setCompositeMap(const rpos::robot_platforms::objects::CompositeMap& map, const core::Pose& pose);

    private:
        boost::shared_ptr<detail::SlamwareTcpClient> getTcpClient();

    };

} }
