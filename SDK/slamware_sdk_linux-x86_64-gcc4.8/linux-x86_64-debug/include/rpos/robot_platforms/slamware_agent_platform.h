/**
* slamware_agent_platform.h
* Slamtec Slamware(r) Agent platform
*
* Created By Gabriel He @ 2016-01-28
* Copyright (c) 2016 Shanghai SlamTec Co., Ltd.
*/

#pragma once

#include <rpos/features/artifact_provider.h>
#include <rpos/features/location_provider.h>
#include <rpos/features/motion_planner.h>
#include <rpos/features/sweep_motion_planner.h>
#include <rpos/features/system_resource.h>
#include <rpos/features/impact_sensor_feature.h>
#include <rpos/robot_platforms/objects/composite_map.h>
#include <rpos/robot_platforms/objects/slamware_scheduler_service.h>
#include <rpos/robot_platforms/objects/slamware_firmware_service.h>
#include <rpos/robot_platforms/slamware_http_exception.h>
#include <boost/shared_ptr.hpp>

#include <vector>

namespace rpos { namespace robot_platforms {


    namespace detail {
        class SlamwareAgentPlatformImpl;
        class SlamwareActionFactory;
        class SlamwareHttpsClient;
    }

    using namespace rpos::robot_platforms::http;

    class SlamwareAgentPlatform
    {
        friend class detail::SlamwareActionFactory;
    public:
        typedef detail::SlamwareAgentPlatformImpl impl_t;

        SlamwareAgentPlatform();
        SlamwareAgentPlatform(boost::shared_ptr<impl_t> impl);
        ~SlamwareAgentPlatform();

    public:
        static SlamwareAgentPlatform connect(const std::string& host, int port);
        void disconnect();

    public:
        features::ArtifactProvider getArtifactProvider();
        features::LocationProvider getLocationProvider();
        features::MotionPlanner getMotionPlanner();
        features::SweepMotionPlanner getSweepMotionPlanner();
        features::SystemResource getSystemResource();
        features::ImpactSensor getImpactSensor();

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

        bool setMap(const core::Pose& pose, const rpos::features::location_provider::Map& map, rpos::features::location_provider::MapType type, rpos::features::location_provider::MapKind kind, bool partially = false);
        bool setMap(const rpos::features::location_provider::Map& map, rpos::features::location_provider::MapType type, rpos::features::location_provider::MapKind kind, bool partially = false);

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


        rpos::features::system_resource::BaseHealthInfo getRobotHealth();
        void clearRobotHealth(int errorCode);

        bool configurateNetwork(rpos::features::system_resource::NetworkMode mode, const std::map<std::string, std::string>& options);

        std::map<std::string, std::string> getNetworkStatus();


    public:
        // Impact Sensor APIs
        bool getSensors(std::vector<features::impact_sensor::ImpactSensorInfo>& sensors);

        bool getSensorValues(std::map<features::impact_sensor::impact_sensor_id_t, features::impact_sensor::ImpactSensorValue>& values);

        bool getSensorValues(const std::vector<features::impact_sensor::impact_sensor_id_t>& sensorIds, std::vector<features::impact_sensor::ImpactSensorValue>& values);

        bool getSensorValue(features::impact_sensor::impact_sensor_id_t sensorId, features::impact_sensor::ImpactSensorValue& value);

        rpos::robot_platforms::objects::CompositeMap getCompositeMap();

        void setCompositeMap(const rpos::robot_platforms::objects::CompositeMap& map, const core::Pose& pose);


    public:
        // Scheduler Service APIs
        std::vector<detail::objects::ScheduledTask> getScheduledTasks();

        bool addScheduledTask(const detail::objects::ScheduledTask& task);

        detail::objects::ScheduledTask getScheduledTask(int taskId);

        detail::objects::ScheduledTask updateScheduledTask(const detail::objects::ScheduledTask& task);

        bool deleteScheduledTask(int taskId);

    public:
        // Firmware Service APIs
        detail::objects::UpdateInfo getUpdateInfo();

        bool startFirmwareUpdate();

        detail::objects::UpdateProgress getFirmwareUpdateProgress();
                
    private:
        boost::shared_ptr<detail::SlamwareHttpsClient> getHttpsClient();

    private:
        boost::shared_ptr<impl_t> impl_;
    };
}}
