#pragma once
#include <rpos/core/pose.h>
#include <vector>
#include <string>
namespace rpos { namespace core {

    struct DiagnosisInfoScanData
    {
        float dist; // in meter
        float angle; // in degree
    };

    struct DiagnosisInfoLidarScan
    {
        std::vector<DiagnosisInfoScanData> lidarScan;
        Pose lidarPose;
    };

    struct DiagnosisInfoDepthCameraScan
    {
        std::vector<DiagnosisInfoScanData> depthCameraScan;
        Pose depthCameraPose;
    };

    struct DiagnosisInfoSensor
    {
        unsigned int sensorId;
        std::string sensorType;
        Pose sensorPose;
        float rawDist;
        bool valid;
    };
    typedef std::vector<DiagnosisInfoSensor> DiagnosisInfoSensorsData;
  
    template<class T>
    struct DiagnosisInfoNameTrait
    {
        static const char* name;
    };

    template<>
    struct DiagnosisInfoNameTrait<DiagnosisInfoLidarScan>
    {
        static const char* name;
    };

    template<>
    struct DiagnosisInfoNameTrait<DiagnosisInfoSensorsData>
    {
        static const char* name;
    };

    template<>
    struct DiagnosisInfoNameTrait<DiagnosisInfoDepthCameraScan>
    {
        static const char* name;
    };

    template<>
    struct DiagnosisInfoNameTrait<std::string>
    {
        static const char* name;
    };

}}