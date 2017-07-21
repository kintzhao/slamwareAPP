#pragma once

#include "json_serialization.h"
#include <rpos/core/pose.h>
#include <rpos/core/geometry.h>
#include <rpos/system/util/log.h>
#include <rpos/core/diagnosis_info.h>
#include <rpos/core/rpos_core_config.h>

namespace rpos { namespace system { namespace serialization { namespace json {

    template <>
	struct Serializer < core::Pose >
    {
        static RPOS_CORE_API Json::Value serialize(const core::Pose& v);
        static RPOS_CORE_API core::Pose deserialize(const Json::Value& v);
    };

    template <>
	struct Serializer < core::RectangleF >
    {
        static RPOS_CORE_API Json::Value serialize(const core::RectangleF& v);
        static RPOS_CORE_API core::RectangleF deserialize(const Json::Value& v);
    };

    template <>
	struct Serializer < core::Vector2i >
    {
        static RPOS_CORE_API Json::Value serialize(const core::Vector2i& v);
        static RPOS_CORE_API core::Vector2i deserialize(const Json::Value& v);
    };

    template <>
	struct Serializer < core::Vector3i >
    {
        static RPOS_CORE_API Json::Value serialize(const core::Vector3i& v);
        static RPOS_CORE_API core::Vector3i deserialize(const Json::Value& v);
    };

    template <>
	struct Serializer < core::Vector4i >
    {
        static RPOS_CORE_API Json::Value serialize(const core::Vector4i& v);
        static RPOS_CORE_API core::Vector4i deserialize(const Json::Value& v);
    };

    template <>
	struct Serializer < core::Vector2f >
    {
        static RPOS_CORE_API Json::Value serialize(const core::Vector2f& v);
        static RPOS_CORE_API core::Vector2f deserialize(const Json::Value& v);
    };

    template <>
	struct Serializer < core::Vector3f >
    {
        static RPOS_CORE_API Json::Value serialize(const core::Vector3f& v);
        static RPOS_CORE_API core::Vector3f deserialize(const Json::Value& v);
    };

    template <>
	struct Serializer < core::Vector4f >
    {
        static RPOS_CORE_API Json::Value serialize(const core::Vector4f& v);
        static RPOS_CORE_API core::Vector4f deserialize(const Json::Value& v);
    };

    template <>
	struct Serializer < core::DiagnosisInfoScanData >
    {
        static RPOS_CORE_API Json::Value serialize(const core::DiagnosisInfoScanData& v);
        static RPOS_CORE_API core::DiagnosisInfoScanData deserialize(const Json::Value& v);
    };

    template <>
	struct Serializer < core::DiagnosisInfoLidarScan >
    {
        static RPOS_CORE_API Json::Value serialize(const core::DiagnosisInfoLidarScan& v);
        static RPOS_CORE_API core::DiagnosisInfoLidarScan deserialize(const Json::Value& v);
    };

    template <>
	struct Serializer < core::DiagnosisInfoDepthCameraScan >
    {
        static RPOS_CORE_API Json::Value serialize(const core::DiagnosisInfoDepthCameraScan& v);
        static RPOS_CORE_API core::DiagnosisInfoDepthCameraScan deserialize(const Json::Value& v);
    };

    template <>
	struct Serializer < core::DiagnosisInfoSensor >
    {
        static RPOS_CORE_API Json::Value serialize(const core::DiagnosisInfoSensor& v);
        static RPOS_CORE_API core::DiagnosisInfoSensor deserialize(const Json::Value& v);
    };

} } } }
