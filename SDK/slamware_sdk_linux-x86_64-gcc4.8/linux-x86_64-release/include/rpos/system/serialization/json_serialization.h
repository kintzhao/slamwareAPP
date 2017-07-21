/*
* json_serialization.h
* RPOS JSON Serialization Utilities
*
* Created By Tony Huang (cnwzhjs@gmail.com) at 2014-12-11
* Copyright 2014 (c) Shanghai Slamtec Co., Ltd.
*/

#pragma once

#include <rpos/rpos_config.h>
#include <json/json.h>
#include <vector>
#include <list>
#include <map>
#include <cstdint>

#define RPOS_LIB_NAME rpos_deps_jsoncpp
#	include <rpos/system/util/auto_link.h>
#undef RPOS_LIB_NAME

namespace rpos { namespace system { namespace serialization { namespace json {

    template <class T>
    struct Serializer
    {
        static Json::Value serialize(const T& v);
        static T deserialize(const Json::Value& jsonValue);
    };

    template <class T>
    Json::Value serialize(const T& v)
    {
        return Serializer<T>::serialize(v);
    }

    template <class T>
    T deserialize(const Json::Value& v)
    {
        return Serializer<T>::deserialize(v);
    }

    template <>
    struct Serializer < std::nullptr_t >
    {
        static Json::Value serialize(const std::nullptr_t& v)
        {
            return Json::Value();
        }

        static std::nullptr_t deserialize(const Json::Value& v)
        {
            return nullptr;
        }
    };

    template <>
    struct Serializer < Json::Value >
    {
        static Json::Value serialize(const Json::Value& v)
        {
            return v;
        }

        static Json::Value deserialize(const Json::Value& v)
        {
            return v;
        }
    };

    template <>
    struct Serializer < std::string >
    {
        static Json::Value serialize(const std::string& v)
        {
            return Json::Value(v);
        }

        static std::string deserialize(const Json::Value& v)
        {
            return v.asString();
        }
    };

    template <>
    struct Serializer < int >
    {
        static Json::Value serialize(const int& v)
        {
            return Json::Value(v);
        }

        static int deserialize(const Json::Value& v)
        {
            return v.asInt();
        }
    };

    template <>
    struct Serializer < unsigned int >
    {
        static Json::Value serialize(const unsigned int& v)
        {
            return Json::Value(v);
        }

        static unsigned int deserialize(const Json::Value& v)
        {
            return v.asUInt();
        }
    };

    template <>
    struct Serializer < std::uint64_t >
    {
        static Json::Value serialize(const std::uint64_t& v)
        {
            return Json::Value((unsigned int)v);
        }

        static std::uint64_t deserialize(const Json::Value& v)
        {
            return v.asUInt();
        }
    };

    template <>
    struct Serializer < bool >
    {
        static Json::Value serialize(const bool& v)
        {
            return Json::Value(v);
        }

        static bool deserialize(const Json::Value& v)
        {
            return v.asBool();
        }
    };

    template <>
    struct Serializer < float >
    {
        static Json::Value serialize(const float& v)
        {
            return Json::Value(v);
        }

        static float deserialize(const Json::Value& v)
        {
            return (float)v.asDouble();
        }
    };

    template <>
    struct Serializer < double >
    {
        static Json::Value serialize(const double& v)
        {
            return Json::Value(v);
        }

        static double deserialize(const Json::Value& v)
        {
            return (double)v.asDouble();
        }
    };

    template <class T>
    struct Serializer < std::map<std::string, T> >
    {
        static Json::Value serialize(const std::map<std::string, T>& v)
        {
            Json::Value output;

            for (auto iter = v.begin(); iter != v.end(); iter++)
            {
                output[iter->first] = json::serialize(iter->second);
            }
            return output;
        }

        static std::map<std::string, T> deserialize(const Json::Value& v)
        {
            std::map<std::string, T> output;

            std::vector<std::string> memberNames = v.getMemberNames();

            for (auto iter = memberNames.begin(); iter != memberNames.end(); iter++)
            {
                output[*iter] = json::deserialize<T>(v[*iter]);
            }

            return output;
        }
    };

    template <class T>
    struct Serializer < std::vector<T> >
    {
        static Json::Value serialize(const std::vector<T>& v)
        {
            Json::Value output(Json::arrayValue);

            for (auto iter = v.begin(); iter != v.end(); iter++)
            {
                output.append(json::serialize(*iter));
            }

            return output;
        }

        static std::vector<T> deserialize(const Json::Value& v)
        {
            std::vector<T> output;

            for (size_t i = 0; i < v.size(); i++)
            {
                output.push_back(json::deserialize<T>(v[i]));
            }

            return output;
        }
    };

    template <class T>
    struct Serializer < std::list<T> >
    {
        static Json::Value serialize(const std::list<T>& v)
        {
            Json::Value output(Json::arrayValue);

            for (auto iter = v.begin(); iter != v.end(); iter++)
            {
                output.append(json::serialize(*iter));
            }

            return output;
        }

        static std::list<T> deserialize(const Json::Value& v)
        {
            std::list<T> output;

            for (size_t i = 0; i < v.size(); i++)
            {
                output.push_back(json::deserialize<T>(v[i]));
            }

            return output;
        }
    };

}}}}
