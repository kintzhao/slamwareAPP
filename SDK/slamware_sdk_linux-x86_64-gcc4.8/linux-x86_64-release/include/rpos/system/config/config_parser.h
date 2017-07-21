#pragma once

#if defined(_WIN32) && !defined(_CRT_SECURE_NO_WARNINGS)
#   define _CRT_SECURE_NO_WARNINGS
#endif

#include <rpos/core/rpos_core_config.h>
#include <json/json.h>
#include <vector>
#include <map>
#include <list>
#include <string>
#include <stdio.h>
#include <stdlib.h>
#include <rpos/system/util/uom.h>

namespace rpos { namespace system { namespace config {

    extern bool rawConfigError(const char* msg, ...);
    extern bool configError(const Json::Value& config, const char* msg, ...);

    template<class T>
    struct ConfigParser {
        static bool parse(const Json::Value& config, T& v);
    };

    template<class T>
    bool parseConfig(const Json::Value& config, T& that)
    {
        return ConfigParser<T>::parse(config, that);
    }

    template<class T>
    bool parseConfigFromFile(const std::string& filename, T& that)
    {
        FILE* file = fopen(filename.c_str(), "rb");

        if (!file)
            return rawConfigError("Failed to open config file: %s", filename.c_str());

        fseek(file, 0, SEEK_END);

        size_t fileSize = ftell(file);

        char* buffer = (char*)malloc(fileSize);

        if (!buffer)
        {
            fclose(file);
            return rawConfigError("Failed to allocate read buffer of %d bytes for file: %s", (int)fileSize, filename.c_str());
        }

        fseek(file, 0, SEEK_SET);
        size_t read = fread(buffer, 1, fileSize, file);

        if (read != fileSize)
        {
            free(buffer);
            fclose(file);
            return rawConfigError("Failed to read config file: %s", filename.c_str());
        }

        fclose(file);

        Json::Value config;
        Json::Reader reader;

        if (!reader.parse(buffer, buffer + fileSize, config, false))
        {
            free(buffer);
            return rawConfigError("Failed to parse config file: %s", filename.c_str());
        }
        
        free(buffer);

        return parseConfig(config, that);
    }

    template<class T>
    bool parseChildConfigWithDefault(const Json::Value& config, const std::string& key, T& that, const T& defaultValue)
    {
        if (!config.isObject())
            return configError(config, "Failed to parse child `%s': config node is not a JSON object", key.c_str());

        if (!config.isMember(key))
        {
            that = defaultValue;
            return true;
        }

        return parseConfig(config[key], that);
    }

    template<class T>
    bool parseChildConfig(const Json::Value& config, const std::string& key, T& that)
    {
        if (!config.isObject())
            return configError(config, "Failed to parse child `%s': config node is not a JSON object", key.c_str());

        if (!config.isMember(key))
        {
            if (parseConfig(Json::Value(), that))
                return true;

            return configError(config, "Child `%s' not found", key.c_str());
        }

        return parseConfig(config[key], that);
    }

#define CONFIG_PARSE_CHILD(Name) \
    do { \
        if (!parseChildConfig(config, #Name, that.Name)) \
            return false; \
    } while (false)

#define CONFIG_PARSE_CHILD_WITH_DEFAULT(Name, Default) \
    do { \
        if (!parseChildConfigWithDefault(config, #Name, that.Name, Default)) \
            return false; \
    } while (false)

    //
    // ------- Begin Composite Types Data Parsers -------
    //
    template<class T>
    struct ConfigParser < std::vector<T> > {
        static bool parse(const Json::Value& config, std::vector<T>& v)
        {
            if (!config.isArray())
                return configError(config, "Failed to parse array: config node is not a JSON array");

            for (size_t i = 0; i < config.size(); i++)
            {
                T item;

                if (!ConfigParser<T>::parse(config[i], item))
                    return false;

                v.push_back(item);
            }

            return true;
        }
    };

    template<class T>
    struct ConfigParser < std::list<T> > {
        static bool parse(const Json::Value& config, std::list<T>& v)
        {
            if (!config.isArray())
                return configError(config, "Failed to parse array: config node is not a JSON array");

            for (size_t i = 0; i < config.size(); i++)
            {
                T item;

                if (!ConfigParser<T>::parse(config[i], item))
                    return false;

                v.push_back(item);
            }

            return true;
        }
    };

    template<class T>
    struct ConfigParser < std::map<std::string, T> > {
        static bool parse(const Json::Value& config, std::map<std::string, T>& v)
        {
            if (!config.isObject())
                return configError(config, "Failed to parse key value store: config node is not a JSON object");

            Json::Value::Members members = config.getMemberNames();

            for (Json::Value::Members::iterator iter = members.begin(); iter != members.end(); iter++)
            {
                T item;

                if (!ConfigParser<T>::parse(config[*iter], item))
                    return false;
                
                v[*iter] = item;
            }

            return true;
        }
	};

	//
	// ------- Begin Basic Types Data Parsers -------
	//
#define SIGNED_INT_CONFIG_PARSER(Type) \
    template<> struct ConfigParser < Type > { \
        static bool parse(const Json::Value& config, Type& v) {\
			if (config.isString()) { \
				if (!util::try_parse_with_unit(config.asString(), v)) \
					return configError(config, "Failed to parse numberic value: string cannot be parsed with uom subsystem"); \
				else \
					return true; \
			} else if (!config.isNumeric()) { \
				return configError(config, "Failed to parse numberic value: config node is not a number"); \
			} \
            v = (Type)config.asInt(); \
            return true; \
        } \
    };

#define UNSIGNED_INT_CONFIG_PARSER(Type) \
    template<> struct ConfigParser < Type > { \
        static bool parse(const Json::Value& config, Type& v) {\
			if (config.isString()) { \
				if (!util::try_parse_with_unit(config.asString(), v)) \
					return configError(config, "Failed to parse numberic value: string cannot be parsed with uom subsystem"); \
				else \
					return true; \
			} else if (!config.isNumeric()) { \
				return configError(config, "Failed to parse numberic value: config node is not a number"); \
			} \
            v = (Type)config.asUInt(); \
            return true; \
        } \
    };

#define FLOAT_CONFIG_PARSER(Type) \
    template<> struct ConfigParser < Type > { \
        static bool parse(const Json::Value& config, Type& v) {\
			if (config.isString()) { \
				if (!util::try_parse_with_unit(config.asString(), v)) \
					return configError(config, "Failed to parse numberic value: string cannot be parsed with uom subsystem"); \
				else \
					return true; \
			} else if (!config.isNumeric()) { \
				return configError(config, "Failed to parse numberic value: config node is not a number"); \
			} \
            v = (Type)config.asDouble(); \
            return true; \
        } \
    };

    SIGNED_INT_CONFIG_PARSER(signed char);
    SIGNED_INT_CONFIG_PARSER(short);
    SIGNED_INT_CONFIG_PARSER(int);
    SIGNED_INT_CONFIG_PARSER(std::int64_t);

    UNSIGNED_INT_CONFIG_PARSER(unsigned char);
    UNSIGNED_INT_CONFIG_PARSER(unsigned short);
    UNSIGNED_INT_CONFIG_PARSER(unsigned int);
    UNSIGNED_INT_CONFIG_PARSER(std::uint64_t);

    FLOAT_CONFIG_PARSER(float);
    FLOAT_CONFIG_PARSER(double);

    template<> struct ConfigParser < std::string > {
        static bool parse(const Json::Value& config, std::string& v)
        {
            if (!config.isString())
                return configError(config, "Failed to parse string value: config node is not a string");

            v = config.asString();

            return true;
        }
    };

    template<> struct ConfigParser < bool > {
        static bool parse(const Json::Value& config, bool& v)
        {
            if (!config.isBool())
                return configError(config, "Failed to parse boolean value: config node is not a bool");

            v = config.asBool();

            return true;
        }
    };

    template<> struct ConfigParser < Json::Value > {
        static bool parse(const Json::Value& config, Json::Value& v) {
            v = config;
            return true;
        }
    };

} } }
