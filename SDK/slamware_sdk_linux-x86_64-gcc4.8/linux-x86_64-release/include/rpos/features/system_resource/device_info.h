/**
* device_info.h
* 
*
* Created   @ 2016-7-4
* Copyright (c) 2014 Shanghai SlamTec Co., Ltd.
*/

#pragma once

#include <rpos/rpos_config.h>
#include <rpos/core/rpos_core_config.h>
#include <string>

namespace rpos { namespace features { namespace system_resource {
            
    class RPOS_CORE_API DeviceInfo
	{
    public:
		DeviceInfo();
		DeviceInfo(const DeviceInfo&);

		~DeviceInfo();
            
    public:
        DeviceInfo& operator=(const DeviceInfo&);

    public:
        std::string deviceID() const;
        std::string& deviceID();

        int manufacturerID() const;
        int& manufacturerID();

        std::string manufacturerName() const;
        std::string& manufacturerName();

        int modelID() const;
        int& modelID();

        std::string modelName() const;
        std::string& modelName();

        std::string hardwareVersion() const;
        std::string& hardwareVersion();

        std::string softwareVersion() const;
        std::string& softwareVersion();

    private:
        std::string deviceId_;
        int modelId_;
        std::string modelName_;
        int manufacturerId_;
        std::string manufacturerName_;
        std::string hardwareVersion_;
        std::string softwareVersion_;
    };

} } }
