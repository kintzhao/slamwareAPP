#pragma once

#include <boost/noncopyable.hpp>

#include <rpos/robot_platforms/objects/metadata.h>

#include <string>

namespace rpos { namespace robot_platforms { namespace objects {

    class RPOS_SLAMWARE_API MapLayer : public boost::noncopyable
    {
    public:
        virtual ~MapLayer();

    public:
        virtual void clear(void);

    public:
        const Metadata& metadata() const;
        Metadata& metadata();
        
        const std::string& getName() const;
        void setName(const std::string& value);

        const std::string& getUsage() const;
        void setUsage(const std::string& value);

        const std::string& getType() const;
        void setType(const std::string& value);

    private:
        Metadata metadata_;
        std::string name_;
        std::string usage_;
        std::string type_;
    };

    //////////////////////////////////////////////////////////////////////////

    class RPOS_SLAMWARE_API UnknownMapLayer : public MapLayer
    {
    public:
        virtual ~UnknownMapLayer();

    public:
        virtual void clear(void);

    public:
        const std::vector<unsigned char>& rawBody() const { return raw_body_; }
        std::vector<unsigned char>& rawBody() { return raw_body_; }

    private:
        std::vector<unsigned char> raw_body_;
    };

}}}
