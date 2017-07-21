#pragma once

#include <rpos/robot_platforms/objects/composite_map.h>

#include <boost/noncopyable.hpp>

namespace rpos { namespace robot_platforms { namespace objects {

    class CompositeMapReaderImpl;

    class RPOS_SLAMWARE_API CompositeMapReader : private boost::noncopyable
    {
    public:
        CompositeMapReader(void);
        ~CompositeMapReader(void);

    public:
        // throw exception if error occurs
        boost::shared_ptr<CompositeMap> loadFile(const std::string& rcFilePath);
        boost::shared_ptr<CompositeMap> loadFile(const std::wstring& rcFilePath);

        // returns valid pointer if succeed, and "rErrMsg" will be empty;
        // returns invalid pointer if error occurs, and error message will be in "rErrMsg".
        boost::shared_ptr<CompositeMap> loadFile(std::string& rErrMsg, const std::string& rcFilePath);
        boost::shared_ptr<CompositeMap> loadFile(std::string& rErrMsg, const std::wstring& rcFilePath);

    private:
        template<class _PathStrT>
        boost::shared_ptr<CompositeMap> doLoadFile_T_(std::string& rErrMsg, const _PathStrT& rcFilePath);

    private:
        CompositeMapReaderImpl* m_pImpl;
    };

}}}
