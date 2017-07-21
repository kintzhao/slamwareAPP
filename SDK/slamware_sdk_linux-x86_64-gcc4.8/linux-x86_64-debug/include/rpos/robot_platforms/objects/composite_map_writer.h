#pragma once

#include <rpos/robot_platforms/objects/composite_map.h>

#include <boost/noncopyable.hpp>

namespace rpos { namespace robot_platforms { namespace objects {

    class CompositeMapWriterImpl;

    class RPOS_SLAMWARE_API CompositeMapWriter : private boost::noncopyable
    {
    public:
        CompositeMapWriter(void);
        ~CompositeMapWriter(void);

    public:
        // throw exception if error occurs
        void saveFile(const std::string& rcFilePath, const CompositeMap& rcCmpstMap);
        void saveFile(const std::wstring& rcFilePath, const CompositeMap& rcCmpstMap);

        // returns true if succeed, and "rErrMsg" will be empty;
        // returns false if error occurs, and error message will be in "rErrMsg".
        bool saveFile(std::string& rErrMsg, const std::string& rcFilePath, const CompositeMap& rcCmpstMap);
        bool saveFile(std::string& rErrMsg, const std::wstring& rcFilePath, const CompositeMap& rcCmpstMap);

    private:
        template<class _PathStrT>
        bool doSaveFile_T_(std::string& rErrMsg, const _PathStrT& rcFilePath, const CompositeMap& rcCmpstMap);

    private:
        CompositeMapWriterImpl* m_pImpl;
    };

}}}
