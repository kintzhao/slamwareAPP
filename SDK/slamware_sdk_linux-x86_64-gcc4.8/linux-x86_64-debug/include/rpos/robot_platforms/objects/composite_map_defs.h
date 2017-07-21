#pragma once

#include <rpos/system/exception.h>

#if (defined(_DEBUG) || defined(DEBUG))
    #define RPOS_COMPOSITEMAP_ASSERT              assert
#else
    #define RPOS_COMPOSITEMAP_ASSERT(expr)
#endif
#define RPOS_COMPOSITEMAP_NOT_USED(x)       ((void)(x))

#define RPOS_COMPOSITEMAP_THROW_EXCEPTION(msg)      throw ::rpos::robot_platforms::objects::CompositeMapException((msg), __FILE__, __FUNCTION__, __LINE__)

#define RPOS_COMPOSITEMAP_METADATA_KEY_NAME                     "name"
#define RPOS_COMPOSITEMAP_METADATA_KEY_USAGE                    "usage"
#define RPOS_COMPOSITEMAP_METADATA_KEY_TYPE                     "type"
#define RPOS_COMPOSITEMAP_METADATA_KEY_COMPRESSION              "compression"
#define RPOS_COMPOSITEMAP_METADATA_KEY_COUNT                    "count"

#define RPOS_COMPOSITEMAP_METADATA_KEY_ORIGIN_X                 "origin_x"
#define RPOS_COMPOSITEMAP_METADATA_KEY_ORIGIN_Y                 "origin_y"
#define RPOS_COMPOSITEMAP_METADATA_KEY_DIMENSION_WIDTH          "dimension_width"
#define RPOS_COMPOSITEMAP_METADATA_KEY_DIMENSION_HEIGHT         "dimension_height"
#define RPOS_COMPOSITEMAP_METADATA_KEY_RESOLUTION_X             "resolution_x"
#define RPOS_COMPOSITEMAP_METADATA_KEY_RESOLUTION_Y             "resolution_y"

namespace rpos { namespace robot_platforms { namespace objects {

    class CompositeMapException : public rpos::system::detail::ExceptionBase
    {
    private:
        typedef rpos::system::detail::ExceptionBase             super_class_type;
    public:
        explicit CompositeMapException(const std::string& msg = "")
            : super_class_type(msg)
        {
            //
        }
        CompositeMapException(const std::string& msg, const char* file, const char* func, int line)
            : super_class_type(msg)
        {
            this->super_class_type::Init(file, func, line);
        }
        virtual ~CompositeMapException(void) throw()
        {
            //
        }
    };

}}}
