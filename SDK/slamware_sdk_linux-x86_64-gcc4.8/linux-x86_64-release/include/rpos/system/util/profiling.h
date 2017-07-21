/*
* profiling.h
* Generic profiling feature
*
* Created By Tony Huang (cnwzhjs@gmail.com) at 2014-10-14
* Copyright 2014 (c) www.robopeak.com
*/

#pragma once

#include <rpos/rpos_config.h>
#include <boost/noncopyable.hpp>
#include <boost/shared_ptr.hpp>
#include <string>

namespace rpos {
    namespace system {
        namespace util {

            class ProfilingScopeImpl;

            class ProfilingScope : public boost::noncopyable {
            public:
                ProfilingScope(const std::string& module, const std::string& operation);
                ~ProfilingScope();

                void addArgument(const std::string& arg);
                void addCheckPoint(const std::string& checkpoint);

            private:
                boost::shared_ptr<ProfilingScopeImpl> impl_;
            };

        }
    }
}
