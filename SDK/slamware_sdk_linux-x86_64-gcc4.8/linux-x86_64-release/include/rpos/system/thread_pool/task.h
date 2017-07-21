#pragma once

#include <boost/function.hpp>
#include <string>

namespace rpos { namespace system { namespace thread_pool {

    class Task {
    public:
        Task();
        Task(boost::function<void()> func);
        Task(boost::function<void()> func, const std::string& name);

    public:
        const std::string& name() const;
        void execute();

    private:
        boost::function<void()> func_;
        std::string name_;
    };

} } }
