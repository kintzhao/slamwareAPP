#pragma once

#include "task.h"
#include <list>
#include <boost/thread/mutex.hpp>
#include <boost/thread/condition_variable.hpp>
#include <boost/noncopyable.hpp>

namespace rpos { namespace system { namespace thread_pool {

    class TaskQueue : private boost::noncopyable {
    public:
        TaskQueue();
        ~TaskQueue();

        bool pop(Task& target);
        void push(const Task& task);

        void runOnce();
        void runAll();

        bool waitForTask(Task& target);

        void notifyAll();

        size_t size() const;

    private:
        mutable boost::mutex lock_;
        boost::condition_variable cond_;
        std::list<Task> tasks_;
    };

} } }
