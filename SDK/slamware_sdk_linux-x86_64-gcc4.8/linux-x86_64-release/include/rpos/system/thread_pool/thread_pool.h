#pragma once

#include "thread_pool_worker.h"
#include <vector>

namespace rpos { namespace system { namespace thread_pool {

    class ThreadPool : public boost::enable_shared_from_this<ThreadPool>, private boost::noncopyable {
    public:
        ThreadPool();
        ~ThreadPool();

    public:
        int getMaxThreads() const;
        void setMaxThreads(int v);

        int getAliveThreads() const;

        int getId() const;

    public:
        void pushTask(boost::function<void()> task);
        void pushTask(const std::string& name, boost::function<void()> task);
        void recycle();

        void dispose();

    private:
        friend class ThreadPoolWorker;

        void giveBirthToWorkers_();

        int id_;

        boost::shared_ptr<TaskQueue> taskQueue_;
        std::vector<boost::shared_ptr<ThreadPoolWorker>> workers_;
        int nextWorkerId_;
        int maxThreads_;
        mutable boost::mutex lock_;
        bool disposed_;
    };

} } }
