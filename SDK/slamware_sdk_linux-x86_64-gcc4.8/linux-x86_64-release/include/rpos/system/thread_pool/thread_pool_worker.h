#pragma once

#include "task_queue.h"
#include <boost/noncopyable.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/enable_shared_from_this.hpp>
#include <boost/thread.hpp>

#define RPOS_SYSTEM_THREAD_POOL_TIMEOUT_TO_BE_ZOMBIE_IN_MS  10000
// if some job cost more than 10 seconds, this thread will be marked as zombie,
// so the pool will invoke another thread to process the rest requests

namespace rpos { namespace system { namespace thread_pool {

    class ThreadPool;

    enum ThreadPoolWorkerStatus {
        ThreadPoolWorkerStatusStopped,
        ThreadPoolWorkerStatusWaitingForJob,
        ThreadPoolWorkerStatusWorking,
        ThreadPoolWorkerStatusZombie
    };

    class ThreadPoolWorker : public boost::enable_shared_from_this<ThreadPoolWorker>, private boost::noncopyable {
    public:
        ThreadPoolWorker(boost::shared_ptr<ThreadPool> threadPool, int workerId);
        ~ThreadPoolWorker();

    public:
        void start();
        void stop();

        ThreadPoolWorkerStatus getStatus();

    private:
        void worker_();

    private:
        boost::thread thread_;
        ThreadPoolWorkerStatus status_;
        boost::shared_ptr<ThreadPool> threadPool_;
        int workerId_;
        boost::mutex lock_;
        boost::chrono::high_resolution_clock::time_point taskStartTime_;
    };

} } }
