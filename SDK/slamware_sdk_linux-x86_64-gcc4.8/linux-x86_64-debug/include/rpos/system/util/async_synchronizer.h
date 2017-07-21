/**
* async_synchronizer.h
* Asynchronious operation synchronizer
*
* Created By Tony Huang @ 2015-4-1
* Copyright 2015 (c) Shanghai Slamtec Co., Ltd.
*/

#pragma once

#include <boost/enable_shared_from_this.hpp>
#include <boost/thread/mutex.hpp>
#include <boost/thread/condition_variable.hpp>
#include <boost/system/error_code.hpp>
#include <boost/function.hpp>
#include <boost/bind.hpp>

#include "void_type.h"

namespace rpos { namespace system { namespace util {

    template < class T, class ErrorHandlerT >
    class AsyncSynchronizer : public boost::enable_shared_from_this<AsyncSynchronizer<T, ErrorHandlerT>>, private boost::noncopyable {
    public:
        AsyncSynchronizer()
            : done_(false)
        {}

        ~AsyncSynchronizer()
        {}

    public:
        boost::function<void(const boost::system::error_code&, T)> getCallback()
        {
            return boost::bind(&AsyncSynchronizer::onCallback, this->shared_from_this(), _1, _2);
        }

        T waitForResult(int timeoutInMs)
        {
            boost::chrono::milliseconds timeout(timeoutInMs);

            boost::unique_lock<boost::mutex> lock(lock_);
            
            if (!done_ && cond_.wait_for(lock, timeout) == boost::cv_status::timeout)
            {
                ErrorHandlerT::timeout();
            }
            //workaround on windows. sometimes wait_for not return timeout, but it actually timeout
            if (!done_)
            {
                ErrorHandlerT::timeout();
            }

            ErrorHandlerT::handle(ec_);

            return result_;
        }

    private:
        static void onCallback(boost::weak_ptr<AsyncSynchronizer> weakSelf, const boost::system::error_code& ec, T result)
        {
            auto self = weakSelf.lock();

            if (!self)
                return;
            {
                boost::lock_guard<boost::mutex> guard(self->lock_);

                if (self->done_)
                {
                    return;
                }

                self->done_ = true;
                self->ec_ = ec;
                self->result_ = result;

            }

            self->cond_.notify_all();
        }

    private:
        T result_;
        bool done_;
        boost::system::error_code ec_;
        boost::mutex lock_;
        boost::condition_variable cond_;
    };

    template <class ErrorHandlerT>
    class AsyncSynchronizer<RPOS_VOID, ErrorHandlerT> : public boost::enable_shared_from_this<AsyncSynchronizer<RPOS_VOID, ErrorHandlerT> >, private boost::noncopyable{
    public:
        AsyncSynchronizer()
            : done_(false)
        {}

        ~AsyncSynchronizer()
        {}

    public:
        boost::function<void(const boost::system::error_code&)> getCallback()
        {
            return boost::bind(&AsyncSynchronizer::onCallback, this->shared_from_this(), _1);
        }

        void waitForResult(int timeoutInMs)
        {
            boost::chrono::milliseconds timeout(timeoutInMs);

            boost::unique_lock<boost::mutex> lock(lock_);
            if (!done_ && cond_.wait_for(lock, timeout) == boost::cv_status::timeout)
            {
                ErrorHandlerT::timeout();
            }
            //workaround on windows. sometimes wait_for not return timeout, but it actually timeout
            if (!done_)
            {
                ErrorHandlerT::timeout();
            }

            ErrorHandlerT::handle(ec_);
        }

    private:
        static void onCallback(boost::weak_ptr<AsyncSynchronizer> weakSelf, const boost::system::error_code& ec)
        {
            auto self = weakSelf.lock();

            if (!self)
                return;
            {
                boost::lock_guard<boost::mutex> guard(self->lock_);

                if (self->done_)
                {
                    return;
                }

                self->done_ = true;
                self->ec_ = ec;

            }
            self->cond_.notify_all();
        }

    private:
        bool done_;
        boost::system::error_code ec_;
        boost::mutex lock_;
        boost::condition_variable cond_;
    };

    namespace detail {

        template < class T >
        struct await_types_
        {
            typedef boost::function<void(const boost::system::error_code&, T)> callback;
            typedef boost::function<void(callback)> operation;
            typedef T return_type;
        };

        template <>
        struct await_types_ < RPOS_VOID >
        {
            typedef boost::function<void(const boost::system::error_code&)> callback;
            typedef boost::function<void(callback)> operation;
            typedef void return_type;
        };

    }

    template < class T, class ErrorHandlerT >
    typename detail::await_types_<T>::return_type awaitFor(int timeoutInMs, typename detail::await_types_<T>::operation operation)
    {
        boost::shared_ptr< AsyncSynchronizer<T, ErrorHandlerT> > sync(new AsyncSynchronizer<T, ErrorHandlerT>);
        operation(sync->getCallback());
        return sync->waitForResult(timeoutInMs);
    }

} } }
