/*
 * debug_server.h
 * Debug server
 *
 * Created By Tony Huang (cnwzhjs@gmail.com)at 2014-12-11
 * Copyright 2014 (c) Shanghai Slamtec Co., Ltd.
 */

#pragma once

#include <rpos/rpos_config.h>
#include <rpos/system/object_handle.h>
#include <rpos/system/types.h>
#include <rpos/system/serialization/json_serialization.h>
#include <boost/noncopyable.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/function.hpp>
#include <boost/thread.hpp>
#include <string>
#include <vector>

namespace rpos {
    namespace system {
        namespace util {

            namespace detail {
                
                class DebugServerImpl;

            }

            class BaseDebugCommand : private boost::noncopyable
            {
            protected:
                BaseDebugCommand(const std::string& key);
                BaseDebugCommand(const std::string& key, const std::string& name, const std::string& description);

            public:
                virtual ~BaseDebugCommand();

            public:
                const std::string& key() const;
                const std::string& name() const;
                const std::string& description() const;

            public:
                virtual Json::Value execute(const Json::Value&) const = 0;

            private:
                std::string key_;
                std::string name_;
                std::string description_;
            };

            template <class RequestT, class ResponseT>
            class DebugCommand : public BaseDebugCommand
            {
            public:
                DebugCommand(const std::string& key, boost::function<ResponseT(const RequestT&)> handler)
                    : BaseDebugCommand(key)
                    , handler_(handler)
                {
                }

                DebugCommand(const std::string& key, const std::string& name, const std::string& description, boost::function<ResponseT(const RequestT&)> handler)
                    : BaseDebugCommand(key, name, description)
                    , handler_(handler)
                {
                }

                virtual ~DebugCommand()
                {
                }

            public:
                virtual Json::Value execute(const Json::Value& request) const
                {
                    return rpos::system::serialization::json::serialize(handler_(rpos::system::serialization::json::deserialize<RequestT>(request)));
                }

            private:
                boost::function<ResponseT(const RequestT&)> handler_;
            };

            class BaseInspectValue : private boost::noncopyable
            {
            protected:
                BaseInspectValue(const std::string& key);
                BaseInspectValue(const std::string& key, const std::string& name, const std::string& description);

            public:
                virtual ~BaseInspectValue();

            public:
                const std::string& key() const;
                const std::string& name() const;
                const std::string& description() const;

                virtual Json::Value getValue() const = 0;
                virtual void setValue(const Json::Value&) = 0;

            private:
                std::string key_;
                std::string name_;
                std::string description_;
            };

            template<class T>
            class InspectValue : public BaseInspectValue
            {
            public:
                InspectValue(const std::string& key, T& v)
                    : BaseInspectValue(key)
                    , readonly_(false)
                    , target_(v)
                {
                }

                InspectValue(const std::string& key, const T& v)
                    : BaseInspectValue(key)
                    , readonly_(true)
                    , target_(const_cast<T&>(v))
                {
                }

                InspectValue(const std::string& key, const std::string& name, const std::string& description, T& v)
                    : BaseInspectValue(key, name, description)
                    , readonly_(false)
                    , target_(v)
                {
                }

                InspectValue(const std::string& key, const std::string& name, const std::string& description, const T& v)
                    : BaseInspectValue(key, name, description)
                    , readonly_(true)
                    , target_(const_cast<T&>(v))
                {
                }

                ~InspectValue()
                {}

            public:
                virtual Json::Value getValue() const
                {
                    return rpos::system::serialization::json::serialize(target_);
                }

                virtual void setValue(const Json::Value& v)
                {
                    if (readonly_)
                        return;

                    target_ = rpos::system::serialization::json::deserialize<T>(v);
                }

            private:
                bool readonly_;
                T& target_;
            };

            template<class T, class LockT>
            class InspectValueWithLock : public BaseInspectValue
            {
            public:
                InspectValueWithLock(const std::string& key, T& v, LockT& lock)
                    : BaseInspectValue(key)
                    , readonly_(false)
                    , target_(v)
                    , lock_(lock)
                {
                }

                InspectValueWithLock(const std::string& key, const T& v, LockT& lock)
                    : BaseInspectValue(key)
                    , readonly_(true)
                    , target_(const_cast<T&>(v))
                    , lock_(lock)
                {
                }

                InspectValueWithLock(const std::string& key, const std::string& name, const std::string& description, T& v, LockT& lock)
                    : BaseInspectValue(key, name, description)
                    , readonly_(false)
                    , target_(v)
                    , lock_(lock)
                {
                }

                InspectValueWithLock(const std::string& key, const std::string& name, const std::string& description, const T& v, LockT& lock)
                    : BaseInspectValue(key, name, description)
                    , readonly_(true)
                    , target_(const_cast<T&>(v))
                    , lock_(lock)
                {
                }

                ~InspectValueWithLock()
                {}

            public:
                virtual Json::Value getValue() const
                {
                    boost::lock_guard<LockT> guard(lock_);

                    return rpos::system::serialization::json::serialize(target_);
                }

                virtual void setValue(const Json::Value& v)
                {
                    if (readonly_)
                        return;

                    boost::lock_guard<LockT> guard(lock_);

                    target_ = rpos::system::serialization::json::deserialize<T>(v);
                }

            private:
                bool readonly_;
                T& target_;
                LockT& lock_;
            };

            class DebugServer : public ObjectHandle < DebugServer, detail::DebugServerImpl >
            {
            public:
                RPOS_OBJECT_CTORS(DebugServer);
                DebugServer(int port);
                ~DebugServer();

            public:
                void start();

            public:
                static DebugServer defaultServer();

            public:
                void registerBaseDebugCommand(boost::shared_ptr<BaseDebugCommand> command);
                void registerBaseInspectValue(boost::shared_ptr<BaseInspectValue> inspectValue);

                template < class RequestT, class ResponseT >
                void registerDebugCommand(const std::string& key, boost::function<ResponseT(const RequestT&)> handler)
                {
                    registerBaseDebugCommand(boost::shared_ptr<BaseDebugCommand>(new DebugCommand<RequestT, ResponseT>(key, handler)));
                }

                template < class T >
                void registerInspectValue(const std::string& key, T& v)
                {
                    registerBaseInspectValue(boost::shared_ptr<BaseInspectValue>(new InspectValue<T>(key, v)));
                }

                template < class T >
                void registerInspectValue(const std::string& key, const T& v)
                {
                    registerBaseInspectValue(boost::shared_ptr<BaseInspectValue>(new InspectValue<T>(key, v)));
                }

                template < class T, class LockT >
                void registerInspectValueWithLock(const std::string& key, T& v, LockT& lock)
                {
                    registerBaseInspectValue(boost::shared_ptr<BaseInspectValue>(new InspectValueWithLock<T, LockT>(key, v, lock)));
                }

                template < class T, class LockT >
                void registerInspectValueWithLock(const std::string& key, const T& v, LockT& lock)
                {
                    registerBaseInspectValue(boost::shared_ptr<BaseInspectValue>(new InspectValueWithLock<T, LockT>(key, v, lock)));
                }

            public:
                void enableUi(const std::string& uiResPath);
            };

        }
    }
}
