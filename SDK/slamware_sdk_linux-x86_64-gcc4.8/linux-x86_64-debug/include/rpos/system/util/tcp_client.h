/**
* tcp_client.h
*
* Created By Tony Huang @ 2015-3-18
* Copyright (c) 2015 Shanghai SlamTec Co., Ltd.
*/

/**
* Usage
*
* class SomeTcpClientHandler : public TcpClient<SomeTcpClientHandler>::ITcpClientHandler
* {
*     // Implement pure virutal functions
* };
*
* class SomeTcpClient : public TcpClient<SomeTcpClientHandler> {
*     // Write ctor & dtor
* };
*
* boost::shared_ptr<SomeTcpClient> client(new SomeTcpClient());
* client->connectTo(someHost, somePort);
* client->start();
*/
#pragma once

#include <rpos/rpos_config.h>
#include <rpos/system/util/log.h>
#include <rpos/system/util/string_utils.h>
#include <rpos/system/target_info.h>
#include <rpos/system/this_thread.h>
#include <boost/noncopyable.hpp>
#include <boost/enable_shared_from_this.hpp>
#include <boost/asio.hpp>
#include <boost/thread.hpp>
#include <boost/atomic.hpp>
#include <boost/bind.hpp>
#include <list>
#include <vector>
#include <string.h>

#define RPOS_SYSTEM_UTIL_TCPCLIENT_DEFAULT_RX_BUFFER_SIZE 1024

namespace rpos { namespace system { namespace util {

    enum TcpClientStatus {
        TcpClientStatusIdle,
        TcpClientStatusResolvingHost,
        TcpClientStatusConnecting,
        TcpClientStatusConnected,
        TcpClientStatusClosing,
        TcpClientStatusClosed,
        TcpClientStatusError = -1
    };

    template <class TcpClientHandlerT, int RxBufferSize = RPOS_SYSTEM_UTIL_TCPCLIENT_DEFAULT_RX_BUFFER_SIZE>
    class TcpClient : public boost::enable_shared_from_this<TcpClient<TcpClientHandlerT, RxBufferSize>>, private boost::noncopyable {
    public:
        typedef boost::shared_ptr<TcpClient<TcpClientHandlerT, RxBufferSize>> Pointer;

    public:
        class ITcpClientHandler {
        public:
            virtual void onHostResolved(Pointer client, boost::asio::ip::basic_resolver_iterator<boost::asio::ip::tcp> begin, boost::asio::ip::basic_resolver_iterator<boost::asio::ip::tcp> end) = 0;
            virtual void onHostResolveFailure(Pointer client, const boost::system::error_code& ec) = 0;
            virtual void onConnected(Pointer client, boost::asio::ip::basic_resolver_iterator<boost::asio::ip::tcp> connectedEntry) = 0;
            virtual void onConnectionFailure(Pointer client, const boost::system::error_code& ec) = 0;
            virtual void onSendError(Pointer client, const boost::system::error_code& ec) = 0;
            virtual void onSendComplete(Pointer client) = 0;
            virtual void onReceiveError(Pointer client, const boost::system::error_code& ec) = 0;
            virtual void onReceiveComplete(Pointer client, const unsigned char* buffer, size_t readBytes) = 0;
            virtual void onConnectionClosed(Pointer client) = 0;
        };

        class EmptyTcpClientHandler : public ITcpClientHandler {
        public:
            virtual void onHostResolved(Pointer client, boost::asio::ip::basic_resolver_iterator<boost::asio::ip::tcp> begin, boost::asio::ip::basic_resolver_iterator<boost::asio::ip::tcp> end)
            {}

            virtual void onHostResolveFailure(Pointer client, const boost::system::error_code& ec)
            {}

            virtual void onConnected(Pointer client, boost::asio::ip::basic_resolver_iterator<boost::asio::ip::tcp> connectedEntry)
            {}

            virtual void onConnectionFailure(Pointer client, const boost::system::error_code& ec)
            {}

            virtual void onSendError(Pointer client, const boost::system::error_code& ec)
            {}

            virtual void onSendComplete(Pointer client)
            {}

            virtual void onReceiveError(Pointer client, const boost::system::error_code& ec)
            {}

            virtual void onReceiveComplete(Pointer client, const unsigned char* buffer, size_t readBytes)
            {}

            virtual void onConnectionClosed(Pointer client)
            {}
        };

    public:
        TcpClient()
            : io_()
            , resolver_(io_)
            , socket_(io_)
            , status_(TcpClientStatusIdle)
            , handler_(new TcpClientHandlerT())
        {
            sending_ = false;
            receiving_ = false;
        }

        virtual ~TcpClient()
        {
            stop();

            if (ioThread_.joinable())
                ioThread_.join();

            delete handler_;
            handler_ = nullptr;
        }

    public:
        void start()
        {
            {
                boost::lock_guard<boost::mutex> guard(lock_);

                if (ioThread_.joinable())
                    return;
            }

            ioThread_ = boost::move(boost::thread(boost::bind(&TcpClient::worker_, this->shared_from_this())));
        }

        void stop()
        {
            {
                boost::lock_guard<boost::mutex> guard(lock_);

                if (!socket_.is_open())
                    return;
            }

            boost::system::error_code ec;
            socket_.shutdown(boost::asio::socket_base::shutdown_type::shutdown_both, ec);
            socket_.close(ec);
        }

        void send(const std::vector<unsigned char>& buffer)
        {
            if (buffer.empty())
                return;

            {
                boost::lock_guard<boost::mutex> guard(sendLock_);

                if (txBuffer_.empty())
                {
                    txBuffer_ = buffer;
                }
                else
                {
                    size_t offset = txBuffer_.size();
                    txBuffer_.resize(offset + buffer.size());
                    memcpy(&txBuffer_[offset], &buffer[0], buffer.size());
                }
            }
            startTransmit_();
        }

        void close()
        {
            stop();
        }

        void stopWorkerThread()
        {
            if (ioThread_.joinable())
                ioThread_.join();
        }

        boost::asio::ip::tcp::socket& socket()
        {
            return socket_;
        }

    private:
        typedef boost::asio::ip::basic_resolver<boost::asio::ip::tcp> resolver;

    public:
        bool connect(const std::string& host, int port)
        {
            {
                boost::lock_guard<boost::mutex> guard(lock_);

                if (status_ != TcpClientStatusIdle)
                    return false;

                status_ = TcpClientStatusResolvingHost;
            }

            std::string portString = rpos::system::util::to_string(port);

#ifdef RPOS_TARGET_ANDROID
            resolver::query query(host, portString);
#else
            resolver::query query(host, portString, boost::asio::ip::resolver_query_base::all_matching);
#endif

            resolver_.async_resolve(query, boost::bind(&TcpClient::onAddressResolved_, this->shared_from_this(), _1, _2));

            return true;
        }

    private:
        void onAddressResolved_(const boost::system::error_code& ec, resolver::iterator it)
        {
            if (ec)
            {
                handler_->onHostResolveFailure(this->shared_from_this(), ec);
                boost::lock_guard<boost::mutex> guard(lock_);
                status_ = TcpClientStatusError;
            }
            else
            {
                handler_->onHostResolved(this->shared_from_this(), it, resolver::iterator());
                boost::lock_guard<boost::mutex> guard(lock_);
                status_ = TcpClientStatusConnecting;
                resolveResult_ = it;

                connect_();
            }
        }

        void connect_()
        {
            resolver::endpoint_type endpoint = resolveResult_->endpoint();
            socket_.async_connect(endpoint, boost::bind(&TcpClient::onConnect_, this->shared_from_this(), _1));
        }

        void onConnect_(const boost::system::error_code& ec)
        {
            if (ec)
            {
                resolveResult_ ++;

                resolver::iterator end;
                if (resolveResult_ == end)
                {
                    handler_->onConnectionFailure(this->shared_from_this(), ec);

                    boost::lock_guard<boost::mutex> guard(lock_);
                    status_ = TcpClientStatusError;
                }
                else
                {
                    boost::system::error_code ec;
                    // try next endpoint
                    socket_.shutdown(boost::asio::socket_base::shutdown_type::shutdown_both, ec);
                    socket_.close(ec);
                    connect_();
                }
            }
            else
            {
                handler_->onConnected(this->shared_from_this(), resolveResult_);

                boost::lock_guard<boost::mutex> guard(lock_);
                status_ = TcpClientStatusConnected;

                startReceive_();
            }
        }

    private:
        // Send
        void startTransmit_()
        {
            {
                boost::lock_guard<boost::mutex> guard(sendLock_);

                if (sending_)
                    return;

                if (!txBuffer_.size())
                    return;

                sending_ = true;
            }

            transmit_();
        }

        void transmit_()
        {
            boost::lock_guard<boost::mutex> guard(sendLock_);

            transferredInBuffer_ = 0;
            transferingBuffer_ = txBuffer_;
            nativeTxBuffer_ = &(transferingBuffer_)[0];
            txBuffer_.clear();

            doTransmit_();
        }

        void doTransmit_()
        {
            socket_.async_write_some(
                boost::asio::buffer(nativeTxBuffer_ + transferredInBuffer_, transferingBuffer_.size() - transferredInBuffer_),
                boost::bind(&TcpClient::onTransmitComplete_, this->shared_from_this(), _1, _2)
                );
        }

        void onTransmitComplete_(const boost::system::error_code& ec, size_t written)
        {
            if (ec)
            {
                handler_->onSendError(this->shared_from_this(), ec);
                close();
                return;
            }

            transferredInBuffer_ += written;

            if (transferredInBuffer_ == transferingBuffer_.size())
            {
                {
                    boost::lock_guard<boost::mutex> guard(sendLock_);
                    sending_ = false;
                }

                if (txBuffer_.size())
                {
                    return startTransmit_();
                }

                handler_->onSendComplete(this->shared_from_this());

                startTransmit_();
            }
            else
            {
                doTransmit_();
            }
        }

    private:
        // Receive
        void startReceive_()
        {
            {
                boost::lock_guard<boost::mutex> guard(rxLock_);

                if (receiving_)
                    return;

                receiving_ = true;
            }

            receive_();
        }

        void receive_()
        {
            socket_.async_read_some(
                boost::asio::buffer(rxBlock_, RxBufferSize),
                boost::bind(&TcpClient::onReceiveComplete_, this->shared_from_this(), _1, _2)
                );
        }

        void onReceiveComplete_(const boost::system::error_code& ec, size_t readBytes)
        {
            if (ec)
            {
                handler_->onReceiveError(this->shared_from_this(), ec);
                close();
                return;
            }

            if (readBytes)
            {
                handler_->onReceiveComplete(this->shared_from_this(), rxBlock_, readBytes);
            }

            receive_();
        }

    private:
        void worker_()
        {
            rpos::system::this_thread::setCurrentThreadName("TcpClient");
            io_.run();
        }

    private:
        boost::asio::io_service io_;
        boost::thread ioThread_;
        resolver resolver_;
        boost::asio::ip::tcp::socket socket_;
        resolver::iterator resolveResult_;

        boost::mutex lock_;

        TcpClientStatus status_;
        TcpClientHandlerT* handler_;

        boost::atomic_bool sending_;
        boost::atomic_bool receiving_;

        boost::mutex sendLock_;
        std::vector<unsigned char> txBuffer_;
        std::vector<unsigned char> transferingBuffer_;
        unsigned char* nativeTxBuffer_;
        size_t transferredInBuffer_;

        unsigned char rxBlock_[RxBufferSize];
        boost::mutex rxLock_;
    };

} } }
