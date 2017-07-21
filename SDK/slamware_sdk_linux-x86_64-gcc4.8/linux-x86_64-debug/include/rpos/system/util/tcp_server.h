/**
* tcp_server.h
* Generic Tcp Server
*
* Created By Tony Huang @ 2014-10-13
* Copyright (c) 2014 Shanghai SlamTec Co., Ltd.
*/

/*
* Usage
*
* class SomeTcpConnectionHandler : public TcpServer<SomeTcpConnectionHandler>::ITcpConnectionHandler
* {
*     // Implement pure virtual functions
* };
*
* class SomeTcpServer : public TcpServer<SomeTcpConnectionHandler>
* {
*     // Write ctor & dtor
* };
*
* boost::shared_ptr<SomeTcpServer> server(new SomeTcpServer(boost::asio::ip::tcp::endpoint(boost::asio::ip::v4(), 1314));
* server->start();
*/
#pragma once

#include <rpos/rpos_config.h>
#include <rpos/system/util/log.h>
#include <boost/noncopyable.hpp>
#include <boost/enable_shared_from_this.hpp>
#include <boost/asio.hpp>
#include <boost/thread.hpp>
#include <boost/atomic.hpp>
#include <boost/bind.hpp>
#include <list>
#include <string.h>

#define RPOS_SYSTEM_UTIL_TCPSERVER_DEFAULT_RX_BUFFER_SIZE 1024

namespace rpos {
    namespace system {
        namespace util {

            template <class TcpConnectionHandlerT, int RxBufferSize = RPOS_SYSTEM_UTIL_TCPSERVER_DEFAULT_RX_BUFFER_SIZE>
            class TcpServer : public boost::enable_shared_from_this<TcpServer<TcpConnectionHandlerT, RxBufferSize>>, private boost::noncopyable
            {
            public:
                class TcpConnection;

                class ITcpConnectionHandler
                {
                public:
                    ITcpConnectionHandler()
                    {
                    }

                    virtual ~ITcpConnectionHandler()
                    {
                    }

                    virtual void onConnectionStarting(boost::shared_ptr<TcpConnection> connection) = 0;
                    virtual void onSendError(boost::shared_ptr<TcpConnection> connection, const boost::system::error_code& ec) = 0;
                    virtual void onSendComplete(boost::shared_ptr<TcpConnection> connection) = 0;
                    virtual void onReceiveError(boost::shared_ptr<TcpConnection> connection, const boost::system::error_code& ec) = 0;
                    virtual void onReceiveComplete(boost::shared_ptr<TcpConnection> connection, const unsigned char* buffer, size_t readBytes) = 0;
                    virtual void onConnectionClosed(boost::shared_ptr<TcpConnection> connection) = 0;
                };

                class EmptyTcpConnectionHandler : public ITcpConnectionHandler
                {
                public:
                    EmptyTcpConnectionHandler()
                    {
                    }

                    virtual ~EmptyTcpConnectionHandler()
                    {
                    }

                    virtual void onConnectionStarting(boost::shared_ptr<TcpConnection> connection)
                    {
                    }

                    virtual void onSendError(boost::shared_ptr<TcpConnection> connection, const boost::system::error_code& ec)
                    {
                    }

                    virtual void onSendComplete(boost::shared_ptr<TcpConnection> connection)
                    {
                    }

                    virtual void onReceiveError(boost::shared_ptr<TcpConnection> connection, const boost::system::error_code& ec)
                    {
                    }

                    virtual void onReceiveComplete(boost::shared_ptr<TcpConnection> connection, const unsigned char* buffer, size_t readBytes)
                    {
                    }

                    virtual void onConnectionClosed(boost::shared_ptr<TcpConnection> connection)
                    {
                    }
                };

                class TcpConnection : public boost::enable_shared_from_this<TcpConnection>, private boost::noncopyable
                {
                public:
                    TcpConnection(boost::shared_ptr<TcpServer> server, boost::asio::io_service& io)
                        : server_(server)
                        , socket_(io)
                        , handler_(new TcpConnectionHandlerT)
                    {
                        sending_ = false;
                        receiving_ = false;
                    }

                    ~TcpConnection()
                    {}

                    boost::shared_ptr<TcpConnectionHandlerT> getHandler()
                    {
                        return handler_;
                    }

                    boost::asio::ip::tcp::socket& socket()
                    {
                        return socket_;
                    }

                    void start()
                    {
                        handler_->onConnectionStarting(this->shared_from_this());
                        startReceive_();
                    }

                    void send(const unsigned char* buffer, size_t size)
                    {
                        if (!size)
                            return;

                        {
                            boost::lock_guard<boost::mutex> guard(sendLock_);

                            size_t offset = txBuffer_.size();
                            txBuffer_.resize(offset + size);
                            memcpy(&txBuffer_[offset], buffer, size);
                        }

                        startTransmit_();
                    }

                    void send(const std::vector<unsigned char>& buffer)
                    {
                        if (buffer.empty())
                            return;

                        bool sent = false;

                        {
                            boost::lock_guard<boost::mutex> guard(sendLock_);

                            if (txBuffer_.empty())
                            {
                                txBuffer_ = buffer;
                                sent = true;
                            }
                        }

                        if (!sent)
                            send(&buffer[0], buffer.size());
                        else
                            startTransmit_();
                    }

                    void close()
                    {
                        close_();
                    }

                    boost::shared_ptr<TcpServer> server()
                    {
                        return server_.lock();
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
                        std::swap(transferingBuffer_, txBuffer_);
                        nativeTxBuffer_ = &(transferingBuffer_)[0];
                        txBuffer_.clear();

                        doTransmit_();
                    }

                    void doTransmit_()
                    {
                        socket_.async_write_some(
                            boost::asio::buffer(nativeTxBuffer_ + transferredInBuffer_, transferingBuffer_.size() - transferredInBuffer_),
                            boost::bind(&TcpConnection::onTransmitComplete_, this->shared_from_this(), _1, _2)
                            );
                    }

                    void onTransmitComplete_(const boost::system::error_code& ec, size_t written)
                    {
                        if (ec)
                        {
                            handler_->onSendError(this->shared_from_this(), ec);
                            close_();
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
                            boost::bind(&TcpConnection::onReceiveComplete_, this->shared_from_this(), _1, _2)
                            );

                        startServerWorker_();
                    }

                    void onReceiveComplete_(const boost::system::error_code& ec, size_t readBytes)
                    {
                        if (ec)
                        {
                            handler_->onReceiveError(this->shared_from_this(), ec);
                            close_();
                            return;
                        }

                        if (readBytes)
                        {
                            handler_->onReceiveComplete(this->shared_from_this(), rxBlock_, readBytes);
                        }

                        receive_();
                    }

                private:
                    void startServerWorker_()
                    {
                        boost::shared_ptr<TcpServer> server = server_.lock();

                        if (server)
                        {
                            server->startWorker_();
                        }
                    }

                    void close_()
                    {
                        if (socket_.is_open())
                        {
                            boost::system::error_code ec;
                            socket_.shutdown(boost::asio::ip::tcp::socket::shutdown_both, ec);

                            if (ec)
                            {
                                socket_.close(ec);
                            }
                        }

                        boost::shared_ptr<TcpServer> server = server_.lock();

                        if (server)
                        {
                            server->handleConnectionClose_(this->shared_from_this());
                        }

                        handler_->onConnectionClosed(this->shared_from_this());
                    }

                private:
                    boost::weak_ptr<TcpServer> server_;
                    boost::asio::ip::tcp::socket socket_;

                    boost::shared_ptr<TcpConnectionHandlerT> handler_;

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

            public:
                TcpServer(const boost::asio::ip::tcp::endpoint& listenPoint)
                    : io_()
                    , acceptor_(io_, listenPoint)
                {
                }

                virtual ~TcpServer()
                {
                    acceptor_.close();

                    if (ioThread_ && ioThread_->joinable())
                    {
                        ioThread_->join();
                    }
                }

                void start()
                {
                    startAccept_();
                }

            private:
                void startWorker_()
                {
                    if (ioThread_ && ioThread_->joinable())
                        return;

                    boost::function<void()> threadProc = boost::bind(&TcpServer::worker_, this->shared_from_this());
                    ioThread_ = boost::shared_ptr<boost::thread>(new boost::thread(threadProc));
                }

                void handleConnectionClose_(boost::shared_ptr<TcpConnection> conn)
                {
                    boost::lock_guard<boost::mutex> guard(lock_);
                    connections_.remove(conn);
                }

            private:
                void startAccept_()
                {
                    boost::shared_ptr<TcpConnection> connection = boost::shared_ptr<TcpConnection>(new TcpConnection(this->shared_from_this(), io_));
                    acceptor_.async_accept(connection->socket(), boost::bind(&TcpServer::handleConnection_, this, connection, _1));
                    startWorker_();
                }

                void handleConnection_(boost::shared_ptr<TcpConnection> conn, const boost::system::error_code& ec)
                {
                    if (!ec)
                    {
                        {
                            boost::lock_guard<boost::mutex> guard(lock_);
                            connections_.push_back(conn);
                        }
                        conn->start();
                    }
                    else
                    {
                        warn_out("rpos.system.util.tcp_server", "Failed to accept connection: %s", ec.message().c_str());
                    }

                    startAccept_();
                }

                void worker_()
                {
                    io_.run();
                }

            protected:
                std::list<boost::shared_ptr<TcpConnection> > getConnections_()
                {
                    boost::lock_guard<boost::mutex> guard(lock_);
                    return connections_;
                }

            private:
                boost::asio::io_service io_;
                boost::asio::ip::tcp::acceptor acceptor_;
                boost::shared_ptr<boost::thread> ioThread_;

                boost::mutex lock_;
                std::list<boost::shared_ptr<TcpConnection> > connections_;
            };

        }
    }
}
