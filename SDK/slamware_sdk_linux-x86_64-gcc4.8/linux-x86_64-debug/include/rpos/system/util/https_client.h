/**
* https_client.h
*
* Created By Gabriel He @ 2016-02-02
* Copyright (c) 2016 Shanghai SlamTec Co., Ltd.
*/

#pragma once

#include <rpos/system/types.h>
#include <rpos/system/util/string_utils.h>

#include <boost/thread/condition_variable.hpp>
#include <boost/enable_shared_from_this.hpp>
#include <boost/make_shared.hpp>
#include <boost/noncopyable.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/thread.hpp>
#include <boost/unordered_map.hpp>

#include <curl/curl.h>

#include <map>
#include <string>
#include <vector>

#define RPOS_LIB_NAME rpos_deps_libcurl
#define RPOS_AUTO_LINK_NO_VERSION
#	include <rpos/system/util/auto_link.h>
#undef RPOS_AUTO_LINK_NO_VERSION
#undef RPOS_LIB_NAME

#define SLEEP_MILLISECOND_100 100

namespace rpos { namespace system { namespace util {

    RPOS_CORE_API class HttpsInit : public boost::noncopyable
    {
    public:
        RPOS_CORE_API static void init();
        RPOS_CORE_API static void destroy();

    private:
        HttpsInit();
        ~HttpsInit();

        static HttpsInit* init_;
        static boost::mutex lock_;
    };

    template <class HttpsClientHandlerT>
    class HttpsClient : public boost::enable_shared_from_this<HttpsClient<HttpsClientHandlerT>>, private boost::noncopyable
    {
    public:
        typedef boost::shared_ptr<HttpsClient<HttpsClientHandlerT>> Pointer;

    public:
        HttpsClient()
            : done_(false)
        {
            HttpsInit::init();
        }

        virtual ~HttpsClient()
        {
            stop();
        }

        void start()
        {
            curl_m_ = curl_multi_init();

            if (selectThread_.joinable())
            {
                return;
            }

            selectThread_ = boost::move(boost::thread(boost::bind(&HttpsClient::worker_, this->shared_from_this())));
        }

        void stop()
        {
            {
                boost::lock_guard<boost::mutex> guard(lock_);

                if(done_)
                {
                    return;
                }
                else
                {
                    done_ = true;
                }
            }

            if (selectThread_.joinable())
            {
                selectThread_.join();
            }

            curl_multi_cleanup(curl_m_);
        }

        void send(unsigned int requestId, std::string& method, std::string& uri, std::vector<std::string>& headers, std::vector<system::types::_u8>& body)
        {
            auto context = new CurlContext();

            context->received_size = 0;
            context->request_id = requestId;

            context->request.method_ = method;
            context->request.uri_ = uri;

            struct curl_slist* curl_header = NULL;
            for(auto it = headers.begin(); it != headers.end(); it++)
            {
                std::string header = (*it);
                curl_header = curl_slist_append(curl_header, header.c_str());
            }
            curl_header = curl_slist_append(curl_header, "Expect:");
            context->request.headers_ = curl_header;


            context->request.body_ = body;

            context->response.status_.clear();
            context->response.headers_.clear();
            context->response.body_.clear();

            context->pointer = this->shared_from_this();
            context->handler_ = new HttpsClientHandlerT();

            CURL* curl = curl_easy_handler_get(context, 500);
            {
                boost::lock_guard<boost::mutex> guard(contexts_lock_);
                contexts_[curl] = context;
            }
            curl_multi_add_handle(curl_m_, curl);
        }

    private:
        class Request
        {
        public:
            Request()
            {}

            ~Request()
            {
                curl_slist_free_all(headers_);
            }

            std::string method_;
            std::string uri_;
            struct curl_slist* headers_;
            std::vector<system::types::_u8> body_;
        };

        class Response
        {
        public:
            Response()
            {}

            std::string status_;
            std::vector<std::string> headers_;
            std::vector<system::types::_u8> body_;
        };

        class CurlContext
        {
        public:
            CurlContext()
            {}

            ~CurlContext()
            {
                delete handler_;
            }

            unsigned int received_size;
            unsigned int request_id;
            std::string header_buffer;

            Request request;
            Response response;

            Pointer pointer;
            HttpsClientHandlerT* handler_;
        };

    private:
        void worker_()
        {
            CURLMsg *msg;
            int msgs_left;

            int running_handles = 0;

            while(true)
            {
                boost::this_thread::sleep(boost::posix_time::milliseconds(SLEEP_MILLISECOND_100));
                {
                    boost::lock_guard<boost::mutex> guard(lock_);

                    if (done_)
                    {
                        break;
                    }
                }

                while(CURLM_CALL_MULTI_PERFORM == curl_multi_perform(curl_m_, &running_handles))
                {
                    boost::this_thread::sleep(boost::posix_time::milliseconds(SLEEP_MILLISECOND_100));
                }

                while(running_handles)
                {
                    if(my_curl_multi_select_(curl_m_) == -1)
                    {
                        continue;
                    }
                    else
                    {
                        while(CURLM_CALL_MULTI_PERFORM == curl_multi_perform(curl_m_, &running_handles))
                        {
                            boost::this_thread::sleep(boost::posix_time::milliseconds(SLEEP_MILLISECOND_100));
                        }
                    }
                }

                while((msg = curl_multi_info_read(curl_m_, &msgs_left)))
                {
                    if(CURLMSG_DONE == msg->msg)
                    {
                        curl_multi_remove_handle(curl_m_, msg->easy_handle);
                        {
                            {
                                boost::lock_guard<boost::mutex> guard(contexts_lock_);
                                CurlContext* context = contexts_[msg->easy_handle];

                                auto response = context->response;
                                context->handler_->receiveComplete(context->pointer, context->request_id, response.status_, response.headers_, response.body_);

                                contexts_.erase(msg->easy_handle);

                                delete context;
                            }
                        }
                        curl_easy_cleanup(msg->easy_handle);
                    }
                }
            }
        }

        int my_curl_multi_select_(CURL* curl_m)
        {
            int ret = 0;

            struct timeval timeout_tv;
            fd_set fd_read;
            fd_set fd_write;
            fd_set fd_except;
            int max_fd = -1;

            FD_ZERO(&fd_read);
            FD_ZERO(&fd_write);
            FD_ZERO(&fd_except);

            timeout_tv.tv_sec = 1;
            timeout_tv.tv_usec = 0;

            curl_multi_fdset(curl_m, &fd_read, &fd_write, &fd_except, &max_fd);

            if(max_fd == -1)
            {
                return -1;
            }

            int ret_code = ::select(max_fd + 1, &fd_read, &fd_write, &fd_except, &timeout_tv);
            switch(ret_code)
            {
            case -1:
                ret = -1;
                break;
            case 0:
            default:
                ret = 0;
                break;
            }

            return ret;
        }

        static size_t curl_writer(void *buffer, size_t size, size_t count, void *stream)
        {
            std::string *pStream = static_cast<std::string *>(stream);
            pStream->append(static_cast<char *>(buffer), size * count);
            return size * count;
        }

        static size_t curl_head_writer(void *buffer, size_t size, size_t count, void *pUser)
        {
            int rs;

            auto context = (CurlContext*)pUser;


            std::string str;

            rs = curl_writer(buffer, size, count, &str);

            context->header_buffer.insert(context->header_buffer.end(), str.begin(), str.end());

            std::string sEnd = "\r\n";

            auto last = context->header_buffer.find(sEnd+sEnd);
            if (last == (context->header_buffer.length()-4))
            {
                unsigned int p;

                p = context->header_buffer.find(sEnd);

                context->response.status_ = context->header_buffer.substr(0, p);
                context->header_buffer.erase(0, p+2);

                while((p=context->header_buffer.find(sEnd)) > 0)
                {
                    auto str = context->header_buffer.substr(0, p);

                    context->response.headers_.push_back(str);

                    context->header_buffer.erase(0, p+2);
                }
            }
            return rs;
        }

        static size_t curl_body_writer(void *buffer, size_t size, size_t count, void *pUser)
        {
            int rs;
            auto context = (CurlContext*)pUser;

            std::string str;
            rs = curl_writer(buffer, size, count, &str);
            if (rs > 0)
            {
                context->received_size += rs;
                context->response.body_.insert(context->response.body_.end(), str.begin(), str.end());
            }

            return rs;
        }

        CURL* curl_easy_handler_get(
            void *pUser,
            unsigned int uiTimeout)
        {
            auto context = (CurlContext*)pUser;

            CURL *curl = curl_easy_init();

            curl_easy_setopt(curl, CURLOPT_VERBOSE, 1L);    // Debug

            curl_easy_setopt(curl, CURLOPT_SSL_VERIFYPEER, 0L); // Do not verify certificate and host

            auto& method     = context->request.method_;
            auto& url        = context->request.uri_;
            auto headers    = context->request.headers_;
            auto& body       = context->request.body_;

            // set method
            curl_easy_setopt(curl, CURLOPT_CUSTOMREQUEST, method.c_str());

            //set url
            curl_easy_setopt(curl, CURLOPT_URL, url.c_str());
            curl_easy_setopt(curl, CURLOPT_NOSIGNAL, 1);

            //set header
            curl_easy_setopt(curl, CURLOPT_HTTPHEADER, headers);

            //set body
            if (body.size() > 0)
            {
                auto body_size = body.size();
                curl_easy_setopt(curl, CURLOPT_POST, 1L);
                curl_easy_setopt(curl, CURLOPT_POSTFIELDSIZE, body_size);
                curl_easy_setopt(curl, CURLOPT_COPYPOSTFIELDS, &body[0]);
            }

            if(uiTimeout > 0)
            {
                curl_easy_setopt(curl, CURLOPT_TIMEOUT, uiTimeout);
            }

            curl_easy_setopt(curl, CURLOPT_HEADERFUNCTION, curl_head_writer);
            curl_easy_setopt(curl, CURLOPT_HEADERDATA, pUser);

            curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION, curl_body_writer);
            curl_easy_setopt(curl, CURLOPT_WRITEDATA, pUser);

            return curl;
        }

    protected:
        CURLM *curl_m_;

    private:
        boost::thread selectThread_;
        boost::mutex lock_;
        boost::mutex contexts_lock_;
        bool done_;
        std::map<void*, CurlContext*> contexts_;
    };
}}}
