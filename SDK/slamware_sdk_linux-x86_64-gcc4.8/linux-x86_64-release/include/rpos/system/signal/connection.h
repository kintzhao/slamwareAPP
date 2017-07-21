/*
* connection.h
* Connection between signal and slot
*
* Created by Tony Huang (cnwzhjs@gmail.com) at 2014-05-20
* Copyright 2012 (c) www.robopeak.com
*/

#pragma once

#include <rpos/rpos_config.h>
#include <rpos/system/types.h>

namespace rpos {
	namespace system {

		namespace detail {
			template<class Signature>
			class SignalImpl;
		}

		template<class Signature>
		class Connection : private boost::noncopyable {
		public:
			typedef int token_t;
			typedef detail::SignalImpl<Signature> signal_impl_t;

            Connection()
                : signal_(nullptr)
            {
            }
            
			Connection(signal_impl_t& signal, token_t token)
				: signal_(&signal), token_(token)
			{}

			Connection(const Connection& that)
				: signal_(that.signal_), token_(that.token_)
			{}

			~Connection()
			{}
            
            Connection& operator=(const Connection& that)
            {
                signal_ = that.signal_;
                token_ = that.token_;
                
                return *this;
            }

			void dispose()
			{
                if (signal_)
                {
                    signal_->removeConnection(token_);
                    signal_ = nullptr;
                }
			}

		private:
			signal_impl_t* signal_;
			token_t token_;
		};

	}
}
