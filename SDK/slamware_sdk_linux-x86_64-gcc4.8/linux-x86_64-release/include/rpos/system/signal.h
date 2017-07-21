/*
* signal.h
* Signal slot infrastructure of rpos
*
* Created by Tony Huang (cnwzhjs@gmail.com) at 2014-05-20
* Copyright 2012 (c) www.robopeak.com
*/

#pragma once

#include <rpos/rpos_config.h>

#include "signal/connection.h"
#include "signal/signal_impl.h"

#include <map>
#include <boost/noncopyable.hpp>
#include <boost/function.hpp>
#include <boost/atomic.hpp>
#include <boost/thread.hpp>

namespace rpos {
	namespace system {

		namespace detail {
			template<class Signature>
			class SignalImpl : private boost::noncopyable {
			public:
				typedef Connection<Signature> connection_t;
				typedef boost::function<Signature> function_t;

				SignalImpl()
					: inSnapshotMode_(false)
				{}

				~SignalImpl()
				{}

				connection_t connect(function_t slot)
				{
					boost::lock_guard<boost::mutex> guard(lock_);

					token_t token = peakToken_.fetch_add(1, boost::memory_order_relaxed);

					if (inSnapshotMode_.load(boost::memory_order_consume))
					{
						pendingActions_.push_back(PendingAction(ModifyActionAdd, slot, token));
					}
					else
					{
						handlers_[token] = slot;
					}

					return connection_t(*this, token);
				}

			protected:
				typedef typename connection_t::token_t token_t;

				friend class Connection<Signature>;

				void beginSnapshot()
				{
					snapshotLock_.lock();

					{
						boost::lock_guard<boost::mutex> guard(lock_);
						inSnapshotMode_.store(true, boost::memory_order_release);
					}
				}

				void endSnapshot()
				{
					{
						boost::lock_guard<boost::mutex> guard(lock_);
						inSnapshotMode_.store(false, boost::memory_order_release);

						if (!pendingActions_.empty())
						{
							for (auto it = pendingActions_.begin(); it != pendingActions_.end(); it++)
							{
								PendingAction& action = *it;

								if (action.action == ModifyActionAdd)
								{
									handlers_[action.token] = action.function;
								}
								else if (action.action == ModifyActionRemove)
								{
									handlers_.erase(action.token);
								}
								else
								{
									assert(!"action.action out of range");
								}
							}

							pendingActions_.clear();
						}
					}

					snapshotLock_.unlock();
				}

				void removeConnection(token_t token)
				{
					boost::lock_guard<boost::mutex> guard(lock_);

					if (inSnapshotMode_.load(boost::memory_order_consume))
					{
						pendingActions_.push_back(PendingAction(ModifyActionRemove, token));
					}
					else
					{
						handlers_.erase(token);
					}
				}

				class SnapshotScope {
				public:
					SnapshotScope(SignalImpl& signal) : signal_(signal) {
						signal.beginSnapshot();
					}

					~SnapshotScope() {
						signal_.endSnapshot();
					}

				private:
					SignalImpl& signal_;
				};

				enum ModifyAction {
					ModifyActionAdd,
					ModifyActionRemove
				};

				struct PendingAction {
					PendingAction(const PendingAction& that)
					: action(that.action), function(that.function), token(that.token)
					{}

					PendingAction(ModifyAction action, function_t func, token_t token)
						: action(action), function(func), token(token)
					{}

					PendingAction(ModifyAction action, token_t token)
						: action(action), function(), token(token)
					{}

					~PendingAction() {}

					ModifyAction action;
					function_t function;
					token_t token;
				};

				std::map<token_t, function_t> handlers_;
				mutable boost::atomic<token_t> peakToken_;
				boost::mutex lock_;

				boost::atomic_bool inSnapshotMode_;
				boost::mutex snapshotLock_;
				std::list<PendingAction> pendingActions_;
			};
		}

		template<class Signature>
		class Signal;

		RPOS_SIGNAL_IMPL(0)
		RPOS_SIGNAL_IMPL(1)
		RPOS_SIGNAL_IMPL(2)
		RPOS_SIGNAL_IMPL(3)
		RPOS_SIGNAL_IMPL(4)

	}
}
