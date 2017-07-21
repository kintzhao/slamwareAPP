#pragma once

#include <boost/chrono.hpp>
#include <boost/thread.hpp>

namespace rpos { namespace system { namespace util {

    class IntervalEvent {
    public:
        template < class DurationT >
        IntervalEvent(const DurationT& interval)
            : interval_(boost::chrono::duration_cast<boost::chrono::microseconds>(interval))
        {
            reset();
        }

        ~IntervalEvent()
        {}

    public:
        void reset()
        {
            next_ = boost::chrono::high_resolution_clock::now() + interval_;
        }

        bool should_trigger() const
        {
            return boost::chrono::high_resolution_clock::now() >= next_;
        }

        bool reset_if_should_trigger()
        {
            if (should_trigger())
            {
                reset();
                return true;
            }
            else
            {
                return false;
            }
        }

        boost::chrono::microseconds duration_to_trigger() const
        {
            return boost::chrono::duration_cast<boost::chrono::microseconds>(next_ - boost::chrono::high_resolution_clock::now());
        }

        void wait_until_trigger()
        {
            if (reset_if_should_trigger())
            {
                return;
            }

            boost::this_thread::sleep_for(duration_to_trigger());
            reset_if_should_trigger();
        }

    private:
        boost::chrono::high_resolution_clock::time_point next_;
        boost::chrono::microseconds interval_;
    };

} } }
