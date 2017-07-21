#pragma once

#include <boost/chrono.hpp>

namespace rpos { namespace system { namespace util {

    template < class T >
    class EventStat {
    public:
        EventStat()
        {
            clear();
        }

        ~EventStat()
        {}

    public:
        void clear()
        {
            startTime_ = lastTime_ = boost::chrono::high_resolution_clock::now();
            lastDuration_ = boost::chrono::milliseconds(0);
            occurred_ = 0;
            sum_ = T();
            last_ = T();
        }

        void push(const T& v)
        {
            auto now = boost::chrono::high_resolution_clock::now();
            lastDuration_ = now - lastTime_;
            lastTime_ = now;

            occurred_++;
            sum_ += v;
            last_ = v;
        }

    public:
        size_t occurred() const
        {
            return occurred_;
        }

        T sum() const
        {
            return sum_;
        }

        T last() const
        {
            return last_;
        }

        boost::chrono::high_resolution_clock::duration lastDuration() const
        {
            return lastDuration_;
        }

        boost::int_least64_t lastDurationInUs() const
        {
            return boost::chrono::duration_cast<boost::chrono::microseconds>(lastDuration()).count();
        }

        double lastFrequency() const
        {
            if (!occurred_)
                return -1;

            return 1000000.0 / lastDurationInUs();
        }

        double averageFrequency() const
        {
            if (!occurred_)
                return -1;

            auto durationInUs = boost::chrono::duration_cast<boost::chrono::microseconds>(lastTime_ - startTime_).count();

            if (!durationInUs)
                return -1;

            return (double)occurred_ / durationInUs * 1000000;
        }

        T average() const
        {
            if (occurred_)
                return sum_ / occurred_;
            else
                return T();
        }

    private:
        boost::chrono::high_resolution_clock::time_point startTime_;
        boost::chrono::high_resolution_clock::time_point lastTime_;
        boost::chrono::high_resolution_clock::duration lastDuration_;

        size_t occurred_;
        T sum_;
        T last_;
    };

} } }
