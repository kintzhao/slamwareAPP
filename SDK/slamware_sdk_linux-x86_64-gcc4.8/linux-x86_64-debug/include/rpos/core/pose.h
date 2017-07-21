/**
* pose.h
* Robot Pose
*
* Created By Tony Huang @ 2014-5-22
* Copyright (c) 2014 Shanghai SlamTec Co., Ltd.
*/

#pragma once

#include <rpos/rpos_config.h>
#include <rpos/core/rpos_core_config.h>

namespace rpos {
    namespace core {

        enum ACTION_DIRECTION
        {
            FORWARD,
            BACKWARD,
            TURNRIGHT,
            TURNLEFT
        };

        class RPOS_CORE_API Location{
        public:
            Location();
            Location(double x, double y, double z = 0);
            Location(const Location&);
            ~Location();

        public:
            Location& operator=(const Location&);

        public:
            double x() const;
            double& x();

            double y() const;
            double& y();

            double z() const;
            double& z();

            double distanceTo(const Location& that) const;

        private:
            double x_, y_, z_;
        };

        class RPOS_CORE_API Rotation{
        public:
            Rotation();
            Rotation(double yaw, double pitch = 0, double roll = 0);
            Rotation(const Rotation&);
            ~Rotation();

        public:
            Rotation& operator=(const Rotation&);

        public:
            double yaw() const;
            double& yaw();

            double pitch() const;
            double& pitch();

            double roll() const;
            double& roll();

        private:
            double yaw_, pitch_, roll_;
        };

        class RPOS_CORE_API Pose{
        public:
            Pose();
            Pose(const Location&, const Rotation&);
            Pose(const Location&);
            Pose(const Rotation&);
            Pose(const Pose&);
            ~Pose();

        public:
            Pose& operator=(const Pose&);

        public:
            const Location& location() const;
            Location& location();

            const Rotation& rotation() const;
            Rotation& rotation();

            double x() const;
            double& x();

            double y() const;
            double& y();

            double z() const;
            double& z();

            double yaw() const;
            double& yaw();

            double pitch() const;
            double& pitch();

            double roll() const;
            double& roll();

        private:
            Location location_;
            Rotation rotation_;
        };

        class RPOS_CORE_API Direction{
        public:
            Direction(ACTION_DIRECTION direction = FORWARD);
            ~Direction();

        public:
            Direction& operator=(const Direction&);

        public:
            ACTION_DIRECTION direction() const;
            ACTION_DIRECTION& direction();

        private:
            ACTION_DIRECTION direction_;
        };

    }
}
