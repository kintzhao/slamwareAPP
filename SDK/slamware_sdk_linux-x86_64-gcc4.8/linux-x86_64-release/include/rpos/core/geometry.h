/*
* geometry.h
* Geometry objects
*
* Created by Jacky Li (eveningwear@gmail.com) at 2014-12-26
* Copyright 2014 (c) www.robopeak.com
*/

#pragma once

#include <rpos/core/rpos_core_config.h>
#include <rpos/system/types.h>

#include <Eigen/Eigen>

#include "detail/geometry_matrix.h"
#include "detail/geometry_line.h"
#include "detail/geometry_rectangle.h"

namespace rpos {
    namespace core {
        typedef rpos::system::types::_u32 SegmentID;

        class RPOS_CORE_API Point {
        public:
            Point();
            Point(float x, float y);
            Point(const Point&);
            ~Point();

        public:
            Point& operator=(const Point&);

        public:
            float x() const;
            float& x();

            float y() const;
            float& y();

        private:
            float x_, y_;
        };

        class RPOS_CORE_API Line {
        public:
            Line();
            Line(const Point &startP, const Point &endP);
            Line(const Point &startP, const Point &endP, int id);
            Line(const Line&);
            ~Line();

        public:
            Line& operator=(const Line&);

        public:
            Point& startP();
            const Point& startP() const;
            Point& endP();
            const Point& endP() const;
            SegmentID& id();
            const SegmentID& id() const;

        private:
            Point startP_;
            Point endP_;
            SegmentID id_;
        };
    }
}