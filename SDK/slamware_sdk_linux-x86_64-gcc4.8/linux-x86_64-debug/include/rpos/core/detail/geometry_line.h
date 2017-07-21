/*
* geometry_line.h
* Line objects
*
* Created by Shikai Chen
* Copyright 2014 (c) www.robopeak.com
*/

#pragma once

#include <rpos/rpos_config.h>
#include "geometry_matrix.h"

namespace rpos {
	namespace core {

		namespace detail {

			template<class VectorT>
			class _Line {
			public:
				typedef typename VectorT::Scalar scalar_t;

				_Line()
				{}

				_Line(VectorT start, VectorT end)
					: start_(start), end_(end)
				{}


				_Line(const _Line& that)
					: start_(that.start_), end_(that.end_)
				{}

				~_Line()
				{}

			public:
				_Line& operator=(const _Line& that)
				{
					start_ = that.start_;
					end_ = that.end_;

					return *this;
				}

			public:


				const VectorT& start() const 
				{
					return start_;
				}

				
				const VectorT& end() const 
				{
					return end_;
				}

				VectorT& start()
				{
					return start_;
				}

				
				VectorT& end()
				{
					return end_;
				}


			private:
				VectorT start_, end_;
			};
		}

		typedef detail::_Line<Vector2f> Line2df;
        typedef detail::_Line<Vector3f> Line3df;
		typedef detail::_Line<Vector2i> Line2di;
        typedef detail::_Line<Vector3i> Line3di;
	}
}