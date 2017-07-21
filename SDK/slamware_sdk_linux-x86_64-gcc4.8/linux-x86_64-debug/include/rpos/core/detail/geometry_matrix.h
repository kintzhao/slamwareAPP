/*
* geometry_matrix.h
* Matrix objects
*
* Created by Tony Huang (cnwzhjs@gmail.com) at 2014-05-25
* Copyright 2014 (c) www.robopeak.com
*/

#pragma once

#include <Eigen/Eigen>

namespace rpos {
	namespace core {

		typedef Eigen::Vector2i Vector2i;
		typedef Eigen::Vector3i Vector3i;
		typedef Eigen::Vector4i Vector4i;

		typedef Eigen::Vector2f Vector2f;
		typedef Eigen::Vector3f Vector3f;
		typedef Eigen::Vector4f Vector4f;

		typedef Eigen::Matrix2f Matrix2f;
		typedef Eigen::Matrix3f Matrix3f;
		typedef Eigen::Matrix4f Matrix4f;

	}
}
