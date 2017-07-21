/*
* fraction.h
* Fraction
*
* Created By Tony Huang @ 2015-9-28
* Copyright 2015 (c) Shanghai Slamtec Co., Ltd.
*/

#pragma once

#include <boost/math/common_factor.hpp>

namespace rpos { namespace system {

	template < class RepT >
	struct Fraction
	{
		RepT numerator;
		RepT denominator;

		Fraction(const RepT& n, const RepT& d = 1)
			: numerator(n)
			, denominator(d)
		{}

		Fraction<RepT> simplify() const
		{
			RepT gcd = boost::math::gcd(numerator, denominator);

			if (gcd == 0 || gcd == 1)
				return *this;
		}

		Fraction<RepT> inverse() const
		{
			return Fraction<RepT>(denominator, numerator);
		}

		RepT flatten() const
		{
			return numerator / denominator;
		}
	};

	template < class RepT >
	Fraction<RepT> operator+(const Fraction<RepT>& a, const Fraction<RepT>& b)
	{
		if (a.denominator == b.denominator)
			return Fraction<RepT>(a.numerator + b.numerator, a.denominator);
		else
			return Fraction<RepT>(a.numerator * b.denominator + b.numerator * a.denominator, a.denominator * b.denominator);
	}

	template < class RepT >
	Fraction<RepT> operator-(const Fraction<RepT>& a, const Fraction<RepT>& b)
	{
		if (a.denominator == b.denominator)
			return Fraction<RepT>(a.numerator - b.numerator, a.denominator);
		else
			return Fraction<RepT>(a.numerator * b.denominator - b.numerator * a.denominator, a.denominator * b.denominator);
	}

	template < class RepT >
	Fraction<RepT> operator*(const Fraction<RepT>& a, const Fraction<RepT>& b)
	{
		return Fraction<RepT>(a.numerator * b.numerator, a.denominator * b.denominator);
	}

	template < class RepT >
	Fraction<RepT> operator/(const Fraction<RepT>& a, const Fraction<RepT>& b)
	{
		return a * b.inverse();
	}

#define RPOS_SYSTEM_FRACTION_OPERATOR_WITH_SCALAR(o) \
	template < class RepT > \
	Fraction<RepT> operator o(const Fraction<RepT>& a, const RepT& b) \
	{ return a o Fraction<RepT>(b); } \
	template < class RepT > \
	Fraction<RepT> operator o(const RepT& a, const Fraction<RepT>& b) \
	{ return Fraction<RepT>(a) o b; }

	RPOS_SYSTEM_FRACTION_OPERATOR_WITH_SCALAR(+);
	RPOS_SYSTEM_FRACTION_OPERATOR_WITH_SCALAR(-);
	RPOS_SYSTEM_FRACTION_OPERATOR_WITH_SCALAR(*);
	RPOS_SYSTEM_FRACTION_OPERATOR_WITH_SCALAR(/);

} }
