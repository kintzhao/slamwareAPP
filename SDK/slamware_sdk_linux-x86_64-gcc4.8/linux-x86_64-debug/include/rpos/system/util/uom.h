/*
* uom.h
* Unit of Measurement
*
* Created By Tony Huang @ 2015-09-28
* Copyright 2015 (c) Shanghai Slamtec Co., Ltd.
*/

#pragma once

#include <rpos/core/rpos_core_config.h>
#include <string>
#include <vector>
#include <boost/math/common_factor.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/noncopyable.hpp>
#include <rpos/system/util/string_utils.h>
#include <rpos/system/fraction.h>
#include <rpos/system/types.h>

namespace rpos{ namespace system{ namespace util {

	namespace units {

		enum Unit {
			// length units
			m,
			um,
			mm,
			cm,
			dm,
			km,
			mil,
			inch,
			foot,
			mile,

			// angle
			rad,
			degree,
			pi,  // this is still rad, just a factor

			// massive
			kg,
			ug,
			mg,
			g,
			oz,
			pound,

			// time duration
			s,
			us,
			ms,
			minute,
			hour,
			day,

			// eletric current
			amp,
			ua,
			ma,

			// temperature
			kelvin,
			celsius,
			fahrenheit,

			// amount of substance
			mol,
			
			// luminous intensity
			cd
		};

	}

	template < class RepT >
	Fraction<RepT> get_unit_factor(units::Unit unit)
	{
#define RPOS_SYSTEM_UTIL_UOM_DEFINE_UNIT_FACTOR(Unit, FactorN, FactorD) \
	case Unit: return Fraction<RepT>(FactorN, FactorD)

		switch (unit)
		{
			RPOS_SYSTEM_UTIL_UOM_DEFINE_UNIT_FACTOR(units::m, 1, 1);
			RPOS_SYSTEM_UTIL_UOM_DEFINE_UNIT_FACTOR(units::um, 1, 1000000);
			RPOS_SYSTEM_UTIL_UOM_DEFINE_UNIT_FACTOR(units::mm, 1, 1000);
			RPOS_SYSTEM_UTIL_UOM_DEFINE_UNIT_FACTOR(units::cm, 1, 100);
			RPOS_SYSTEM_UTIL_UOM_DEFINE_UNIT_FACTOR(units::dm, 1, 10);
			RPOS_SYSTEM_UTIL_UOM_DEFINE_UNIT_FACTOR(units::km, 1000, 1);
			RPOS_SYSTEM_UTIL_UOM_DEFINE_UNIT_FACTOR(units::mil, 254, 10000000);
			RPOS_SYSTEM_UTIL_UOM_DEFINE_UNIT_FACTOR(units::inch, 254, 10000);
			RPOS_SYSTEM_UTIL_UOM_DEFINE_UNIT_FACTOR(units::foot, 3048, 10000);
			RPOS_SYSTEM_UTIL_UOM_DEFINE_UNIT_FACTOR(units::mile, 1609344, 1000);
			RPOS_SYSTEM_UTIL_UOM_DEFINE_UNIT_FACTOR(units::rad, 1, 1);
			RPOS_SYSTEM_UTIL_UOM_DEFINE_UNIT_FACTOR(units::degree, 174532925, 10000000000);
			RPOS_SYSTEM_UTIL_UOM_DEFINE_UNIT_FACTOR(units::pi, 314159265359, 100000000000);
			RPOS_SYSTEM_UTIL_UOM_DEFINE_UNIT_FACTOR(units::kg, 1000, 1);
			RPOS_SYSTEM_UTIL_UOM_DEFINE_UNIT_FACTOR(units::ug, 1, 1000000000);
			RPOS_SYSTEM_UTIL_UOM_DEFINE_UNIT_FACTOR(units::mg, 1, 1000000);
			RPOS_SYSTEM_UTIL_UOM_DEFINE_UNIT_FACTOR(units::g, 1, 1000);
			RPOS_SYSTEM_UTIL_UOM_DEFINE_UNIT_FACTOR(units::oz, 283495231, 10000000000);
			RPOS_SYSTEM_UTIL_UOM_DEFINE_UNIT_FACTOR(units::pound, 45359237, 100000000);
			RPOS_SYSTEM_UTIL_UOM_DEFINE_UNIT_FACTOR(units::s, 1, 1);
			RPOS_SYSTEM_UTIL_UOM_DEFINE_UNIT_FACTOR(units::us, 1, 1000000);
			RPOS_SYSTEM_UTIL_UOM_DEFINE_UNIT_FACTOR(units::ms, 1, 1000);
			RPOS_SYSTEM_UTIL_UOM_DEFINE_UNIT_FACTOR(units::minute, 60, 1);
			RPOS_SYSTEM_UTIL_UOM_DEFINE_UNIT_FACTOR(units::hour, 3600, 1);
			RPOS_SYSTEM_UTIL_UOM_DEFINE_UNIT_FACTOR(units::day, 86400, 1);
			RPOS_SYSTEM_UTIL_UOM_DEFINE_UNIT_FACTOR(units::amp, 1, 1);
			RPOS_SYSTEM_UTIL_UOM_DEFINE_UNIT_FACTOR(units::ua, 1, 1000000);
			RPOS_SYSTEM_UTIL_UOM_DEFINE_UNIT_FACTOR(units::ma, 1, 1000);
			RPOS_SYSTEM_UTIL_UOM_DEFINE_UNIT_FACTOR(units::kelvin, 1, 1);
			RPOS_SYSTEM_UTIL_UOM_DEFINE_UNIT_FACTOR(units::celsius, 1, 1);
			RPOS_SYSTEM_UTIL_UOM_DEFINE_UNIT_FACTOR(units::fahrenheit, 5, 9);
			RPOS_SYSTEM_UTIL_UOM_DEFINE_UNIT_FACTOR(units::mol, 1, 1);
			RPOS_SYSTEM_UTIL_UOM_DEFINE_UNIT_FACTOR(units::cd, 1, 1);
		default:
			throw std::invalid_argument("invalid unit");
		}
	}

	template < class RepT, units::Unit unit >
	static inline RepT normalize_unit_value(const RepT& v)
	{
		if (unit == units::celsius)
		{
			return (RepT)(v + 273.16);
		}
		else if (unit == units::fahrenheit)
		{
			return (RepT)((v + 459.67) * 5 / 9);
		}
		else
		{
			return (v * get_unit_factor<RepT>(unit)).flatten();
		}
	}

	template < class RepT >
	RepT normalize_unit_value(const RepT& v, units::Unit unit)
	{
		switch (unit)
		{
		case units::m: return normalize_unit_value<RepT, units::m>(v);
		case units::um: return normalize_unit_value<RepT, units::um>(v);
		case units::mm: return normalize_unit_value<RepT, units::mm>(v);
		case units::cm: return normalize_unit_value<RepT, units::cm>(v);
		case units::dm: return normalize_unit_value<RepT, units::dm>(v);
		case units::km: return normalize_unit_value<RepT, units::km>(v);
		case units::mil: return normalize_unit_value<RepT, units::mil>(v);
		case units::inch: return normalize_unit_value<RepT, units::inch>(v);
		case units::foot: return normalize_unit_value<RepT, units::foot>(v);
		case units::mile: return normalize_unit_value<RepT, units::mile>(v);
		case units::rad: return normalize_unit_value<RepT, units::rad>(v);
		case units::degree: return normalize_unit_value<RepT, units::degree>(v);
		case units::pi: return normalize_unit_value<RepT, units::pi>(v);
		case units::kg: return normalize_unit_value<RepT, units::kg>(v);
		case units::ug: return normalize_unit_value<RepT, units::ug>(v);
		case units::mg: return normalize_unit_value<RepT, units::mg>(v);
		case units::g: return normalize_unit_value<RepT, units::g>(v);
		case units::oz: return normalize_unit_value<RepT, units::oz>(v);
		case units::pound: return normalize_unit_value<RepT, units::pound>(v);
		case units::s: return normalize_unit_value<RepT, units::s>(v);
		case units::us: return normalize_unit_value<RepT, units::us>(v);
		case units::ms: return normalize_unit_value<RepT, units::ms>(v);
		case units::minute: return normalize_unit_value<RepT, units::minute>(v);
		case units::hour: return normalize_unit_value<RepT, units::hour>(v);
		case units::day: return normalize_unit_value<RepT, units::day>(v);
		case units::amp: return normalize_unit_value<RepT, units::amp>(v);
		case units::ua: return normalize_unit_value<RepT, units::ua>(v);
		case units::ma: return normalize_unit_value<RepT, units::ma>(v);
		case units::kelvin: return normalize_unit_value<RepT, units::kelvin>(v);
		case units::celsius: return normalize_unit_value<RepT, units::celsius>(v);
		case units::fahrenheit: return normalize_unit_value<RepT, units::fahrenheit>(v);
		case units::mol: return normalize_unit_value<RepT, units::mol>(v);
		case units::cd: return normalize_unit_value<RepT, units::cd>(v);
		default:
			throw std::invalid_argument("invalid unit");
		}
	}

	template < class RepT >
	RepT normalize_unit_value(typename types::NumberTypeMeta<RepT>::most_advanced_t v, std::vector<units::Unit>& multiple_units, std::vector<units::Unit>& divide_units)
	{
		Fraction<typename types::NumberTypeMeta<RepT>::most_advanced_t> output(v);

		for (auto iter = multiple_units.begin(); iter != multiple_units.end(); iter++)
		{
			output = output * get_unit_factor<typename types::NumberTypeMeta<RepT>::most_advanced_t>(*iter);
		}

		for (auto iter = divide_units.begin(); iter != divide_units.end(); iter++)
		{
			output = output / get_unit_factor<typename types::NumberTypeMeta<RepT>::most_advanced_t>(*iter);
		}

		return (RepT)output.flatten();
	}

	bool try_parse(const std::string& s, units::Unit& v);

	bool split_number_and_units(const std::string& s, std::string& number, std::vector<units::Unit>& multiple_units, std::vector<units::Unit>& divide_units);

	template < class RepT >
	bool try_parse_with_unit(const std::string& s, RepT& v)
	{
		std::string numberStr;
		std::vector<units::Unit> multiple_units;
		std::vector<units::Unit> divide_units;

		if (!split_number_and_units(s, numberStr, multiple_units, divide_units))
			return false;

		typename types::NumberTypeMeta<RepT>::most_advanced_t av;

		if (!try_parse(numberStr, av))
			return false;

		for (auto iter = divide_units.begin(); iter != divide_units.end(); iter++)
		{
			switch (*iter)
			{
			case units::celsius:
			case units::fahrenheit:
				return false;
			}
		}

		v = normalize_unit_value<RepT>(av, multiple_units, divide_units);

		return true;
	}

} } }
