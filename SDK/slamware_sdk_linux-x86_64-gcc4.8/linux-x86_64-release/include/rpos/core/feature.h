/**
* feature.h
* Feature means a set of functions of robot
*
* Created By Tony Huang @ 2014-5-22
* Copyright (c) 2014 Shanghai SlamTec Co., Ltd.
*/

#pragma once

#include <rpos/rpos_config.h>
#include <rpos/system/object_handle.h>
#include <string>
#include <vector>

namespace rpos {
	namespace core {

		class Feature;

		namespace detail {
			class FeatureImpl;

			template<class FeatureT>
			struct feature_caster {
				static FeatureT cast(Feature&);
			};
		}

		class RPOS_CORE_API Feature : public rpos::system::ObjectHandle<Feature, detail::FeatureImpl>{
		public:
			RPOS_OBJECT_CTORS(Feature);
			~Feature();

		public:
			std::string featureId();
			std::string description();
            std::vector<std::string> dependencies();

            bool initialize();
            bool finalize();

			template<class FeatureT>
			FeatureT cast() {
				return detail::feature_caster<FeatureT>::cast(*this);
			}

		private:
			template<class FeatureT>
			friend struct detail::feature_caster;
		};

	}
}