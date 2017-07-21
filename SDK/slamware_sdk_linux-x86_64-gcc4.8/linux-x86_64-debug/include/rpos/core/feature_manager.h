/**
* feature_manager.h
* Feature manager manages application-scope features
* It will resolve the dependency chain of features and initialize features in correct order
*
* Created By Tony Huang @ 2015-2-4
* Copyright (c) 2015 Shanghai Slamtec Co., Ltd.
*/

#pragma once

#include <rpos/rpos_config.h>
#include <rpos/system/object_handle.h>
#include "feature.h"

namespace rpos { namespace core {

    namespace detail {

        class FeatureManagerImpl;

    }

    class FeatureManager : public system::ObjectHandle < FeatureManager, detail::FeatureManagerImpl > {
    public:
        RPOS_OBJECT_CTORS(FeatureManager);
        ~FeatureManager();

    public:
        bool registerFeature(Feature feature);
        void clearFeatures();
        bool removeFeature(const std::string& featureId);
        bool removeFeature(Feature feature);

    public:
        std::vector<Feature> getFeatures() const;

    public:
        Feature find(const std::string& featureId);

        template<class TFeature>
        bool resolve(const std::string& featureId, TFeature& feature)
        {
            feature = find(featureId).cast<TFeature>();

            return (bool)feature;
        }

        bool initialize(const std::string& featureId);
        bool initializeAll();

        bool finalize(const std::string& identifier);
        bool finalizeAll();

    public:
        static FeatureManager defaultManager();
    };

} }
