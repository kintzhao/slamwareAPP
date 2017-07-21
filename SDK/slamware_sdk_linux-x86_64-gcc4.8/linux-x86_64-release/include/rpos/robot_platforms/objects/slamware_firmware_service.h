#pragma once

#include <string>

namespace rpos { namespace robot_platforms { namespace detail { namespace objects {

    struct UpdateInfo
    {
        std::string currentVersion;
        std::string newVersion;
        std::string newVersionReleaseDate;
        std::string newVersionChangeLog;

        UpdateInfo& operator=(const UpdateInfo& that)
        {
            currentVersion = that.currentVersion;
            newVersion = that.newVersion;
            newVersionReleaseDate = that.newVersionReleaseDate;
            newVersionChangeLog = that.newVersionChangeLog;

            return *this;
        }
    };

    struct UpdateProgress
    {
        unsigned int currentStep;           // Range from 0 to {totalSteps - 1}.
        unsigned int totalSteps;            // The number of total steps.
        std::string currentStepName;        // The name of the current step.
        unsigned int currentStepProgress;   // Expressed as a percentage. 

        UpdateProgress& operator=(const UpdateProgress& that)
        {
            currentStep = that.currentStep;
            totalSteps = that.totalSteps;
            currentStepName = that.currentStepName;
            currentStepProgress = that.currentStepProgress;

            return *this;
        }
    };

}}}}