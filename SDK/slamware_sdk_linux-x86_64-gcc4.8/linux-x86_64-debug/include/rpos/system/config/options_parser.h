#pragma once

#include <string>
#include <map>
#include <vector>
#include <boost/noncopyable.hpp>

namespace rpos { namespace system { namespace config {

    struct Option {
        Option()
            : acceptArgument(false)
            , exists(false)
        {}

        std::string name;
        std::string description;
        std::string shortOption;
        std::string longOption;
        bool acceptArgument;

        bool exists;
        std::string argument;
    };

    class OptionParser : private boost::noncopyable {
    public:
        void addOption(Option* option);
        
        bool parse(int argc, const char* argv[], std::vector<std::string>& outRestArguments);
		void printHelp();

    private:
        std::map<std::string, Option*> shortOptions_;
        std::map<std::string, Option*> longOptions_;
    };

} } }
