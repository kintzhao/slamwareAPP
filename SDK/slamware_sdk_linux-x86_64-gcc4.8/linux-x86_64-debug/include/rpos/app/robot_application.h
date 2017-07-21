#pragma once 
#include <rpos/system/util/log.h>
#include <rpos/system/config/config_parser.h>
#include <rpos/system/config/core_objects_config_parser.h>
#include <rpos/core/feature_manager.h>

namespace rpos
{ 
	namespace frameworks 
	{
		struct RobotApplicationLogFileConfig 
		{
			rpos::system::util::LogLevel logLevel;
			std::string filename;
		};

		struct RobotApplicationLogConfig 
		{
			rpos::system::util::LogLevel consoleLogLevel;
			std::vector<RobotApplicationLogFileConfig> logFiles;
		};

		struct RobotApplicationConfig 
		{
			RobotApplicationLogConfig log;
		};

		namespace detail
		{

			template < class TApplication >
			struct get_option_class 
			{
				typedef RobotApplicationConfig value;
			};

		}

		template < class TApplication, class TOption = typename detail::get_option_class<TApplication>::value >
		class RobotApplication : public boost::enable_shared_from_this<TApplication>, private boost::noncopyable
		{
		public:
			template <class TFeature>
			TFeature createFeature()
			{
				TFeature feature;
				return feature;
			}

			template <class TFeature, class TConfig>
			TFeature createFeatureWithConfig(const TConfig& config)
			{
				auto manager = rpos::core::FeatureManager::defaultManager();
				static TFeature feature(config);
				manager.registerFeature(feature);
				return feature;
			}
		public:
			virtual int run(int argc, const char* argv[]) = 0;
		};
	}
}

namespace rpos 
{ 
	namespace system 
	{ 
		namespace config 
		{ 

			template<>
			struct ConfigParser <rpos::frameworks::RobotApplicationLogFileConfig > {
				static bool parse(const Json::Value& config, rpos::frameworks::RobotApplicationLogFileConfig& that)
				{
					CONFIG_PARSE_CHILD_WITH_DEFAULT(logLevel, rpos::system::util::LogLevelInfo);
					CONFIG_PARSE_CHILD(filename);

					return true;
				}
			};
			
			template<>
			struct ConfigParser <rpos::frameworks::RobotApplicationLogConfig > {
				static bool parse(const Json::Value& config, rpos::frameworks::RobotApplicationLogConfig& that)
				{
#if defined(DEBUG) || defined(_DEBUG)
					CONFIG_PARSE_CHILD_WITH_DEFAULT(consoleLogLevel, rpos::system::util::LogLevelDebug);
#else
					CONFIG_PARSE_CHILD_WITH_DEFAULT(consoleLogLevel, rpos::system::util::LogLevelInfo);
#endif

					CONFIG_PARSE_CHILD(logFiles);

					return true;
				}

			};

			template<>
			struct ConfigParser <rpos::frameworks::RobotApplicationConfig> 
			{
				static bool parse(const Json::Value& config, rpos::frameworks::RobotApplicationConfig& that)
				{
					CONFIG_PARSE_CHILD(log);
					return true;
				}
			};
		}
	}
}