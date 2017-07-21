#pragma once
#include <rpos/app/robot_application.h>

namespace rpos 
{ 
	namespace frameworks 
	{
		template < class TApplication, class TOption = typename detail::get_option_class<TApplication>::value >
		class SlamwareRobotApplication : public RobotApplication<TApplication, TOption>
		{
		public:
			 TOption config_;
		};
	} 
}