/**
 kint.zhao  huasheng_zyh@163.com
   2017.0721
*/
#include <ros/ros.h>

#include <regex>
#include <boost/thread.hpp>
#include <boost/chrono.hpp>
#include <rpos/robot_platforms/slamware_core_platform.h>
#include <rpos/features/location_provider/map.h>

#include  "slamwareAPP.h"

using namespace rpos::robot_platforms;
using namespace rpos::features;
using namespace rpos::features::location_provider;
using namespace std;

int main(int argc, char** argv)
{
  ros::init(argc, argv, "slamwareAPP");
  const int reconnect_count_threshold = 10;

  slamwareAPP slamwareAPPNode;

  int reconnect_count_timer = 0;
  while(!slamwareAPPNode.connectSlamware() && reconnect_count_timer++ < reconnect_count_threshold)
  {
      std::cout<<" ===> reconnect : "<<reconnect_count_timer<<std::endl;
  }
  if(reconnect_count_timer >= reconnect_count_threshold)
  {
      std::cout<<" ===> reconnect error, cannot connect the slamware ! "<<std::endl;
      return(0);
  }
  std::cout<<" ===> slamwareAPP start work! "<<std::endl;
  slamwareAPPNode.startSlamwareWork();

  ros::spin();

  return(0);
}

