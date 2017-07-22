/**
 kint.zhao  huasheng_zyh@163.com
   2017.0721
*/
#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "std_srvs/Empty.h"
#include "std_msgs/Float64.h"
#include "nav_msgs/GetMap.h"
#include <nav_msgs/Path.h>


#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>

#include "tf/transform_listener.h"
#include "tf/transform_broadcaster.h"
#include "message_filters/subscriber.h"
#include "tf/message_filter.h"

#include <boost/thread.hpp>

#include <rpos/robot_platforms/slamware_core_platform.h>
#include <rpos/robot_platforms/slamware_core_platform.h>

using namespace rpos::robot_platforms;
using namespace rpos::features;
using namespace rpos::features::location_provider;
using namespace rpos::system::types;
using namespace rpos::core;

class slamwareAPP {
public:
    slamwareAPP();
    slamwareAPP(std::string ip);
    ~slamwareAPP();

    void init();
    bool connectSlamware();
    void startSlamwareWork();

    void publishRobotPose(double publish_period);
    void publishScan(double publish_period);
    void publishMap(double publish_period);
    void publishPlanPath(float publish_period);
    void robotControlCallback (const geometry_msgs::TwistConstPtr &vel);
    void moveToGoalCallback (const geometry_msgs::PoseStamped::ConstPtr& goal);

    ros::Publisher scan_pub_;
    ros::Publisher robot_pose_pub_;
    ros::Publisher gridmap_pub_;
    ros::Publisher gridmap_info_pub_;
    ros::Publisher global_plan_pub_;

    ros::Subscriber robot_vel_sub_;
    ros::Subscriber goal_sub_;

private:
    ros::NodeHandle nh_;
    std::string ip_addres_;
    bool workOk;
    SlamwareCorePlatform SDP_;

    float robot_pose_pub_period_;
    float scan_pub_period_;
    float map_pub_period_;
    float transform_publish_period;

    tf::TransformBroadcaster tfB_;

    boost::thread*  robot_pose_pub_thread_;
    boost::thread* scan_pub_thread_;
    boost::thread* map_pub_thread_;
    boost::thread* transform_thread_;
    boost::thread* plan_path_pub_thread_;

    std::string robot_frame_;
    std::string laser_frame_;
    std::string map_frame_;
    std::string odom_frame_;

    std::string topic_control;
    std::string topic_goal;
};

