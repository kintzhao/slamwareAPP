/**
 kint.zhao  huasheng_zyh@163.com
   2017.0721
*/
#include  "slamwareAPP.h"

slamwareAPP::slamwareAPP(std::string ip):workOk(false),ip_addres_(ip),
    nh_("~"), robot_pose_pub_thread_(NULL),scan_pub_thread_(NULL),map_pub_thread_(NULL),plan_path_pub_thread_(NULL)
{

}

slamwareAPP::slamwareAPP():workOk(false), ip_addres_("192.168.11.1"),
    nh_("~"), robot_pose_pub_thread_(NULL),scan_pub_thread_(NULL),map_pub_thread_(NULL),plan_path_pub_thread_(NULL)
{

}

slamwareAPP::~slamwareAPP()
{
    if(robot_pose_pub_thread_){
        delete robot_pose_pub_thread_;
    }

    if(scan_pub_thread_){
        delete scan_pub_thread_;
    }

    if(map_pub_thread_){
        delete map_pub_thread_;
    }

    if(transform_thread_)
    {
        delete transform_thread_;
    }
    if(plan_path_pub_thread_)
    {
        delete plan_path_pub_thread_;
    }
}

bool slamwareAPP::connectSlamware()
{
    try {
        SDP_ = SlamwareCorePlatform::connect("10.16.131.69", 1445);
        std::cout <<"SDK Version: " << SDP_.getSDKVersion() << std::endl;
        std::cout <<"SDP Version: " << SDP_.getSDPVersion() << std::endl;
    } catch(ConnectionTimeOutException& e) {
        std::cout <<e.what() << std::endl;
        return false;
    } catch(ConnectionFailException& e) {
        std::cout <<e.what() << std::endl;
        return false;
    }
    std::cout <<"Connection Successfully" << std::endl;
    return true;
}

void slamwareAPP::init()
{
    robot_frame_ = "/base_link";
    odom_frame_ = "/odom";
    laser_frame_ = "/laser";
    map_frame_ = "/map";

    topic_control = "/cmd_vel";
    topic_goal = "/move_base_simple/goal";

    robot_pose_pub_period_ = 0.05;
    scan_pub_period_ = 0.1;
    map_pub_period_ = 0.2;
    transform_publish_period = 0.05;

    scan_pub_ = nh_.advertise<sensor_msgs::LaserScan>("scan", 10);
    robot_pose_pub_ = nh_.advertise<nav_msgs::Odometry> ("odom", 10);
    gridmap_pub_ = nh_.advertise<nav_msgs::OccupancyGrid> ("map", 1, true);
    gridmap_info_pub_ = nh_.advertise<nav_msgs::MapMetaData>("map_metadata", 1, true);
    global_plan_pub_ = nh_.advertise<nav_msgs::Path>("global_plan_path", 2);

    robot_vel_sub_ = nh_.subscribe (topic_control, 10, &slamwareAPP::robotControlCallback, this);
    goal_sub_ = nh_.subscribe (topic_goal, 10, &slamwareAPP::moveToGoalCallback, this);

    robot_pose_pub_thread_ = new boost::thread(boost::bind(&slamwareAPP::publishRobotPose, this, robot_pose_pub_period_));
    scan_pub_thread_ = new boost::thread(boost::bind(&slamwareAPP::publishScan, this, scan_pub_period_));
    map_pub_thread_ = new boost::thread(boost::bind(&slamwareAPP::publishMap, this, map_pub_period_));
    plan_path_pub_thread_ = new boost::thread(boost::bind(&slamwareAPP::publishPlanPath, this, transform_publish_period));

}

void slamwareAPP::startSlamwareWork()
{
    init();
}

void slamwareAPP::publishRobotPose(double publish_period)
{
    if(publish_period == 0)
        return;

    ros::Rate r(1.0 / publish_period);
    while(ros::ok())
    {
        //send TF transform
        tf::Transform identy_transform;
        identy_transform.setOrigin (tf::Vector3 (0.0, 0.0, 0.0));
        identy_transform.setRotation (tf::Quaternion(0,0,0,1));
        tfB_.sendTransform ( tf::StampedTransform (identy_transform, ros::Time::now (), map_frame_,
                                      odom_frame_));
        tfB_.sendTransform ( tf::StampedTransform (identy_transform, ros::Time::now (), robot_frame_,
                                      laser_frame_));

        //check power
        int battPercentage = SDP_.getBatteryPercentage();
        if(battPercentage < 10)
            std::cout <<"lower power!! Battery: " << battPercentage << "%" << std::endl;

        rpos::core::Location location = SDP_.getLocation();
        rpos::core::Pose pose = SDP_.getPose();

        //publish odom transform
        tf::Transform transform;
        transform.setOrigin (tf::Vector3 (pose.x(), pose.y(), 0.0));
        tf::Quaternion q = tf::createQuaternionFromYaw (pose.yaw());
        transform.setRotation (q);
        tfB_.sendTransform ( tf::StampedTransform (transform, ros::Time::now (), odom_frame_,
                                      robot_frame_));

        //send TF transform
        nav_msgs::Odometry robotPose;
        robotPose.header.frame_id = map_frame_;
        robotPose.header.stamp = ros::Time::now ();
        robotPose.pose.pose.position.x = pose.x();
        robotPose.pose.pose.position.y = pose.y();
        robotPose.pose.pose.orientation = tf::createQuaternionMsgFromYaw(pose.yaw());
        robot_pose_pub_.publish (robotPose);
                r.sleep();
    }
}

void slamwareAPP::publishScan(double transform_publish_period)
{
    if(transform_publish_period == 0)
        return;

    ros::Rate r(1.0 / transform_publish_period);
    while(ros::ok())
    {
        ros::Time start_scan_time = ros::Time::now();
        rpos::features::system_resource::LaserScan ls = SDP_.getLaserScan();
        ros::Time end_scan_time = ros::Time::now();
        double scan_time = (end_scan_time - start_scan_time).toSec() * 1e-3;

        std::vector<rpos::core::LaserPoint> points = ls.getLaserPoints();

        if(points.empty())
            continue;

        sensor_msgs::LaserScan scan_msg;

        scan_msg.header.stamp = start_scan_time;
        scan_msg.header.frame_id = laser_frame_;
        scan_msg.angle_min =  points.front().angle();
        scan_msg.angle_max =  points.back().angle();
        scan_msg.angle_increment = (scan_msg.angle_max - scan_msg.angle_min) / (double)(points.size()-1);

        scan_msg.scan_time = scan_time;
        scan_msg.time_increment = scan_time / (double)(points.size()-1);
        scan_msg.range_min = 0.15;
        scan_msg.range_max = 8.0;

        scan_msg.intensities.resize(points.size());
        scan_msg.ranges.resize(points.size());

        for(int i = 0; i< points.size(); i++)
        {
            if(!points[i].valid())
            {
                scan_msg.ranges[i] = std::numeric_limits<float>::infinity();
                continue;
            }
            scan_msg.ranges[i] = points[i].distance();
        }
        scan_pub_.publish(scan_msg);
        r.sleep();
    }
}

void slamwareAPP::publishMap(double publish_period)
{
    if(publish_period == 0)
        return;

    ros::Rate r(1.0 / publish_period);
    while(ros::ok())
    {
        Map map = SDP_.getMap(MapTypeBitmap8Bit, rpos::core::RectangleF(-10, -10, 20, 20), rpos::features::location_provider::EXPLORERMAP);
        std::vector<_u8> & mapData = map.getMapData();
        rpos::core::RectangleF rec = map.getMapArea();

        nav_msgs::GetMap::Response map_ros;
        map_ros.map.info.resolution = map.getMapResolution().x();
        map_ros.map.info.origin.position.x = map.getMapPosition().x();
        map_ros.map.info.origin.position.y = map.getMapPosition().y();
        map_ros.map.info.origin.position.z = 0.0;
        map_ros.map.info.origin.orientation.x = 0.0;
        map_ros.map.info.origin.orientation.y = 0.0;
        map_ros.map.info.origin.orientation.z = 0.0;
        map_ros.map.info.origin.orientation.w = 1.0;

        map_ros.map.info.width = map.getMapDimension().x();
        map_ros.map.info.height = map.getMapDimension().y();
        map_ros.map.data.resize(map_ros.map.info.width * map_ros.map.info.height);

        for (int y=0; y<map.getMapDimension().y(); y++)
        {
            for (int x=0; x<map.getMapDimension().x(); x++)
            {
                _u8 value = mapData[x+ y*map.getMapDimension().x()];
                if (value  == 0)
                    map_ros.map.data[x+ y*map.getMapDimension().x()] = -1;
                else if(value <= 127)
                    map_ros.map.data[x+ y*map.getMapDimension().x()] = 0;
                else if(value > 127)
                    map_ros.map.data[x+ y*map.getMapDimension().x()] = 100;
            }
        }

        // Set the header information on the map
        map_ros.map.header.stamp = ros::Time::now();
        map_ros.map.header.frame_id = map_frame_;

        gridmap_pub_.publish(map_ros.map);
        gridmap_info_pub_.publish(map_ros.map.info);

        r.sleep();
    }
}

void slamwareAPP::publishPlanPath(float publish_period)
{
    if(publish_period == 0)
        return;

    ros::Rate r(1.0 / publish_period);
    while(ros::ok())
    {
        rpos::actions::MoveAction moveAction = SDP_.getCurrentAction();

        if(moveAction)
        {
            rpos::features::motion_planner::Path path = moveAction.getRemainingPath();
            std::vector<rpos::core::Location> locations = path.getPoints();

            //create a path message
            nav_msgs::Path paths;
            paths.poses.resize(locations.size());
            paths.header.frame_id = map_frame_;
            paths.header.stamp = ros::Time();

            for(int i = 0; i< locations.size(); i++)
            {
                geometry_msgs::PoseStamped pose_stamp;
                pose_stamp.header.frame_id = map_frame_;
                pose_stamp.header.stamp = ros::Time();
                pose_stamp.pose.position.x = locations[i].x();
                pose_stamp.pose.position.y = locations[i].y();
                pose_stamp.pose.position.z = 0.0;
                pose_stamp.pose.orientation.w = 1;
                pose_stamp.pose.orientation.x = 0;
                pose_stamp.pose.orientation.y = 0;
                pose_stamp.pose.orientation.z = 0;
                paths.poses[i] = pose_stamp;
            }
            global_plan_pub_.publish(paths);
        }
        r.sleep();
    }
}

void slamwareAPP::robotControlCallback (const geometry_msgs::TwistConstPtr &vel)
{
    //std::cout<<" "<<vel->linear.x <<"  "<<vel->angular.z<<std::endl;
    if(vel->linear.x > 0.005f)
        rpos::actions::MoveAction act = SDP_.moveBy(ACTION_DIRECTION::FORWARD);
    else   if(vel->linear.x < -0.005f)
        rpos::actions::MoveAction act = SDP_.moveBy(ACTION_DIRECTION::BACKWARD);

    if( vel->angular.z > 0.001f)
        rpos::actions::MoveAction act = SDP_.moveBy(ACTION_DIRECTION::TURNLEFT);
    else if( vel->angular.z < -0.001f)
        rpos::actions::MoveAction act = SDP_.moveBy(ACTION_DIRECTION::TURNRIGHT);

    return;
}

void slamwareAPP::moveToGoalCallback (const geometry_msgs::PoseStamped::ConstPtr& goal)
{
    //std::cout<<" "<<goal->pose.position.x <<"  "<<goal->pose.position.y;
    rpos::actions::MoveAction moveAction = SDP_.getCurrentAction();
    boost::this_thread::sleep_for(boost::chrono::milliseconds(2000));
    moveAction = SDP_.moveTo(rpos::core::Location(goal->pose.position.x, goal->pose.position.y), false, true);

    return;
}

