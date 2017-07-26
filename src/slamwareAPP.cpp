/**
 kint.zhao  huasheng_zyh@163.com
   2017.0721
*/
#include  "slamwareAPP.h"

slamwareAPP::slamwareAPP(std::string ip):ip_addres_(ip),
    nh_("~"), robot_pose_pub_thread_(NULL),scan_pub_thread_(NULL),map_pub_thread_(NULL),plan_path_pub_thread_(NULL)
{
    init();
}

slamwareAPP::slamwareAPP():ip_addres_( "192.168.11.1"),
    nh_("~"), robot_pose_pub_thread_(NULL),scan_pub_thread_(NULL),map_pub_thread_(NULL),plan_path_pub_thread_(NULL)
{
    init();
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

void slamwareAPP::init()
{
    //ip
    if(!nh_.getParam("ip_addres", ip_addres_))
    {
        ip_addres_ = "192.168.11.1";
    }

    //laser compensate
    if(!nh_.getParam("angle_compensate", angle_compensate_))
    {
        angle_compensate_ = true;
    }

    if(!nh_.getParam("fixed_odom_map_tf", fixed_odom_map_tf_))
    {
        fixed_odom_map_tf_ = true;
    }

    //frame
    if(!nh_.getParam("robot_frame", robot_frame_))
        robot_frame_ = "/base_link";

    if(!nh_.getParam("odom_frame", odom_frame_))
        odom_frame_ = "/odom";

    if(!nh_.getParam("laser_frame", laser_frame_))
        laser_frame_ = "/laser";

    if(!nh_.getParam("map_frame", map_frame_))
        map_frame_ = "/map";

    //topic
    if(!nh_.getParam("vel_control_topic", vel_control_topic_))
        vel_control_topic_ = "/cmd_vel";

    if(!nh_.getParam("goal_topic", goal_topic_))
        goal_topic_ = "/move_base_simple/goal";

    if(!nh_.getParam("scan_topic", scan_topic_))
        scan_topic_= "scan";

    if(!nh_.getParam("odom_topic", odom_topic_))
        odom_topic_="odom";

    if(!nh_.getParam("map_topic", map_topic_))
        map_topic_ = "map";

    if(!nh_.getParam("map_info_topic", map_info_topic_))
        map_info_topic_ = "map_metadata";

    if(!nh_.getParam("path_topic", path_topic_))
        path_topic_ ="global_plan_path";

    //param
    if(!nh_.getParam("robot_pose_pub_period", robot_pose_pub_period_))
        robot_pose_pub_period_ = 0.05;
    if(!nh_.getParam("scan_pub_period", scan_pub_period_))
        scan_pub_period_ = 0.1;
    if(!nh_.getParam("map_pub_period", map_pub_period_))
        map_pub_period_ = 0.2;
    if(!nh_.getParam("path_pub_period", path_pub_period_))
        path_pub_period_ = 0.05;

    if(!nh_.getParam("map_size_down_left_x", map_size_down_left_x_))
        map_size_down_left_x_ = -20;
    if(!nh_.getParam("map_size_down_left_y", map_size_down_left_y_))
        map_size_down_left_y_ = -20;
    if(!nh_.getParam("map_size_width", map_size_width_))
        map_size_width_ = 40;
    if(!nh_.getParam("map_size_height", map_size_height_))
        map_size_height_ = 40;
}

bool slamwareAPP::connectSlamware()
{
    try {
        SDP_ = SlamwareCorePlatform::connect(ip_addres_, 1445);
        std::cout <<" ===> SDK Version: " << SDP_.getSDKVersion() << std::endl;
        std::cout <<" ===> SDP Version: " << SDP_.getSDPVersion() << std::endl;
    } catch(ConnectionTimeOutException& e) {
        std::cout <<" ===>!!! connect fail, TIMEOUT! " << std::endl;
        return false;
    } catch(ConnectionFailException& e) {
        std::cout <<" ===>!!! connect fail, Fail! " << std::endl;
        return false;
    }
    std::cout <<" ===> Connection Successfully" << std::endl;
    return true;
}

void slamwareAPP::loopThreads()
{
    scan_pub_ = nh_.advertise<sensor_msgs::LaserScan>(scan_topic_, 10);
    robot_pose_pub_ = nh_.advertise<nav_msgs::Odometry> (odom_topic_, 10);
    gridmap_pub_ = nh_.advertise<nav_msgs::OccupancyGrid> (map_topic_, 1, true);
    gridmap_info_pub_ = nh_.advertise<nav_msgs::MapMetaData>(map_info_topic_, 1, true);
    global_plan_pub_ = nh_.advertise<nav_msgs::Path>(path_topic_, 2);

    robot_vel_sub_ = nh_.subscribe (vel_control_topic_, 10, &slamwareAPP::robotControlCallback, this);
    goal_sub_ = nh_.subscribe (goal_topic_, 1, &slamwareAPP::moveToGoalCallback, this);

    robot_pose_pub_thread_ = new boost::thread(boost::bind(&slamwareAPP::publishRobotPose, this, robot_pose_pub_period_));
    scan_pub_thread_ = new boost::thread(boost::bind(&slamwareAPP::publishScan, this, scan_pub_period_));
    map_pub_thread_ = new boost::thread(boost::bind(&slamwareAPP::publishMap, this, map_pub_period_));
    plan_path_pub_thread_ = new boost::thread(boost::bind(&slamwareAPP::publishPlanPath, this, path_pub_period_));
}

void slamwareAPP::startSlamwareWork()
{
    loopThreads();
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
        if(fixed_odom_map_tf_)//only for debug rosrun
        {
            tfB_.sendTransform ( tf::StampedTransform (identy_transform, ros::Time::now (), map_frame_,
                                      odom_frame_));
            tfB_.sendTransform ( tf::StampedTransform (identy_transform, ros::Time::now (), robot_frame_,
                                          laser_frame_));
        }

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

        const std::vector<rpos::core::LaserPoint>& points = ls.getLaserPoints();
        if(points.empty())// skip publish
            continue;

        sensor_msgs::LaserScan scan_msg;

        scan_msg.header.stamp = start_scan_time;
        scan_msg.header.frame_id = laser_frame_;
        scan_msg.scan_time = scan_time;
        scan_msg.time_increment = scan_time / (double)(points.size()-1);
        scan_msg.range_min = 0.15;
        scan_msg.range_max = 8.0;

        if (angle_compensate_)
        {
            const int angle_compensate_multiple = 1;
            const int angle_compensate_nodes_count = 360*angle_compensate_multiple;
            bool firstValidPoint = false;
            int angle_compensate_offset_index = 0;
            std::vector<rpos::core::LaserPoint> angle_compensate_points(angle_compensate_nodes_count, rpos::core::LaserPoint(0.0,0.0,false));
            std::cout<<"points.size() : "<<points.size() <<std::endl;
            for(int i=0; i< points.size(); i++)
            {
               if(points[i].valid())
               {
                   int angle_index = (int)(points[i].angle() * angle_compensate_multiple * 180.0 / M_PI);
                   std::cout<<"angle_index  "<<angle_index<<std::endl;
                   if(!firstValidPoint)
                   {
                       angle_compensate_offset_index = angle_index;
                       firstValidPoint = true;
                   }

                   for (int j = 0; j < angle_compensate_multiple; j++) {
                       angle_compensate_points[angle_compensate_offset_index -angle_index + j] = points[i];
                   }
               }
            }

            scan_msg.ranges.resize(angle_compensate_nodes_count);
            scan_msg.intensities.resize(angle_compensate_nodes_count);
            for(int i = 0; i< angle_compensate_points.size(); i++)
            {
                if(!angle_compensate_points[i].valid())
                {
                    scan_msg.ranges[i] = std::numeric_limits<float>::infinity();
                }
                else
                {
                    scan_msg.ranges[i] = angle_compensate_points[i].distance();
                }
            }

            bool reversed = (points.front().angle() > points.back().angle());
            if ( reversed ) {
              scan_msg.angle_min =  M_PI;
              scan_msg.angle_max =  -M_PI;
            } else {
              scan_msg.angle_min =  -M_PI;
              scan_msg.angle_max =  M_PI;
            }
            scan_msg.angle_increment =
                (scan_msg.angle_max - scan_msg.angle_min) / (double)(angle_compensate_nodes_count-1);
        }
        else
        {
            scan_msg.intensities.resize(points.size());
            scan_msg.ranges.resize(points.size());

            for(int i = 0; i< points.size(); i++)
            {
                if(!points[i].valid())
                {
                    scan_msg.ranges[i] = std::numeric_limits<float>::infinity();
                }
                else
                {
                    scan_msg.ranges[i] = points[i].distance();
                }
            }
            scan_msg.angle_min =  points.front().angle();
            scan_msg.angle_max =  points.back().angle();
            scan_msg.angle_increment = (scan_msg.angle_max - scan_msg.angle_min) / (double)(points.size()-1);
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
        Map map = SDP_.getMap(MapTypeBitmap8Bit, rpos::core::RectangleF(map_size_down_left_x_, map_size_down_left_y_, map_size_width_, map_size_height_),
                              rpos::features::location_provider::EXPLORERMAP);
        std::vector<_u8> & mapData = map.getMapData();
        if(mapData.empty())// skip publish
            continue;

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
            const std::vector<rpos::core::Location>& locations = path.getPoints();

            if(locations.empty())// skip publish
                continue;

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
    rpos::actions::MoveAction moveAction = SDP_.getCurrentAction();
    moveAction = SDP_.getCurrentAction();
    boost::this_thread::sleep_for(boost::chrono::milliseconds(2000));
    moveAction = SDP_.moveTo(rpos::core::Location(goal->pose.position.x, goal->pose.position.y), false, true);

    while(ros::ok() && moveAction.getStatus() != rpos::core::ActionStatusFinished)
    {
        boost::this_thread::sleep_for(boost::chrono::milliseconds(500));
    }

    float yaw = tf::getYaw(goal->pose.orientation);
    rpos::actions::MoveAction act = SDP_.rotateTo(Rotation(yaw,0.0,0.0));
    return;
}

