/**
 kint.zhao  huasheng_zyh@163.com
   2017.0721
*/
#include  "slamwareAPP.h"

slamwareAPP::slamwareAPP(std::string ip):workOk(false),ip_addres_(ip),
    nh_("~"), robot_pose_pub_thread_(NULL),scan_pub_thread_(NULL),map_pub_thread_(NULL)
{

}

slamwareAPP::slamwareAPP():workOk(false), ip_addres_("192.168.11.1"),
    nh_("~"), robot_pose_pub_thread_(NULL),scan_pub_thread_(NULL),map_pub_thread_(NULL)
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
    robot_frame_ = "/odom";
    laser_frame_ = "/laser";
    map_frame_ = "/map";

    robot_pose_pub_period_ = 0.01;
    scan_pub_period_ = 0.1;
    map_pub_period_ = 0.2;

    scan_pub_ = nh_.advertise<sensor_msgs::LaserScan>("scan", 10);
    robot_pose_pub_ = nh_.advertise<nav_msgs::Odometry> ("odom", 10);
    gridmap_pub_ = nh_.advertise<nav_msgs::OccupancyGrid> ("map", 1, true);
    gridmap_info_pub_ = nh_.advertise<nav_msgs::MapMetaData>("map_metadata", 1, true);
    //robot_vel_sub_ = nh_.subscribe ("/cmd_vel", 10, robotVelocityCallback);

    robot_pose_pub_thread_ = new boost::thread(boost::bind(&slamwareAPP::publishRobotPose, this, robot_pose_pub_period_));
    scan_pub_thread_ = new boost::thread(boost::bind(&slamwareAPP::publishScan, this, scan_pub_period_));
    map_pub_thread_ = new boost::thread(boost::bind(&slamwareAPP::publishMap, this, map_pub_period_));

}

void slamwareAPP::startSlamwareWork()
{
    init();
}

void slamwareAPP::publishRobotPose(double transform_publish_period)
{
    if(transform_publish_period == 0)
        return;

    ros::Rate r(1.0 / transform_publish_period);
    while(ros::ok())
    {
        rpos::core::Location location = SDP_.getLocation();
        std::cout << "Robot Location: " << std::endl;
        std::cout << "x: " << location.x() << ", ";
        std::cout << "y: " << location.y() << std::endl;

        rpos::core::Pose pose = SDP_.getPose();
        std::cout << "Robot Pose: " << std::endl;
        std::cout << "x: " << pose.x() << ", ";
        std::cout << "y: " << pose.y() << ", ";
        std::cout << "yaw: " << pose.yaw() << std::endl;

        int battPercentage = SDP_.getBatteryPercentage();
        std::cout <<"Battery: " << battPercentage << "%" << std::endl;


        nav_msgs::Odometry robotPose;
        robotPose.header.frame_id = robot_frame_;
        robotPose.header.stamp = ros::Time::now ();
        robotPose.pose.pose.position.x = pose.x();
        robotPose.pose.pose.position.y = pose.y();
        robotPose.pose.pose.orientation = tf::createQuaternionMsgFromYaw (
                    pose.yaw());

        robot_pose_pub_.publish (robotPose);
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
        scan_msg.angle_min =  points.begin()->angle();
        scan_msg.angle_max =  points.end()->angle();

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
            std::cout << "distance: " << points[i].distance() << ", angle: " << points[i].angle() << ", valid: " << (points[i].valid()?"true":"false") << std::endl;
        }
        scan_pub_.publish(scan_msg);
    }

}

void slamwareAPP::publishMap(double transform_publish_period)
{
    if(transform_publish_period == 0)
        return;

    ros::Rate r(1.0 / transform_publish_period);
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

//void robotVelocityCallback(const sensor_msgs::LaserScan::ConstPtr& scan)
//{
//    std::cout<<"subscribe vel "<<std::endl;
//}

