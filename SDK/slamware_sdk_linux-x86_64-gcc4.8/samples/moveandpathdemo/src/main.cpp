/**
* Slamtec Robot Go Action and Get Path Demo
*
* Created By Jacky Li @ 2014-8-8
* Copyright (c) 2014 Shanghai SlamTec Co., Ltd.
*/
#include <regex>
#include <boost/thread.hpp>
#include <boost/chrono.hpp>
#include <rpos/robot_platforms/slamware_core_platform.h>
#include <rpos/features/location_provider/map.h>

using namespace rpos::robot_platforms;
using namespace rpos::features;
using namespace rpos::features::location_provider;

std::string ipaddress = "";
const char *ipReg = "\\d{1,3}\\.\\d{1,3}\\.\\d{1,3}\\.\\d{1,3}";

void showHelp(std::string appName)
{
    std::cout << "SLAMWARE console demo." << std::endl << \
        "Usage: " << appName << " <slamware_address>" << std::endl;
}

bool parseCommandLine(int argc, const char * argv[])
{
    bool opt_show_help = false;

    for (int pos=1; pos<argc; ++pos )
    {
        const char * current = argv[pos];
        if(strcmp(current, "-h") == 0) {
            opt_show_help = true;
        } else {
            ipaddress = current;
        }
    }

    std::regex reg(ipReg);
    if(! opt_show_help && ! std::regex_match(ipaddress, reg)) 
    {
        opt_show_help = true;
    }

    if (opt_show_help)
    {
        showHelp("moveandpathdemo");
        return false;
    }

    return true;
}

int main(int argc, const char * argv[])
{
    if(! parseCommandLine(argc, argv) )
    {
        return 1;
    }

    std::vector<rpos::core::Location> pointsToGo;
    pointsToGo.push_back(rpos::core::Location(2, 2));
    pointsToGo.push_back(rpos::core::Location(-2, 2));
    pointsToGo.push_back(rpos::core::Location(-2, -2));
    pointsToGo.push_back(rpos::core::Location(2, -2));

    SlamwareCorePlatform sdp;
    try {
        sdp = SlamwareCorePlatform::connect(argv[1], 1445);
        std::cout <<"SDK Version: " << sdp.getSDKVersion() << std::endl;
        std::cout <<"SDP Version: " << sdp.getSDPVersion() << std::endl;
    } catch(ConnectionTimeOutException& e) {
        std::cout <<e.what() << std::endl;
        return 1;
    } catch(ConnectionFailException& e) {
        std::cout <<e.what() << std::endl;
        return 1;
    }
    std::cout <<"Connection Successfully" << std::endl;

    int i = 0;
    while(true)
    {
        try
        {
            rpos::actions::MoveAction moveAction = sdp.getCurrentAction();

            if(moveAction)
            {
                std::cout << (moveAction.isEmpty()? "Empty" : "Non-Empty") << std::endl;
                std::cout << "Action ID: " << moveAction.getActionId() << std::endl;
                std::cout << "Action Name: " << moveAction.getActionName() << std::endl;
                boost::this_thread::sleep_for(boost::chrono::milliseconds(2000));

                if(moveAction.getStatus()==rpos::core::ActionStatusFinished) 
                {
                    moveAction = sdp.moveTo(pointsToGo, false, true);
                }
            }
            else
            {
                std::cout << "Empty" << std::endl;
                std::cout << "Action Name: " << "EmptyAction" << std::endl;
                boost::this_thread::sleep_for(boost::chrono::milliseconds(2000));

                moveAction = sdp.moveTo(pointsToGo, false, true);
            }

            while(moveAction)
            {
                std::cout << (moveAction.isEmpty()? "Empty" : "Non-Empty") << std::endl;
                std::cout << "Action ID: " << moveAction.getActionId() << std::endl;
                std::cout << "Action Name: " << moveAction.getActionName() << std::endl;
                boost::this_thread::sleep_for(boost::chrono::milliseconds(2000));

                if(i%4==2) {
                    moveAction.cancel();
                } else if(i%4==3) {
                    std::cout << "Wait until done" << std::endl;
                    moveAction.waitUntilDone();
                    std::cout << "Done" << std::endl;
                }

                boost::this_thread::sleep_for(boost::chrono::milliseconds(2000));
                rpos::features::motion_planner::Path milestones = moveAction.getRemainingMilestones();
                std::vector<rpos::core::Location> points = milestones.getPoints();

                std::cout << "Remaining Milestones: " << std::endl;
                for(std::vector<rpos::core::Location>::const_iterator it=points.begin(); it!=points.end(); it++)
                {
                    std::cout << "(" << it->x() << ", ";
                    std::cout << it->y() << ")" << std::endl;
                }
                boost::this_thread::sleep_for(boost::chrono::milliseconds(2000));

                rpos::features::motion_planner::Path path = moveAction.getRemainingPath();
                std::vector<rpos::core::Location> locations = path.getPoints();

                if(locations.size()<=0)
                {
                    break;
                }

                std::cout << "Remaining Path: " << std::endl;
                for(std::vector<rpos::core::Location>::const_iterator it=locations.begin(); it!=locations.end(); it++)
                {
                    std::cout << "(" << it->x() << ", ";
                    std::cout << it->y() << ")" << std::endl;
                }

                rpos::core::Location location = sdp.getLocation();
                std::cout << "Robot Location: " << std::endl;
                std::cout << "x: " << location.x() << ", ";
                std::cout << "y: " << location.y() << std::endl;

                rpos::core::Pose pose = sdp.getPose();
                std::cout << "Robot Pose: " << std::endl;
                std::cout << "x: " << pose.x() << ", ";
                std::cout << "y: " << pose.y() << ", ";
                std::cout << "yaw: " << pose.yaw() << std::endl;

                int battPercentage = sdp.getBatteryPercentage();
                std::cout <<"Battery: " << battPercentage << "%" << std::endl;
            }
        } catch(ConnectionFailException e) {
            std::cout << e.what() << std::endl;
            break;
        } catch(RequestTimeOutException& e) {
            std::cout << e.what() << std::endl;
        }

        i++;
    }

    return 0;
}