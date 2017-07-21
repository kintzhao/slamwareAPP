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
using namespace rpos::system::types;

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
        showHelp("mapdemo");
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

    SlamwareCorePlatform sdp;
    try {
        sdp = SlamwareCorePlatform::connect(ipaddress, 1445);
        std::cout <<"SDK Version: " << sdp.getSDKVersion() << std::endl;
        std::cout <<"SDP Version: " << sdp.getSDPVersion() << std::endl;
        std::cout <<"Bartter Status: " << sdp.getBatteryIsCharging() << std::endl;
        std::cout <<"Barttery Percetage: " << sdp.getBatteryPercentage() << std::endl;
        std::cout <<"Power Status: " << sdp.getDCIsConnected() << std::endl;
    } catch(ConnectionTimeOutException& e) {
        std::cout <<e.what() << std::endl;
        return 1;
    } catch(ConnectionFailException& e) {
        std::cout <<e.what() << std::endl;
        return 1;
    }
    std::cout <<"Connection Successfully" << std::endl;

    try {
        Map map = sdp.getMap(MapTypeBitmap8Bit, rpos::core::RectangleF(-1, -1, 2, 2), rpos::features::location_provider::EXPLORERMAP);
        std::vector<_u8> & mapData = map.getMapData();
        std::cout << "Map Data: " << std::endl;
        for(std::vector<_u8>::const_iterator it=mapData.begin(); it!=mapData.end(); it++)
        {
            std::cout << int(*it) << " ";
        }
        std::cout << std::endl << std::endl;

        std::cout << "Map Position" << std::endl;
        std::cout << map.getMapPosition() << std::endl << std::endl;

        std::cout << "Map Dimension" << std::endl;
        std::cout << map.getMapDimension() << std::endl << std::endl;

        std::cout << "Map Resolution" << std::endl;
        std::cout << map.getMapResolution() << std::endl << std::endl;

        std::cout << "Map Area" << std::endl;
        rpos::core::RectangleF rec = map.getMapArea();

        std::cout << "x: " << rec.x() << std::endl;
        std::cout << "y: " << rec.y() << std::endl;
        std::cout << "width:  " << rec.width() << std::endl;
        std::cout << "height: " << rec.height() << std::endl << std::endl;
    } catch(ConnectionFailException e) {
        std::cout << e.what() << std::endl;
    } catch(RequestTimeOutException& e) {
        std::cout << e.what() << std::endl;
    }

    std::cout << std::endl << std::endl;

    while(true)
    {
        try {
            rpos::core::RectangleF knownArea = sdp.getKnownArea(MapTypeBitmap8Bit, rpos::features::location_provider::EXPLORERMAP);
            std::cout << "Known Area: " << std::endl;
            std::cout << "(" << knownArea.x() << ", " << knownArea.y() << ", " << knownArea.width() << ", " << knownArea.height() << ")" << std::endl;
            Map map = sdp.getMap(MapTypeBitmap8Bit, knownArea, rpos::features::location_provider::EXPLORERMAP);
            rpos::features::system_resource::LaserScan ls = sdp.getLaserScan();
            std::vector<rpos::core::LaserPoint> points = ls.getLaserPoints();
            for(std::vector<rpos::core::LaserPoint>::const_iterator it=points.begin(); it!=points.end(); it++)
            {
                std::cout << "distance: " << it->distance() << ", angle: " << it->angle() << ", valid: " << (it->valid()?"true":"false") << std::endl;
            }
        } catch(ConnectionFailException e) {
            std::cout << e.what() << std::endl;
            break;
        } catch(RequestTimeOutException& e) {
            std::cout << e.what() << std::endl;
        }
    }

    return 0;
}
