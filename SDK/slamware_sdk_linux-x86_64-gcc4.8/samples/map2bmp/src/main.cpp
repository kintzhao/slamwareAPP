/*
 * Copyright (C) 2014 SLAMTEC Co., Ltd.
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 */


/**
* SLAMWARE Map Dump Utility 
* The current built map will be saved with BMP format
*  
* By CSK
* Copyright (c) 2014 Shanghai SlamTec Co., Ltd.
*/

// runtime support
#include <cstdio>
#include <cstring>
#include <string>

// SLAMWARE necessary headers
#include <rpos/rpos.h>
#include <rpos/robot_platforms/slamware_core_platform.h>
#include <rpos/features/location_provider/map.h>

// C++ Bitmap Library from http://www.partow.net/
#include "bitmap_image.hpp"

using namespace std;
using namespace rpos::robot_platforms;
using namespace rpos::features;
using namespace rpos::features::location_provider;
using namespace rpos::system::detail;
using namespace rpos::system::types;

#define DEFAULT_DUMPIMAGE_FILENAME "slamware_mapdump"
#define DEFAULT_SDP_PORT           1445

SlamwareCorePlatform sdp;

void showHelp(int argc, const char * argv[])
{
    printf("SLAMWARE Map Dump Utility\n\n"
           "USAGE:\n"
           "%s [OPTS] [-o filename] <SDP IP Address>\n"
           "SDP IP Address      The ip address string of the SLAMWARE SDP.\n"
           "-o  filename        Bitmap filename to be used to save the dumped map.\n"
           "                    If not specified, the default name %s will be used.\n"
           "-p  port            SDP Server port.\n"
           "                    No file will be generated.\n"
           "                    By default, the port %d will be used.\n"
           "--txt-format        The dumped file will be saved in Text format instead of BMP.\n"
           "--info-only         Only display map size and related info. \n"
           "-h                  Show this message.\n"
           , argv[0], DEFAULT_DUMPIMAGE_FILENAME, DEFAULT_SDP_PORT);
}


bool saveToBmp(const char * filename, Map map)
{
    std::string finalFilename = filename;
    finalFilename += ".bmp";

    bitmap_image mapBitmap(map.getMapDimension().x(), map.getMapDimension().y());

    for (size_t posY = 0; posY < map.getMapDimension().y(); ++posY)
    {
        for (size_t posX = 0; posX < map.getMapDimension().x(); ++posX)
        {
            rpos::system::types::_u8 cellValue = ((rpos::system::types::_u8)128U) + map.getMapData()[posX + (map.getMapDimension().y()-posY-1) * map.getMapDimension().x()];
            mapBitmap.set_pixel(posX, posY, cellValue, cellValue, cellValue);
        }
    }
    mapBitmap.save_image(finalFilename);
    return true;
}

bool saveToTxt(const char * filename, Map map)
{
    std::string finalFilename = filename;
    finalFilename += ".txt";

    FILE * fp = fopen(finalFilename.c_str(), "w");
    if (!fp) {
        fprintf(stderr, "Cannot create the file %s.\n", finalFilename.c_str());
        return false;
    }

    fprintf(fp, "# SLAMWARE MAP FILE\n");
    fprintf(fp, "Area: %.4f, %.4f, %.4f, %.4f\n", map.getMapArea().x(), map.getMapArea().y(), map.getMapArea().width(), map.getMapArea().height());
    fprintf(fp, "Cell Resolution: %.4f, %.4f\n", map.getMapResolution().x(), map.getMapResolution().y());
    fprintf(fp, "Cell Dimension: %lu, %lu\n", map.getMapDimension().x(), map.getMapDimension().y());


    for (size_t posY = 0; posY < map.getMapDimension().y(); ++posY)
    {
        for (size_t posX = 0; posX < map.getMapDimension().x(); ++posX)
        {
            fprintf(fp, "%d, ",  (char)map.getMapData()[posX + posY *map.getMapDimension().x() ]);

        }
        fprintf(fp, "\n");
    }

    fclose(fp);

    return true;
}

void displayMapInfo(Map map)
{
    printf("> Map Area: (%.4f,%.4f,%.4f,%.4f)\n"
        , map.getMapArea().left()
        , map.getMapArea().top()
        , map.getMapArea().right()
        , map.getMapArea().bottom());

    printf("> Cell Dimension: (%u,%u)\n"
        , map.getMapDimension().x()
        , map.getMapDimension().y());

    printf("> Cell Resolution: (%.4f,%.4f)\n"
        , map.getMapResolution().x()
        , map.getMapResolution().y());

    printf("> Timestamp: %ull\n"
        , map.getMapTimestamp());
}

void displayLocalizationInfo(const rpos::core::Pose & pose)
{
    printf("> Position: (%.4f,%.4f,%.4f)\n"
        , pose.x(), pose.y(), pose.z());

    printf("> Heading: %.4f\n"
        , pose.yaw()*180.0f/M_PI);
}


int main(int argc, const char * argv[])
{
    bool         opt_needShowHelp = false;
    bool         opt_noSave       = false;
    bool         opt_txtFileMode  = false;
    const char * opt_sdpIPAddress = NULL;
    const char * opt_dumpImageFilename = DEFAULT_DUMPIMAGE_FILENAME;
    int          opt_sdpPort = DEFAULT_SDP_PORT;

    // parse the command line...
    for (int currentArg = 1;  currentArg< argc; ++currentArg) 
    {
        if (strcmp(argv[currentArg], "-o") == 0) {
            if (++currentArg < argc) {
                opt_dumpImageFilename = argv[currentArg];
            }
        } else if (strcmp(argv[currentArg], "-h") == 0) {
            opt_needShowHelp = true;
        } else if (strcmp(argv[currentArg], "--txt-format") == 0) {
            opt_txtFileMode = true;
        } else if (strcmp(argv[currentArg], "--info-only") == 0) {
            opt_noSave = true;
        } else if (strcmp(argv[currentArg], "-p") == 0) {
            if (++currentArg < argc) {
                opt_sdpPort = atoi(argv[currentArg]);
            } else {
                opt_needShowHelp = true;
            }
        }else {
            opt_sdpIPAddress = argv[currentArg];
        }
    }


    if (opt_needShowHelp || !opt_sdpIPAddress) {
        showHelp(argc, argv);
        return -1;
    }

    
    
    try {
        printf("Connecting to the SDP @ %s...\n", opt_sdpIPAddress);
        sdp = SlamwareCorePlatform::connect(opt_sdpIPAddress, opt_sdpPort);

        printf("Fetching Map Info...\n");
        rpos::core::RectangleF knownArea = sdp.getKnownArea(MapTypeBitmap8Bit, rpos::features::location_provider::EXPLORERMAP);
        Map map = sdp.getMap(MapTypeBitmap8Bit, knownArea, rpos::features::location_provider::EXPLORERMAP);
        displayMapInfo(map);

        printf("Fetching Localization Info...\n");
        rpos::core::Pose robotPose = sdp.getPose();
        displayLocalizationInfo(robotPose);

        if (opt_noSave) {
            return 0;
        }

        // dump the map data to file....

        if (opt_txtFileMode) {
            if (!saveToTxt(opt_dumpImageFilename, map)) {
                return -3;
            }
        } else {
            if (!saveToBmp(opt_dumpImageFilename, map)) {
                return -3;
            }
        }

        return 0;
    } catch (ExceptionBase & e) {
        fprintf(stderr, "Failed: $s\n", e.what());
        return -2;
    }



    
}
