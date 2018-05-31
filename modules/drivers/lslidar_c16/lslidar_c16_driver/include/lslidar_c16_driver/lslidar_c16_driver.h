/***************************************************************************
Copyright 2018 The Leishen Authors. All Rights Reserved                     /
                                                                            /
Licensed under the Apache License, Version 2.0 (the "License");             /
you may not use this file except in compliance with the License.            /
You may obtain a copy of the License at                                     /
                                                                            /
    http://www.apache.org/licenses/LICENSE-2.0                              /
                                                                            /
Unless required by applicable law or agreed to in writing, software         /
distributed under the License is distributed on an "AS IS" BASIS,           /
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.    /
See the License for the specific language governing permissions and         /
limitations under the License.                                              /
****************************************************************************/

#ifndef LSLIDAR_C16_DRIVER_H
#define LSLIDAR_C16_DRIVER_H

#include <unistd.h>
#include <stdio.h>
#include <netinet/in.h>
#include <string>
#include <time.h>

#include <boost/shared_ptr.hpp>

#include <ros/ros.h>
#include <std_msgs/UInt64.h>
#include <diagnostic_updater/diagnostic_updater.h>
#include <diagnostic_updater/publisher.h>

#include <lslidar_c16_msgs/LslidarC16Packet.h>
#include <lslidar_c16_msgs/LslidarC16ScanUnified.h>

namespace apollo {
namespace drivers {
namespace lslidar_c16_driver {

//static uint16_t UDP_PORT_NUMBER = 8080;
static uint16_t PACKET_SIZE = 1206;

class LslidarC16Driver {
public:

    LslidarC16Driver(ros::NodeHandle& n, ros::NodeHandle& pn);
    ~LslidarC16Driver();

    bool initialize();
    bool polling();

    void initTimeStamp(void);    
    void getFPGA_GPSTimeStamp(lslidar_c16_msgs::LslidarC16PacketPtr &packet);

    typedef boost::shared_ptr<LslidarC16Driver> LslidarC16DriverPtr;
    typedef boost::shared_ptr<const LslidarC16Driver> LslidarC16DriverConstPtr;

private:

    bool loadParameters();
    bool createRosIO();
    bool openUDPPort();
    int getPacket(lslidar_c16_msgs::LslidarC16PacketPtr& msg);

    // Ethernet relate variables
    std::string device_ip_string;
    in_addr device_ip;
    int UDP_PORT_NUMBER;
    int socket_id;

    // ROS related variables
    ros::NodeHandle nh;
    ros::NodeHandle pnh;

    std::string frame_id;
    ros::Publisher packet_pub;    

    // Diagnostics updater
    diagnostic_updater::Updater diagnostics;
    boost::shared_ptr<diagnostic_updater::TopicDiagnostic> diag_topic;
    double diag_min_freq;
    double diag_max_freq;

    uint64_t pointcloudTimeStamp;
    unsigned char packetTimeStamp[10];
    struct tm cur_time;
    unsigned short int us;
    unsigned short int ms;
    ros::Time timeStamp;
};

typedef LslidarC16Driver::LslidarC16DriverPtr LslidarC16DriverPtr;
typedef LslidarC16Driver::LslidarC16DriverConstPtr LslidarC16DriverConstPtr;

} // namespace lslidar_driver
}
}
#endif // _LSLIDAR_C16_DRIVER_H_
