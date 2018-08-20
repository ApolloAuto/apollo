/***************************************************************************
Copyright 2018 The Apollo Authors. All Rights Reserved                     /
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

#ifndef LSLIDAR_DRIVER_H
#define LSLIDAR_DRIVER_H

#include <unistd.h>
#include <stdio.h>
#include <netinet/in.h>
#include <string>
#include <time.h>

#include <boost/shared_ptr.hpp>

#include <ros/ros.h>
#include <std_msgs/UInt64.h>

#include <lslidar_msgs/LslidarPacket.h>

namespace apollo {
namespace drivers {
namespace lslidar_driver {

//static uint16_t UDP_PORT_NUMBER = 8080;
static uint16_t PACKET_SIZE = 1206;

class LslidarDriver {
public:

    LslidarDriver(ros::NodeHandle& n, ros::NodeHandle& pn);
    ~LslidarDriver();

    bool initialize();
    bool polling();

    void initTimeStamp(void);
    void getFPGA_GPSTimeStamp(lslidar_msgs::LslidarPacketPtr &packet);

    typedef boost::shared_ptr<LslidarDriver> LslidarDriverPtr;
    typedef boost::shared_ptr<const LslidarDriver> LslidarDriverConstPtr;

private:

    bool loadParameters();
    bool createRosIO();
    bool openUDPPort();
    int getPacket(lslidar_msgs::LslidarPacketPtr& msg);

    // Ethernet related variables
    std::string device_ip_string;
    in_addr device_ip;
    int UDP_PORT_NUMBER;
    int socket_id;

    // ROS related variables
    ros::NodeHandle nh;
    ros::NodeHandle pnh;

    std::string frame_id;
    ros::Publisher packet_pub;

    uint64_t pointcloudTimeStamp;
    unsigned char packetTimeStamp[10];
    struct tm cur_time;
    unsigned short int us;
    unsigned short int ms;
    ros::Time timeStamp;
};

typedef LslidarDriver::LslidarDriverPtr LslidarDriverPtr;
typedef LslidarDriver::LslidarDriverConstPtr LslidarDriverConstPtr;

} // namespace lslidar_driver
}
}
#endif // _LSLIDAR_DRIVER_H_
