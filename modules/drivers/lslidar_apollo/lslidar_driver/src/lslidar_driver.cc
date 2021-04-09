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

#include <string>
#include <cmath>
#include <unistd.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <poll.h>
#include <errno.h>
#include <fcntl.h>
#include <sys/file.h>

#include <ros/ros.h>
#include <tf/transform_listener.h>

#include <lslidar_driver/lslidar_driver.h>

namespace apollo {
namespace drivers {
namespace lslidar_driver {

LslidarDriver::LslidarDriver(
        ros::NodeHandle& n, ros::NodeHandle& pn):
    nh(n),
    pnh(pn),
    socket_id(-1){
}

LslidarDriver::~LslidarDriver() {
    (void) close(socket_id);
}

bool LslidarDriver::loadParameters() {
    pnh.param("frame_id", frame_id, std::string("lslidar"));
    pnh.param("device_ip", device_ip_string, std::string("192.168.1.222"));
    pnh.param<int>("device_port", UDP_PORT_NUMBER, 2368);
    inet_aton(device_ip_string.c_str(), &device_ip);
    ROS_INFO_STREAM("Opening UDP socket: address " << device_ip_string);
    ROS_INFO_STREAM("Opening UDP socket: port " << UDP_PORT_NUMBER);
    return true;
}

bool LslidarDriver::createRosIO() {
    // Output lidar_packet publisher
    packet_pub = nh.advertise<lslidar_msgs::LslidarPacket>(
                "lslidar_packet", 100);
    return true;
}

bool LslidarDriver::openUDPPort() {
    socket_id = socket(PF_INET, SOCK_DGRAM, 0);
    if (socket_id == -1) {
        perror("socket");
        return false;
    }

    sockaddr_in my_addr;                     // my address information
    memset(&my_addr, 0, sizeof(my_addr));    // initialize to zeros
    my_addr.sin_family = AF_INET;            // host byte order
    my_addr.sin_port = htons(UDP_PORT_NUMBER);      // short, in network byte order
    ROS_INFO_STREAM("Opening UDP socket: port " << UDP_PORT_NUMBER);
    my_addr.sin_addr.s_addr = INADDR_ANY;    // automatically fill in my IP

    if (bind(socket_id, (sockaddr *)&my_addr, sizeof(sockaddr)) == -1) {
        perror("bind");                 // TODO: ROS_ERROR errno
        return false;
    }

    if (fcntl(socket_id, F_SETFL, O_NONBLOCK|FASYNC) < 0) {
        perror("non-block");
        return false;
    }

    return true;
}

bool LslidarDriver::initialize() {

    this->initTimeStamp();

    if (!loadParameters()) {
        ROS_ERROR("Cannot load all required ROS parameters...");
        return false;
    }

    if (!createRosIO()) {
        ROS_ERROR("Cannot create all ROS IO...");
        return false;
    }

    if (!openUDPPort()) {
        ROS_ERROR("Cannot open UDP port...");
        return false;
    }
    ROS_INFO("Initialised lslidar c16 without error");
    return true;
}

int LslidarDriver::getPacket(
        lslidar_msgs::LslidarPacketPtr& packet) {

    double time1 = ros::Time::now().toSec();

    struct pollfd fds[1];
    fds[0].fd = socket_id;
    fds[0].events = POLLIN;
    static const int POLL_TIMEOUT = 2000; // one second (in msec)

    sockaddr_in sender_address;
    socklen_t sender_address_len = sizeof(sender_address);

    while (true)
    {
        do {
            // poll() until input available
            int retval = poll(fds, 1, POLL_TIMEOUT);
            if (retval < 0)             // poll() error?
            {
                if (errno != EINTR)
                    ROS_ERROR("poll() error: %s", strerror(errno));
                return 1;
            }
            if (retval == 0)            // poll() timeout?
            {
                ROS_WARN("lslidar poll() timeout");
                return 1;
            }
            if ((fds[0].revents & POLLERR)
                    || (fds[0].revents & POLLHUP)
                    || (fds[0].revents & POLLNVAL)) // device error?
            {
                ROS_ERROR("poll() reports lslidar error");
                return 1;
            }
        } while ((fds[0].revents & POLLIN) == 0);

        // Receive packets that should now be available from the
        // socket using a blocking read.
        ssize_t nbytes = recvfrom(socket_id, &packet->data[0], PACKET_SIZE,  0,
                (sockaddr*) &sender_address, &sender_address_len);

        if (nbytes < 0)
        {
            if (errno != EWOULDBLOCK)
            {
                perror("recvfail");
                ROS_INFO("recvfail");
                return 1;
            }
        }
        else if ((size_t) nbytes == PACKET_SIZE)
        {
            // read successful,
            // if packet is not from the lidar scanner we selected by IP,
            // continue otherwise we are done
            if( device_ip_string != "" && sender_address.sin_addr.s_addr != device_ip.s_addr )
                continue;
            else
                break; //done
        }
    }

    // get GPS and FPGA timestamp from packet
    this->getFPGA_GPSTimeStamp(packet);
    packet->stamp = this->timeStamp;

    return 0;
}

bool LslidarDriver::polling()
{
    // Allocate a new shared pointer for zero-copy sharing with other nodelets.
    lslidar_msgs::LslidarPacketPtr packet(
                new lslidar_msgs::LslidarPacket());

    while (true)
    {
        // keep reading until full packet received
        int rc = getPacket(packet);
        if (rc == 0) break;       // got a full packet?
        if (rc < 0) return false; // end of file reached?
    }

    // publish message using time of last packet read
    // ROS_DEBUG("Publishing a full lslidar scan.");
    packet_pub.publish(*packet);
    return true;
}

void LslidarDriver::initTimeStamp(void)
{
    int i;

    for(i = 0;i < 10;i ++)
    {
        this->packetTimeStamp[i] = 0;
    }
    this->pointcloudTimeStamp = 0;

    this->timeStamp = ros::Time(0.0);
}

void LslidarDriver::getFPGA_GPSTimeStamp(lslidar_msgs::LslidarPacketPtr &packet)
{
    unsigned char head2[] = {packet->data[0],packet->data[1],packet->data[2],packet->data[3]};

    if(head2[0] == 0xA5 && head2[1] == 0xFF)
    {
        if(head2[2] == 0x00 && head2[3] == 0x5A)
        {
            this->packetTimeStamp[4] = packet->data[41];
            this->packetTimeStamp[5] = packet->data[40];
            this->packetTimeStamp[6] = packet->data[39];
            this->packetTimeStamp[7] = packet->data[38];
            this->packetTimeStamp[8] = packet->data[37];
            this->packetTimeStamp[9] = packet->data[36];
        }
    }
    else if(head2[0] == 0xFF && head2[1] == 0xEE)
    {
        uint64_t packet_timestamp;
        packet_timestamp = (packet->data[1200]  +
                        packet->data[1201] * pow(2, 8) +
                        packet->data[1202] * pow(2, 16) +
                        packet->data[1203] * pow(2, 24)) * 1e3;

        cur_time.tm_sec = this->packetTimeStamp[4];
        cur_time.tm_min = this->packetTimeStamp[5];
        cur_time.tm_hour = this->packetTimeStamp[6];
        cur_time.tm_mday = this->packetTimeStamp[7];
        cur_time.tm_mon = this->packetTimeStamp[8]-1;
        cur_time.tm_year = this->packetTimeStamp[9]+2000-1900;
        this->pointcloudTimeStamp = static_cast<uint64_t>(timegm(&cur_time));

	    // timeStamp = ros::Time(this->pointcloudTimeStamp+total_us/10e5);

        timeStamp = ros::Time(this->pointcloudTimeStamp, packet_timestamp);
        ROS_DEBUG("ROS TS: %f, GPS: y:%d m:%d d:%d h:%d m:%d s:%d; FPGA: us:%lu",
        timeStamp.toSec(),
        cur_time.tm_year,cur_time.tm_mon,cur_time.tm_mday,cur_time.tm_hour,cur_time.tm_min,cur_time.tm_sec,
        packet_timestamp);

    }
}

}  // namespace lslidar_driver
}  // namespace drivers
}  // namespace apollo
