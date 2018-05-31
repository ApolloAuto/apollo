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

#include <lslidar_c16_driver/lslidar_c16_driver.h>

namespace apollo {
namespace drivers {
namespace lslidar_c16_driver {

LslidarC16Driver::LslidarC16Driver(
        ros::NodeHandle& n, ros::NodeHandle& pn):
    nh(n),
    pnh(pn),
    socket_id(-1){
    return;
}

LslidarC16Driver::~LslidarC16Driver() {
    (void) close(socket_id);
    return;
}

bool LslidarC16Driver::loadParameters() {

  pnh.param("frame_id", frame_id, std::string("lslidar"));
  pnh.param("device_ip", device_ip_string, std::string("192.168.1.222"));
  pnh.param<int>("device_port", UDP_PORT_NUMBER, 2368);
  inet_aton(device_ip_string.c_str(), &device_ip);
  ROS_INFO_STREAM("Opening UDP socket: address " << device_ip_string);
  ROS_INFO_STREAM("Opening UDP socket: port " << UDP_PORT_NUMBER);
  return true;
}

bool LslidarC16Driver::createRosIO() {

  // ROS diagnostics
  diagnostics.setHardwareID("Lslidar_C16");
  // c16 publishs 20*16 thousands points per second.
  // Each packet contains 12 blocks. And each block
  // contains 32 points. Together provides the
  // packet rate.
  const double diag_freq = 16*20000.0 / (12*32);
  diag_max_freq = diag_freq;
  diag_min_freq = diag_freq;
  ROS_INFO("expected frequency: %.3f (Hz)", diag_freq);

    using namespace diagnostic_updater;
    diag_topic.reset(new TopicDiagnostic(
                         "lslidar_packets", diagnostics,
                         FrequencyStatusParam(&diag_min_freq, &diag_max_freq, 0.1, 10),
                         TimeStampStatusParam()));

    // Output
    packet_pub = nh.advertise<lslidar_c16_msgs::LslidarC16Packet>(
                "lslidar_packet", 100);
    return true;
}

bool LslidarC16Driver::openUDPPort() {
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

bool LslidarC16Driver::initialize() {
    
    this->initTimeStamp();
    /*
    //time init debug
    ROS_ERROR("packet_time:%lf",this->packetTimeStamp);
    for(int i = 0;i < 10;i ++)
    {
        ROS_ERROR("FPGA_time:%ju",this->FPGATimeStamp[i]);
    }
    */

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

int LslidarC16Driver::getPacket(
        lslidar_c16_msgs::LslidarC16PacketPtr& packet) {

    double time1 = ros::Time::now().toSec();

    struct pollfd fds[1];
    fds[0].fd = socket_id;
    fds[0].events = POLLIN;
  static const int POLL_TIMEOUT = 2000; // one second (in msec)

    sockaddr_in sender_address;
    socklen_t sender_address_len = sizeof(sender_address);

    while (true)
    {
        // Unfortunately, the Linux kernel recvfrom() implementation
        // uses a non-interruptible sleep() when waiting for data,
        // which would cause this method to hang if the device is not
        // providing data.  We poll() the device first to make sure
        // the recvfrom() will not block.
        //
        // Note, however, that there is a known Linux kernel bug:
        //
        //   Under Linux, select() may report a socket file descriptor
        //   as "ready for reading", while nevertheless a subsequent
        //   read blocks.  This could for example happen when data has
        //   arrived but upon examination has wrong checksum and is
        //   discarded.  There may be other circumstances in which a
        //   file descriptor is spuriously reported as ready.  Thus it
        //   may be safer to use O_NONBLOCK on sockets that should not
        //   block.

        // poll() until input available
        do {
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

//        ROS_DEBUG_STREAM("incomplete lslidar packet read: "
//                         << nbytes << " bytes");

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

    this->getFPGA_GPSTimeStamp(packet);

    // Average the times at which we begin and end reading.  Use that to
    // estimate when the scan occurred.
    //double time2 = ros::Time::now().toSec();
    //packet->stamp = ros::Time((time2 + time1) / 2.0);
    packet->stamp = this->timeStamp;

    return 0;
}

bool LslidarC16Driver::polling()
{
    // Allocate a new shared pointer for zero-copy sharing with other nodelets.
    lslidar_c16_msgs::LslidarC16PacketPtr packet(
                new lslidar_c16_msgs::LslidarC16Packet());

    // Since the lslidar delivers data at a very high rate, keep
    // reading and publishing scans as fast as possible.
    //for (int i = 0; i < config_.npackets; ++i)
    //  {
    //    while (true)
    //      {
    //        // keep reading until full packet received
    //        int rc = input_->getPacket(&scan->packets[i]);
    //        if (rc == 0) break;       // got a full packet?
    //        if (rc < 0) return false; // end of file reached?
    //      }
    //  }
    while (true)
    {
        // keep reading until full packet received
        int rc = getPacket(packet);
        if (rc == 0) break;       // got a full packet?
        if (rc < 0) return false; // end of file reached?
    }

    // publish message using time of last packet read
    ROS_DEBUG("Publishing a full lslidar scan.");
    packet_pub.publish(*packet);

    // notify diagnostics that a message has been published, updating
    // its status
    diag_topic->tick(packet->stamp);
    diagnostics.update();

    return true;
}

void LslidarC16Driver::initTimeStamp(void)
{
    int i;

    for(i = 0;i < 10;i ++)
    {
        this->packetTimeStamp[i] = 0;
    }
    this->pointcloudTimeStamp = 0;

    this->timeStamp = ros::Time(0.0);
}

void LslidarC16Driver::getFPGA_GPSTimeStamp(lslidar_c16_msgs::LslidarC16PacketPtr &packet)
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
        unsigned int total_us = (packet->data[1200]) | (packet->data[1201] << 8) | (packet->data[1202] << 16) | (packet->data[1203] << 24);
        us = total_us % 1000;
        ms = total_us / 1000;
	if(ms >= 1000)
		// ROS_INFO("ms:%ju",ms);

        if(ms > 1000)
        {
            ms %= 1000;
        }

        total_us = ms * 1000 + us;
        
        cur_time.tm_sec = this->packetTimeStamp[4];
        cur_time.tm_min = this->packetTimeStamp[5];
        cur_time.tm_hour = this->packetTimeStamp[6];
        cur_time.tm_mday = this->packetTimeStamp[7];
        cur_time.tm_mon = this->packetTimeStamp[8]-1;
        cur_time.tm_year = this->packetTimeStamp[9]+2000-1900;
        this->pointcloudTimeStamp = static_cast<uint64_t>(timegm(&cur_time));

	//ROS_INFO("y:%ju m:%ju d:%ju h:%ju m:%ju s:%ju",cur_time.tm_year,cur_time.tm_mon,cur_time.tm_mday,cur_time.tm_hour,cur_time.tm_min,cur_time.tm_sec);

        timeStamp = ros::Time(this->pointcloudTimeStamp+total_us/10e5);
        //ROS_INFO("%lf",timeStamp.toSec());
    }
}

} // namespace lslidar_driver
}
}