/******************************************************************************
* Copyright 2018 The Apollo Authors. All Rights Reserved.
*
* Licensed under the Apache License, Version 2.0 (the "License");
* you may not use this file except in compliance with the License.
* You may obtain a copy of the License at
*
* http://www.apache.org/licenses/LICENSE-2.0
*
* Unless required by applicable law or agreed to in writing, software
* distributed under the License is distributed on an "AS IS" BASIS,
* WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
* See the License for the specific language governing permissions and
* limitations under the License.
*****************************************************************************/

/* -*- mode: C++ -*-
 *
 *  Copyright (C) 2007 Austin Robot Technology, Yaxin Liu, Patrick Beeson
 *  Copyright (C) 2009, 2010 Austin Robot Technology, Jack O'Quin
 *  Copyright (C) 2015, Jack O'Quin
 *
 *  License: Modified BSD Software License Agreement
 *
 *  $Id$
 */

#include <arpa/inet.h>
#include <errno.h>
#include <fcntl.h>
#include <poll.h>
#include <sys/file.h>
#include <sys/socket.h>
#include <unistd.h>

#include "rslidar_driver/socket_input.h"

namespace apollo {
namespace drivers {
namespace rslidar {

SocketInput::SocketInput() : sockfd_(-1), port_(0) {}

/** @brief destructor */
SocketInput::~SocketInput(void) {
  (void)close(sockfd_);
}

void SocketInput::init(int &port) {
  if (sockfd_ != -1) {
    (void)close(sockfd_);
  }

  // connect to RSLIDAR UDP port
  ROS_INFO_STREAM("Opening UDP socket: port " << uint16_t(port));
  port_ = port;
  sockfd_ = socket(AF_INET, SOCK_DGRAM, 0);

  if (sockfd_ == -1) {
    ROS_ERROR_STREAM("Init socket failed, UDP port is " << port);
    ROS_BREAK();
  }

  sockaddr_in my_addr;                       // my address information
  memset(&my_addr, 0, sizeof(my_addr));      // initialize to zeros
  my_addr.sin_family = AF_INET;              // host byte order
  my_addr.sin_port = htons(uint16_t(port));  // short, in network byte order
  my_addr.sin_addr.s_addr = INADDR_ANY;      // automatically fill in my IP
  //    my_addr.sin_addr.s_addr = inet_addr("192.168.1.100");

  if (bind(sockfd_, (sockaddr *)&my_addr, sizeof(sockaddr)) == -1) {
    ROS_ERROR_STREAM("Socket bind failed! Port " << port_);
    ROS_BREAK();
  }

  if (fcntl(sockfd_, F_SETFL, O_NONBLOCK | FASYNC) < 0) {
    ROS_ERROR_STREAM("non-block! Port " << port_);
    ROS_BREAK();
  }

  ROS_DEBUG_STREAM("rslidar socket fd is " << sockfd_ << ", port " << port_);
}

/** @brief Get one rslidar packet. */
int SocketInput::get_msop_data_packet(rslidar_msgs::rslidarPacket *pkt) {
  double time1 = ros::Time::now().toSec();
  while (true) {
    if (!input_available(POLL_TIMEOUT)) {
      return SOCKET_TIMEOUT;
    }
    // Receive packets that should now be available from the
    // socket using a blocking read.
    ssize_t nbytes = recvfrom(sockfd_, &(pkt->data[0]), FIRING_DATA_PACKET_SIZE,
                              0, NULL, NULL);

    if (nbytes < 0) {
      if (errno != EWOULDBLOCK) {
        ROS_ERROR_STREAM("recvfail from port " << port_);
        return RECIEVE_FAIL;
      }
    }

    if ((size_t)nbytes == FIRING_DATA_PACKET_SIZE) {
      // read successful, done now
      break;
    }

    ROS_ERROR_STREAM("Incomplete RSLIDAR rising data packet read: "
                     << nbytes << " bytes from port " << port_);
  }
  double time2 = ros::Time::now().toSec();
  pkt->stamp = ros::Time((time2 + time1) / 2.0);

  return 0;
}



bool SocketInput::input_available(int timeout) {
  struct pollfd fds[1];
  fds[0].fd = sockfd_;
  fds[0].events = POLLIN;
  
  do {
    int retval = poll(fds, 1, POLL_TIMEOUT);

    if (retval < 0) {  // poll() error?
      if (errno != EINTR) {
        ROS_ERROR_STREAM("RSLIDAR port "
                         << port_ << "poll() error: " << strerror(errno));
      }
      return false;
    }

    if (retval == 0) {  // poll() timeout?
      ROS_WARN_STREAM("RSLIDAR port " << port_ << " poll() timeout");
      return false;
    }

    if ((fds[0].revents & POLLERR) || (fds[0].revents & POLLHUP) ||
        (fds[0].revents & POLLNVAL)) {  // device error?
      ROS_ERROR_STREAM("RSLIDAR port " << port_
                                        << "poll() reports RSLIDAR error");
      return false;
    }
  } while ((fds[0].revents & POLLIN) == 0);
  return true;
}

}  // namespace rslidar
}  // namespace drivers
}  // namespace apollo
