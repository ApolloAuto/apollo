 /******************************************************************************
 * Copyright 2017 The Apollo Authors. All Rights Reserved.
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

#include "velodyne_driver/socket_input.h"

namespace apollo {
namespace drivers {
namespace velodyne {

////////////////////////////////////////////////////////////////////////
// InputSocket class implementation
////////////////////////////////////////////////////////////////////////

/** @brief constructor
 *
 *  @param private_nh private node handle for driver
 *  @param udp_port UDP port number to connect
 */
SocketInput::SocketInput() : _sockfd(-1), _port(0) {}

/** @brief destructor */
SocketInput::~SocketInput(void) { 
  (void)close(_sockfd); 
}

void SocketInput::init(int &port) {
  if (_sockfd != -1) {
    (void)close(_sockfd);
  }

  // connect to Velodyne UDP port
  ROS_INFO_STREAM("Opening UDP socket: port " << uint16_t(port));
  _port = port;
  _sockfd = socket(AF_INET, SOCK_DGRAM, 0);

  if (_sockfd == -1) {
    ROS_ERROR_STREAM("Init socket failed, UDP port is " << port);
    ROS_BREAK();
  }

  sockaddr_in my_addr;                       // my address information
  memset(&my_addr, 0, sizeof(my_addr));      // initialize to zeros
  my_addr.sin_family = AF_INET;              // host byte order
  my_addr.sin_port = htons(uint16_t(port));  // short, in network byte order
  my_addr.sin_addr.s_addr = INADDR_ANY;      // automatically fill in my IP
  //    my_addr.sin_addr.s_addr = inet_addr("192.168.1.100");

  if (bind(_sockfd, (sockaddr *)&my_addr, sizeof(sockaddr)) == -1) {
    ROS_ERROR_STREAM("Socket bind failed! Port " << _port);
    ROS_BREAK();
  }

  if (fcntl(_sockfd, F_SETFL, O_NONBLOCK | FASYNC) < 0) {
    ROS_ERROR_STREAM("non-block! Port " << _port);
    ROS_BREAK();
  }

  ROS_DEBUG_STREAM("Velodyne socket fd is " << _sockfd << ", port " << _port);
}

/** @brief Get one velodyne packet. */
int SocketInput::get_firing_data_packet(velodyne_msgs::VelodynePacket *pkt) {
  double time1 = ros::Time::now().toSec();
  while (true) {
    if (!input_available(POLL_TIMEOUT)) {
      return SOCKET_TIMEOUT;
    }
    // Receive packets that should now be available from the
    // socket using a blocking read.
    ssize_t nbytes = recvfrom(_sockfd, &(pkt->data[0]), FIRING_DATA_PACKET_SIZE,
                              0, NULL, NULL);

    if (nbytes < 0) {
      if (errno != EWOULDBLOCK) {
        ROS_ERROR_STREAM("recvfail from port " << _port);
        return RECIEVE_FAIL;
      }
    }

    if ((size_t)nbytes == FIRING_DATA_PACKET_SIZE) {
      // read successful, done now
      break;
    }

    ROS_ERROR_STREAM("Incomplete Velodyne rising data packet read: "
                     << nbytes << " bytes from port " << _port);
  }
  double time2 = ros::Time::now().toSec();
  pkt->stamp = ros::Time((time2 + time1) / 2.0);

  return 0;
}

bool SocketInput::input_available(int timeout) {
  struct pollfd fds[1];
  fds[0].fd = _sockfd;
  fds[0].events = POLLIN;
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

    if (retval < 0) {  // poll() error?
      if (errno != EINTR) {
        ROS_ERROR_STREAM("Velodyne port "
                         << _port << "poll() error: " << strerror(errno));
      }
      return false;
    }

    if (retval == 0) {  // poll() timeout?
      ROS_WARN_STREAM("Velodyne port " << _port << " poll() timeout");
      return false;
    }

    if ((fds[0].revents & POLLERR) || (fds[0].revents & POLLHUP) ||
        (fds[0].revents & POLLNVAL)) {  // device error?
      ROS_ERROR_STREAM("Velodyne port " << _port
                                        << "poll() reports Velodyne error");
      return false;
    }
  } while ((fds[0].revents & POLLIN) == 0);
  return true;
}

}  // namespace velodyne
}  // namespace drivers
}  // namespace apollo
