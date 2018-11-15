/******************************************************************************
* Modification Copyright 2018 The Apollo Authors. All Rights Reserved.
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

#include "modules/drivers/lidar_velodyne/common/socket_input.h"

#include <arpa/inet.h>
#include <errno.h>
#include <fcntl.h>
#include <poll.h>
#include <sys/file.h>
#include <sys/socket.h>
#include <unistd.h>

#include "modules/common/log.h"

namespace apollo {
namespace drivers {
namespace lidar_velodyne {

// InputSocket class implementation

/** @brief constructor
 *
 *  @param private_nh private node handle for driver
 *  @param udp_port UDP port number to connect
 */
SocketInput::SocketInput() : sockfd_(-1), port_(0) {}

/** @brief destructor */
SocketInput::~SocketInput(void) { (void)close(sockfd_); }

bool SocketInput::init(int port) {
  if (sockfd_ != -1) {
    (void)close(sockfd_);
  }

  // connect to Velodyne UDP port
  AINFO << "Opening UDP socket: port " << uint16_t(port);
  port_ = port;
  sockfd_ = socket(AF_INET, SOCK_DGRAM, 0);

  if (sockfd_ == -1) {
    AERROR << "Init socket failed, UDP port is " << port;
    return false;
  }

  sockaddr_in my_addr;                       // my address information
  memset(&my_addr, 0, sizeof(my_addr));      // initialize to zeros
  my_addr.sin_family = AF_INET;              // host byte order
  my_addr.sin_port = htons(uint16_t(port));  // short, in network byte order
  my_addr.sin_addr.s_addr = INADDR_ANY;      // automatically fill in my IP
  //    my_addr.sin_addr.s_addr = inet_addr("192.168.1.100");

  if (bind(sockfd_, reinterpret_cast<sockaddr *>(&my_addr), sizeof(sockaddr)) ==
      -1) {
    AERROR << "Socket bind failed! Port " << port_;
    return false;
  }

  if (fcntl(sockfd_, F_SETFL, O_NONBLOCK | FASYNC) < 0) {
    AERROR << "non-block! Port " << port_;
    return false;
  }

  ADEBUG << "Velodyne socket fd is " << sockfd_ << ", port " << port_;
  return true;
}

/** @brief Get one velodyne packet. */
int SocketInput::get_firing_data_packet(velodyne_msgs::VelodynePacket *pkt) {
  double time1 = ros::Time::now().toSec();
  while (true) {
    if (!input_available(POLL_TIMEOUT)) {
      AWARN_EVERY(100) << "no input data, wait time out.";
      return SOCKET_TIMEOUT;
    }
    // Receive packets that should now be available from the
    // socket using a blocking read.
    ssize_t nbytes = recvfrom(sockfd_, &(pkt->data[0]), FIRING_DATA_PACKET_SIZE,
                              0, NULL, NULL);

    if (nbytes < 0) {
      if (errno != EWOULDBLOCK) {
        AERROR << "recv fail from port " << port_;
        return RECIEVE_FAIL;
      }
    }

    if ((size_t)nbytes == FIRING_DATA_PACKET_SIZE) {
      // read successful, done now
      break;
    }

    AERROR << "Incomplete Velodyne rising data packet read: " << nbytes
           << " bytes from port " << port_;
  }
  double time2 = ros::Time::now().toSec();
  pkt->stamp = ros::Time((time2 + time1) / 2.0);

  return 0;
}

int SocketInput::get_positioning_data_packtet(const NMEATimePtr &nmea_time) {
  while (true) {
    if (!input_available(POLL_TIMEOUT)) {
      return 1;
    }
    // Receive packets that should now be available from the
    // socket using a blocking read.
    // Last 234 bytes not use
    uint8_t bytes[POSITIONING_DATA_PACKET_SIZE];
    ssize_t nbytes =
        recvfrom(sockfd_, bytes, POSITIONING_DATA_PACKET_SIZE, 0, NULL, NULL);

    if (nbytes < 0) {
      if (errno != EWOULDBLOCK) {
        AERROR << "recvfail from port " << port_;
        return 1;
      }
    }

    if ((size_t)nbytes == POSITIONING_DATA_PACKET_SIZE) {
      // read successful, exract nmea time
      if (exract_nmea_time_from_packet(nmea_time, bytes)) {
        break;
      } else {
        return 1;
      }
    }

    AINFO << "incomplete Velodyne packet read: " << nbytes
          << " bytes from port " << port_;
  }

  return 0;
}

bool SocketInput::input_available(int timeout) {
  struct pollfd fds[1];
  fds[0].fd = sockfd_;
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
        ADEBUG << "Velodyne port " << port_
               << "poll() error: " << strerror(errno);
      }
      return false;
    }

    if (retval == 0) {  // poll() timeout?
      AWARN_EVERY(100) << "Velodyne port " << port_ << " poll() timeout";
      return false;
    }

    if ((fds[0].revents & POLLERR) || (fds[0].revents & POLLHUP) ||
        (fds[0].revents & POLLNVAL)) {  // device error?
      AERROR << "Velodyne port " << port_ << "poll() reports Velodyne error";
      return false;
    }
  } while ((fds[0].revents & POLLIN) == 0);
  return true;
}

}  // namespace lidar_velodyne
}  // namespace drivers
}  // namespace apollo
