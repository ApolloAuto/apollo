/******************************************************************************
 * copyright 2020 The Apollo Authors. All Rights Reserved.
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

#include "modules/drivers/lidar_surestar/lib/socket_input.h"

#include <arpa/inet.h>
#include <errno.h>
#include <fcntl.h>
#include <poll.h>
#include <sys/file.h>
#include <sys/socket.h>
#include <unistd.h>

namespace apollo {
namespace drivers {
namespace surestar {

////////////////////////////////////////////////////////////////////////
// InputSocket class implementation
////////////////////////////////////////////////////////////////////////

/** @brief constructor
 *
 *  @param private_nh private node handle for driver
 *  @param udp_port UDP port number to connect
 */
SocketInput::SocketInput() : Input(), _sockfd(-1), _port(0) {}

/** @brief destructor */
SocketInput::~SocketInput(void) { (void)close(_sockfd); }

void SocketInput::init(uint32_t port) {
  if (_sockfd != -1) {
    (void)close(_sockfd);
  }

  // connect to surestar UDP port
  AINFO << "Opening UDP socket: port " << uint16_t(port);
  _port = port;
  _sockfd = socket(AF_INET, SOCK_DGRAM, 0);

  if (_sockfd == -1) {
    AERROR << " Init socket failed, UDP port is " << port;
  }

  sockaddr_in my_addr;                       // my address information
  memset(&my_addr, 0, sizeof(my_addr));      // initialize to zeros
  my_addr.sin_family = AF_INET;              // host byte order
  my_addr.sin_port = htons(uint16_t(port));  // short, in network byte order
  my_addr.sin_addr.s_addr = INADDR_ANY;      // automatically fill in my IP
  // my_addr.sin_addr.s_addr = inet_addr("255.255.255.255");
  AINFO << "SocketInput::init-51 " << my_addr.sin_addr.s_addr;

  const int opt = -1;
  // int rtn = setsockopt(_sockfd, SOL_SOCKET, SO_REUSEPORT,
  //                      (char *)(&opt), sizeof(opt));
  int rtn = setsockopt(_sockfd, SOL_SOCKET, SO_REUSEPORT, &opt, sizeof(opt));
  if (rtn < 0) {
    AINFO << "setsockopt failed !!!!!!!!!!";
    return;
  }

  // if (bind(_sockfd, (sockaddr *)&my_addr, sizeof(sockaddr)) == -1) {
  if (bind(_sockfd, reinterpret_cast<sockaddr *>(&my_addr), sizeof(sockaddr)) ==
      -1) {
    AERROR << " Socket bind failed! Port " << _port;
    return;
  }

  if (fcntl(_sockfd, F_SETFL, O_NONBLOCK | FASYNC) < 0) {
    AERROR << " non-block! Port " << _port;
    return;
  }

  AINFO << "surestar socket fd is " << _sockfd << ", port " << _port;
}

/** @brief Get one surestar packet. */
int SocketInput::get_firing_data_packet(
    apollo::drivers::Surestar::SurestarPacket *pkt) {
  // double time1 = ros::Time::Now().toSec();
  while (true) {
    if (!input_available(POLL_TIMEOUT)) {
      AINFO << "SocketInput::get_firing_data_packet---SOCKET_TIMEOUT";
      return SOCKET_TIMEOUT;
    }
    // Receive packets that should now be available from the
    // socket using a blocking read.
    uint8_t bytes[FIRING_DATA_PACKET_SIZE];
    ssize_t nbytes =
        recvfrom(_sockfd, bytes, FIRING_DATA_PACKET_SIZE, 0, NULL, NULL);

    if (nbytes < 0) {
      if (errno != EWOULDBLOCK) {
        AERROR << " recvfail from port " << _port;
        return RECIEVE_FAIL;
      }
    }

    if ((size_t)nbytes == FIRING_DATA_PACKET_SIZE) {
      pkt->set_data(bytes, FIRING_DATA_PACKET_SIZE);
      break;
    }

    AERROR << " Incomplete surestar rising data packet read: " << nbytes
           << " bytes from port " << _port;
  }
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
        recvfrom(_sockfd, bytes, POSITIONING_DATA_PACKET_SIZE, 0, NULL, NULL);

    if (nbytes < 0) {
      if (errno != EWOULDBLOCK) {
        AERROR << " recvfail from port " << _port;
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

    AINFO << "incomplete surestar packet read: " << nbytes
          << " bytes from port " << _port;
  }

  return 0;
}

bool SocketInput::input_available(int timeout) {
  (void)timeout;
  struct pollfd fds[1];
  fds[0].fd = _sockfd;
  fds[0].events = POLLIN;

  do {
    int retval = poll(fds, 1, POLL_TIMEOUT);
    if (retval < 0) {  // poll() error?
      if (errno != EINTR) {
        AERROR << " surestar port " << _port
               << "poll() error: " << strerror(errno);
      }
      return false;
    }
    if (retval == 0) {
      AERROR << " surestar port " << _port << " poll() timeout";
      return false;
    }
    if ((fds[0].revents & POLLERR) || (fds[0].revents & POLLHUP) ||
        (fds[0].revents & POLLNVAL)) {
      AERROR << " surestar port " << _port << "poll() reports surestar error";
      return false;
    }
  } while ((fds[0].revents & POLLIN) == 0);
  return true;
}

}  // namespace surestar
}  // namespace drivers
}  // namespace apollo
