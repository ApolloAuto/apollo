/******************************************************************************
 * Copyright 2021 The Apollo Authors. All Rights Reserved.
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

#include "modules/drivers/lidar/robosense/lib/socket_input_16p.h"

#include <arpa/inet.h>
#include <errno.h>
#include <fcntl.h>
#include <poll.h>
#include <sys/file.h>
#include <sys/socket.h>
#include <time.h>
#include <unistd.h>

namespace apollo {
namespace drivers {
namespace robosense {

////////////////////////////////////////////////////////////////////////
// InputSocket class implementation
////////////////////////////////////////////////////////////////////////

/** @brief constructor
 *
 *  @param private_nh private node handle for driver
 *  @param udp_port UDP port number to connect
 */
SocketInput16P::SocketInput16P() : Input(), sockfd_(-1), port_(0) {}

/** @brief destructor */
SocketInput16P::~SocketInput16P(void) { (void)close(sockfd_); }

void SocketInput16P::init(uint32_t port) {
  if (sockfd_ != -1) {
    (void)close(sockfd_);
  }

  // connect to suteng UDP port
  AINFO << "Opening UDP socket: port " << uint16_t(port);
  port_ = port;
  sockfd_ = socket(AF_INET, SOCK_DGRAM, 0);

  if (sockfd_ == -1) {
    AERROR << " Init socket failed, UDP port is " << port;
  }

  sockaddr_in my_addr;                       // my address information
  memset(&my_addr, 0, sizeof(my_addr));      // initialize to zeros
  my_addr.sin_family = AF_INET;              // host byte order
  my_addr.sin_port = htons(uint16_t(port));  // short, in network byte order
  my_addr.sin_addr.s_addr = INADDR_ANY;      // automatically fill in my IP
  AINFO << "SocketInput16P::init " << my_addr.sin_addr.s_addr;

  const int opt = -1;
  int rtn = setsockopt(sockfd_, SOL_SOCKET, SO_REUSEPORT, &opt, sizeof(opt));
  if (rtn < 0) {
    AINFO << "setsockopt failed !!!!!!!!!!";
    return;
  }

  if (bind(sockfd_, reinterpret_cast<sockaddr*>(&my_addr), sizeof(sockaddr)) ==
      -1) {
    AERROR << " Socket bind failed! Port " << port_;
    return;
  }

  if (fcntl(sockfd_, F_SETFL, O_NONBLOCK | FASYNC) < 0) {
    AERROR << " non-block! Port " << port_;
    return;
  }

  AINFO << "suteng socket fd is " << sockfd_ << ", port " << port_;
}

/** @brief Get one suteng packet. */
int SocketInput16P::get_firing_data_packet(
    apollo::drivers::suteng::SutengPacket* pkt, bool use_gps_time) {
  while (true) {
    if (!input_available(POLL_TIMEOUT)) {
      AINFO << "SocketInput16P::get_firing_data_packet---SOCKET_TIMEOUT";
      return SOCKET_TIMEOUT;
    }
    // Receive packets that should now be available from the
    // socket using a blocking read.
    uint8_t bytes[FIRING_DATA_PACKET_SIZE];
    ssize_t nbytes =
        recvfrom(sockfd_, bytes, FIRING_DATA_PACKET_SIZE, 0, NULL, NULL);

    if (nbytes < 0) {
      if (errno != EWOULDBLOCK) {
        AERROR << " recvfail from port " << port_;
        return RECIEVE_FAIL;
      }
    }

    if ((size_t)nbytes == FIRING_DATA_PACKET_SIZE) {
      pkt->set_data(bytes, FIRING_DATA_PACKET_SIZE);

      if (!use_gps_time) {
        pkt->set_stamp(apollo::cyber::Time().Now().ToNanosecond());
        if (!flags) {
          AINFO << "robo msop first Cyber-timestamp: [" << pkt->stamp() << "]";
          flags = true;
        }
      } else {
        uint64_t timestamp_nsec = 0;
        exract_utc_time_from_packet(bytes + 20, &timestamp_nsec);

        pkt->set_stamp(timestamp_nsec);
        if (!flags) {
          AINFO << "robo msop first PPS-GPS-timestamp: [" << timestamp_nsec
                << "]";
          flags = true;
        }
      }
      break;
    }
    AERROR << " Incomplete suteng rising data packet read: " << nbytes
           << " bytes from port " << port_;
  }
  return 0;
}

bool SocketInput16P::exract_utc_time_from_packet(const uint8_t* bytes,
                                                 uint64_t* utc_time_ns) {
  unsigned int field_index = 0;

  time_t tmp_utc_time_sec = 0;
  uint64_t tmp_utc_time_usec = 0;

  for (field_index = 0; field_index < 6; field_index++) {
    tmp_utc_time_sec <<= 8;
    tmp_utc_time_sec += static_cast<time_t>(bytes[field_index]);
  }

  for (field_index = 6; field_index < 10; field_index++) {
    tmp_utc_time_usec <<= 8;
    tmp_utc_time_usec += static_cast<uint64_t>(bytes[field_index]);
  }

  *utc_time_ns = static_cast<uint64_t>(tmp_utc_time_usec) * 1e3 +
                 static_cast<uint64_t>((tmp_utc_time_sec)*1e9);  // ns

  // AINFO << "Get UTC time [" << *utc_time_ns
  //       << "]ns from pkt, sec: " << tmp_utc_time_sec
  //       << " usec:" << tmp_utc_time_usec;
  return true;
}

int SocketInput16P::get_positioning_data_packet(
    apollo::drivers::suteng::SutengPacket* pkt, bool use_gps_time) {
  while (true) {
    if (!input_available(POLL_TIMEOUT * 5)) {
      return 1;
    }
    // Receive packets that should now be available from the
    // socket using a blocking read.
    uint8_t bytes[POSITIONING_DATA_PACKET_SIZE];

    ssize_t nbytes =
        recvfrom(sockfd_, bytes, POSITIONING_DATA_PACKET_SIZE, 0, NULL, NULL);

    if (nbytes < 0) {
      if (errno != EWOULDBLOCK) {
        AERROR << " recvfail from port " << port_;
        return 1;
      }
    }

    if ((size_t)nbytes == POSITIONING_DATA_PACKET_SIZE) {
      // read successful, exract UTC time
      pkt->set_data(bytes, POSITIONING_DATA_PACKET_SIZE);

      if (!use_gps_time) {
        pkt->set_stamp(apollo::cyber::Time().Now().ToNanosecond());
        if (!flags) {
          AINFO << "robo difop first Cyber-timestamp: [" << pkt->stamp() << "]";
          flags = true;
        }
      } else {
        uint64_t timestamp_nsec = 0;
        exract_utc_time_from_packet(bytes + 303, &timestamp_nsec);
        if (!flags) {
          AINFO << "robo difop first PPS-GPS-timestamp: [" << timestamp_nsec
                << "] at Cyber-timestamp: ["
                << apollo::cyber::Time().Now().ToNanosecond() << "]";
          flags = true;
        }
        pkt->set_stamp(timestamp_nsec);
      }
      break;
    }

    AINFO << "incomplete suteng packet read: " << nbytes << " bytes from port "
          << port_;
  }

  return 0;
}

bool SocketInput16P::input_available(int timeout) {
  struct pollfd fds[1];
  fds[0].fd = sockfd_;
  fds[0].events = POLLIN;
  do {
    int retval = poll(fds, 1, timeout);

    if (retval < 0) {  // poll() error?
      if (errno != EINTR) {
        AERROR << " suteng port " << port_
               << "poll() error: " << strerror(errno);
      }
      return false;
    }

    if (retval == 0) {  // poll() timeout?
      AERROR << " suteng port " << port_ << " poll() timeout";
      return false;
    }

    if ((fds[0].revents & POLLERR) || (fds[0].revents & POLLHUP) ||
        (fds[0].revents & POLLNVAL)) {  // device error?
      AERROR << " suteng port " << port_ << "poll() reports suteng error";
      return false;
    }
  } while ((fds[0].revents & POLLIN) == 0);
  return true;
}

}  // namespace robosense
}  // namespace drivers
}  // namespace apollo
