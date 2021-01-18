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

#include "modules/drivers/lidar/lidar_robosense/include/lib/socket_input.h"

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
SocketInput::SocketInput() : Input(), _sockfd(-1), _port(0) {}

/** @brief destructor */
SocketInput::~SocketInput(void) { (void)close(_sockfd); }

void SocketInput::init(uint32_t port) {
  if (_sockfd != -1) {
    (void)close(_sockfd);
  }

  // connect to suteng UDP port
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
  AINFO << "SocketInput::init " << my_addr.sin_addr.s_addr;

  const int opt = -1;
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

  AINFO << "suteng socket fd is " << _sockfd << ", port " << _port;
}

/** @brief Get one suteng packet. */
int SocketInput::get_firing_data_packet(
    apollo::drivers::suteng::SutengPacket *pkt, int time_zone,
    uint64_t _start_time) {
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
      // uint64_t _temp_stamp = apollo::cyber::Time().Now().ToNanosecond();
      // uint64_t _pkt_start_diff = _temp_stamp -  _start_time;
      // AINFO<<"pkt _start_time:["<<_start_time<<"]";
      // AINFO<<"_pkt_start_diff:"<<apollo::cyber::Time(_pkt_start_diff -
      // last_pkt_stamp).ToSecond()*1000<<" ms"; last_pkt_stamp =
      // _pkt_start_diff;

      // pkt->set_stamp(_pkt_start_diff);
      //// NMEATimePtr nmea_time(new NMEATime);
      //// exract_nmea_time_from_packet(nmea_time, bytes+20);
      //
      //// tm pkt_time;
      //// memset(&pkt_time, 0, sizeof(pkt_time));
      //// pkt_time.tm_year = nmea_time->year + 100;
      //// pkt_time.tm_mon = nmea_time->mon - 1;
      //// pkt_time.tm_mday = nmea_time->day;
      //// pkt_time.tm_hour = nmea_time->hour + time_zone;
      //// pkt_time.tm_min = nmea_time->min;
      //// pkt_time.tm_sec = 0;
      tm pkt_time;
      memset(&pkt_time, 0, sizeof(pkt_time));
      pkt_time.tm_year = static_cast<int>(bytes[20] + 100);
      pkt_time.tm_mon = static_cast<int>(bytes[21] - 1);
      pkt_time.tm_mday = static_cast<int>(bytes[22]);
      pkt_time.tm_hour = static_cast<int>(bytes[23] + time_zone);
      pkt_time.tm_min = static_cast<int>(bytes[24]);
      pkt_time.tm_sec = static_cast<int>(bytes[25]);

      uint64_t timestamp_sec = static_cast<uint64_t>((mktime(&pkt_time)) * 1e9);
      // uint64_t timestamp_nsec = static_cast<uint64_t>(nmea_time->sec*1e6 +
      // nmea_time->msec*1e3 + nmea_time->usec)*1e3
      //                           + timestamp_sec; //ns

      // uint64_t timestamp_nsec = mktime(&stm) + 0.001 * (256 * pkt.data[26] +
      // pkt.data[27]) +
      //                       0.000001 * (256 * pkt.data[28] + pkt.data[29]);
      uint64_t timestamp_nsec =
          static_cast<uint64_t>(1000 * (256 * bytes[26] + bytes[27]) +
                                (256 * bytes[28] + bytes[29])) *
              1e3 +
          timestamp_sec;  // ns
      pkt->set_stamp(timestamp_nsec);

      if (!flags) {
        AINFO << "robo first PPS-GPS-timestamp: [" << timestamp_nsec << "]";
        // AINFO << "first PPS-GPS-time: [" <<
        // nmea_time->year<<"/"<<nmea_time->mon<<"/"<<nmea_time->day<<"-"
        //      <<nmea_time->hour<<"/"<< nmea_time->min<<"/"<<
        //      nmea_time->sec<<"-" << nmea_time->msec
        //      <<"/"<< nmea_time->usec<< "]";
        flags = true;
      }
      // AINFO << "PPS-GPS-time: [" <<
      // nmea_time->year<<"/"<<nmea_time->mon<<"/"<<nmea_time->day<<"-"
      //        <<nmea_time->hour<<"/"<< nmea_time->min<<"/"<<
      //        nmea_time->sec<<"-" << nmea_time->msec
      //        <<"/"<< nmea_time->usec<< "]";
      // AINFO<<"timestamp_nsec:["<<timestamp_nsec<<"]";
      break;
    }

    AERROR << " Incomplete suteng rising data packet read: " << nbytes
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
      if (exract_nmea_time_from_packet(nmea_time, bytes + 303)) {
        break;
      } else {
        return 1;
      }
    }

    AINFO << "incomplete suteng packet read: " << nbytes << " bytes from port "
          << _port;
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
        AERROR << " suteng port " << _port
               << "poll() error: " << strerror(errno);
      }
      return false;
    }

    if (retval == 0) {  // poll() timeout?
      AERROR << " suteng port " << _port << " poll() timeout";
      return false;
    }

    if ((fds[0].revents & POLLERR) || (fds[0].revents & POLLHUP) ||
        (fds[0].revents & POLLNVAL)) {  // device error?
      AERROR << " suteng port " << _port << "poll() reports suteng error";
      return false;
    }
  } while ((fds[0].revents & POLLIN) == 0);
  return true;
}

}  // namespace robosense
}  // namespace drivers
}  // namespace apollo
