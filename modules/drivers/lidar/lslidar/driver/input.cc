/******************************************************************************
 * Copyright 2020 The Apollo Authors. All Rights Reserved.
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

#include "modules/drivers/lidar/lslidar/driver/input.h"

#include <arpa/inet.h>
#include <fcntl.h>
#include <poll.h>
#include <sys/file.h>
#include <sys/socket.h>

#include <cerrno>
#include <cstdint>
#include <cstring>
#include <memory>

namespace apollo {
namespace drivers {
namespace lslidar {
namespace driver {

Input::Input(uint16_t port, std::string lidar_ip, int packet_size) {
    packet_size_ = packet_size;
    inet_aton(lidar_ip.c_str(), &devip_);
}

int Input::GetPacket(LslidarPacket *pkt) {
    return 0;
}

Input::~Input(void) {
    (void)close(sockfd_);
}

InputSocket::InputSocket(uint16_t port, std::string lidar_ip, int packet_size) :
        Input(port, lidar_ip, packet_size) {
    port_ = port;
    sockfd_ = -1;
    sockfd_ = socket(PF_INET, SOCK_DGRAM, 0);
    if (-1 == sockfd_) {
        AERROR << "socket open error";
        return;
    }

    inet_aton(lidar_ip.c_str(), &devip_);
    sockaddr_in myAddress;                     // my address information
    memset(&myAddress, 0, sizeof(myAddress));  // initialize to zeros
    myAddress.sin_family = AF_INET;            // host byte order
    myAddress.sin_port = htons(port);          // port in network byte order
    myAddress.sin_addr.s_addr = INADDR_ANY;    // automatically fill in my IP

    if (bind(sockfd_,
             reinterpret_cast<sockaddr *>(&myAddress),
             sizeof(sockaddr))
        == -1) {
        AERROR << "bind error, port:" << port;
        return;
    }

    if (fcntl(sockfd_, F_SETFL, O_NONBLOCK | FASYNC) < 0) {
        AERROR << "fcntl error";
        return;
    }
}

InputSocket::~InputSocket(void) {
    (void)close(sockfd_);
}

int InputSocket::GetPacket(LslidarPacket *pkt) {
    double time1 = apollo::cyber::Time().Now().ToSecond();
    struct pollfd fds[1];
    fds[0].fd = sockfd_;
    fds[0].events = POLLIN;
    static const int POLL_TIMEOUT = 3000;  // one second (in msec)

    sockaddr_in sender_address;
    socklen_t sender_address_len = sizeof(sender_address);

    while (true) {
        // Receive packets that should now be available from the
        // socket using a blocking read.
        // poll() until input available
        do {
            int retval = poll(fds, 1, POLL_TIMEOUT);
            if (retval < 0) {
                if (errno != EINTR)
                    AERROR << "poll() error: " << strerror(errno);

                return 1;
            }
            if (retval == 0) {
                AERROR << "lslidar poll() timeout, port: " << port_;
                return 1;
            }
            if ((fds[0].revents & POLLERR) || (fds[0].revents & POLLHUP)
                || (fds[0].revents & POLLNVAL)) {
                AWARN << "poll() reports lslidar error";
                return 1;
            }
        } while ((fds[0].revents & POLLIN) == 0);
        uint8_t bytes[1212];
        ssize_t nbytes = recvfrom(
                sockfd_,
                bytes,
                packet_size_,
                0,
                reinterpret_cast<sockaddr *>(&sender_address),
                &sender_address_len);
        if (nbytes < 0) {
            if (errno != EWOULDBLOCK) {
                AERROR << "recvfail";
                return 1;
            }
        } else if ((size_t)nbytes == size_t(packet_size_)) {
            if (sender_address.sin_addr.s_addr != devip_.s_addr) {
                AERROR << "lidar IP parameter set error,please reset in config "
                          "file";
                continue;
            } else {
                pkt->set_data(bytes, packet_size_);
                break;
            }
        }
        AERROR << "incomplete lslidar packet read: " << nbytes << " bytes";
    }

    // Average the times at which we begin and end reading.  Use that to
    // estimate when the scan occurred.
    double time2 = apollo::cyber::Time().Now().ToSecond();
    AINFO << apollo::cyber::Time((time2 + time1) / 2.0).ToNanosecond();
    return 0;
}

InputPCAP::InputPCAP(
        uint16_t port,
        std::string lidar_ip,
        int packet_size,
        double packet_rate,
        std::string filename,
        bool read_once,
        bool read_fast,
        double repeat_delay) :
        Input(port, lidar_ip, packet_size),
        packet_rate_(packet_rate),
        filename_(filename) {
    pcap_ = NULL;
    empty_ = true;
    packet_rate_ = packet_rate;
    lidar_ip_ = lidar_ip;
    // get parameters using private node handle
    read_once_ = read_once;
    read_fast_ = read_fast;
    repeat_delay_ = repeat_delay;

    if (read_once_)
        AINFO << "Read input file only once.";
    if (read_fast_)
        AINFO << "Read input file as quickly as possible.";
    if (repeat_delay_ > 0.0)
        AINFO << "Delay %.3f seconds before repeating input file."
              << repeat_delay_;

    // Open the PCAP dump file
    AERROR << "Opening PCAP file " << filename_;
    if ((pcap_ = pcap_open_offline(filename_.c_str(), errbuf_)) == NULL) {
        AERROR << "Error opening lslidar socket dump file.";
        return;
    }

    std::stringstream filter;
    if (lidar_ip != "") {
        filter << "src host " << lidar_ip << " && ";
    }
    filter << "udp dst port " << port;
    pcap_compile(
            pcap_,
            &pcap_packet_filter_,
            filter.str().c_str(),
            1,
            PCAP_NETMASK_UNKNOWN);
}

/** destructor */
InputPCAP::~InputPCAP(void) {
    pcap_close(pcap_);
}

/** @brief Get one lslidar packet. */
int InputPCAP::GetPacket(LslidarPacket *pkt) {
    struct pcap_pkthdr *header;
    const u_char *pkt_data;

    // while (flag == 1)
    while (!apollo::cyber::IsShutdown()) {
        int res;
        if ((res = pcap_next_ex(pcap_, &header, &pkt_data)) >= 0) {
            // Skip packets not for the correct port and from the
            // selected IP address.
            if (!lidar_ip_.empty()
                && (0
                    == pcap_offline_filter(
                            &pcap_packet_filter_, header, pkt_data)))
                continue;

            // Keep the reader from blowing through the file.
            if (read_fast_ == false)
                usleep(static_cast<int>(1000 * 1000 / packet_rate_ / 1.1));
            if (1206 == packet_size_) {
                uint8_t bytes[1206];
                memcpy(bytes, pkt_data + 42, packet_size_);
                pkt->set_data(bytes, packet_size_);
            } else if (1212 == packet_size_) {
                uint8_t bytes[1212];
                memcpy(bytes, pkt_data + 42, packet_size_);
                pkt->set_data(bytes, packet_size_);
            }

            empty_ = false;
            return 0;  // success
        }

        if (empty_) {
            AINFO << "Error " << res
                  << " reading lslidar packet: " << pcap_geterr(pcap_);
            return -1;
        }

        if (read_once_) {
            AINFO << "end of file reached -- done reading.";
            return -1;
        }

        if (repeat_delay_ > 0.0) {
            AINFO << "end of file reached -- delaying" << repeat_delay_
                  << "seconds.";
            usleep(rint(repeat_delay_ * 1000000.0));
        }

        AINFO << "replaying lslidar dump file";

        // I can't figure out how to rewind the file, because it
        // starts with some kind of header.  So, close the file
        // and reopen it with pcap.
        pcap_close(pcap_);
        pcap_ = pcap_open_offline(filename_.c_str(), errbuf_);
        empty_ = true;  // maybe the file disappeared?
    }                   // loop back and try again
    return 0;
}

}  // namespace driver
}  // namespace lslidar
}  // namespace drivers
}  // namespace apollo
