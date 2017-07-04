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

#include <inttypes.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <netinet/tcp.h>

#include <boost/thread/thread.hpp>

#include <ros/ros.h>

#include <gtest/gtest.h>

#include "gnss/parser.h"
#include "gnss/stream.h"

DEFINE_string(serial_port, "/dev/pts/28", "serial port");

namespace apollo {
namespace drivers {
namespace gnss_ut {

static const uint16_t TCP_PORT = 60000;
static const uint16_t UDP_PORT = 61000;
static const int COMM_TIMES = 100;
static volatile int s_server_ready = 0;

static bool stream_comm_test(::apollo::drivers::gnss::Stream *stream) {
    // connect
    int cnt = 0;
    bool connected = false;
    uint8_t buffer[1024];
    size_t data_len = 128;
    ssize_t len = -1;

    connected = stream->connect();
    if (!connected) {
        std::cout << "stream connect failed." << std::endl;
        return false;
    }

    bzero(buffer, sizeof(buffer));
    for (int i = 0; i < data_len; ++i) {
        buffer[i] = i;
    }

    while (cnt < COMM_TIMES) {
        len = stream->write(buffer, data_len);
        if (len > 0) {
            len = stream->read(buffer, 1024);
            if (len < 0) {
                std::cout << "tcp cliend read failed." << std::endl;
            }
            ++cnt;
        } else {
            std::cout << "tcp client write failed." << std::endl;
        }
    }

    connected = stream->disconnect();
    if (!connected) {
        std::cout << "stream disconnect failed." << std::endl;
        return false;
    }

    // check data
#if 0
    for (int i = 0; i < data_len; ++i) {
        if (buffer[i] != (i)) {
            std::cout << "buffer " << i << ": " << buffer[i] << "." << std::endl;
            return false;
        }
    }
#endif
    return true;
}

static void udp_server() {
    int fd = -1;
    int reuseaddr = 1;
    fd = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if (fd < 0) {
        // error
        ROS_ERROR_STREAM("create socket failed, errno: " << errno << ".");
        return ;
    }

    if (setsockopt(fd, SOL_SOCKET, SO_REUSEADDR, &reuseaddr, sizeof(reuseaddr)) < 0) {
        ROS_ERROR_STREAM("Set listen socket reuse addr failed: " <<  strerror(errno) << std::endl);
        return ;
    }

    struct sockaddr_in sockaddr;
    socklen_t socklenth = sizeof(sockaddr);
    bzero(&sockaddr, sizeof(sockaddr));
    sockaddr.sin_family = AF_INET;
    sockaddr.sin_port = htons(UDP_PORT);
    sockaddr.sin_addr.s_addr = htonl(INADDR_ANY);
    std::cout << "UDP port: " << UDP_PORT << ", fd: " << fd << "." << std::endl;
    if (bind(fd, (struct sockaddr*)&sockaddr, sizeof(sockaddr)) < 0) {
        // error
        ROS_ERROR_STREAM("bind socket failed, errno: " << errno << ", " << strerror(errno));
        return ;
    }
    s_server_ready = 1;

    uint8_t buffer[1024];
    int len = 0;
    int cnt = 0;
    while (cnt < COMM_TIMES) {
        len = ::recvfrom(fd,
                         buffer,
                         1024,
                         0,
                         (struct sockaddr*)&sockaddr,
                         (socklen_t*)&socklenth);
        if (len > 0) {
            len = ::sendto(fd, buffer, len, 0,
                           (struct sockaddr*)&sockaddr,
                           (socklen_t) sizeof(sockaddr));
            if (len <= 0) {
                ROS_ERROR_STREAM("udp server send data failed, seq " << cnt << ".");
            }
            ++cnt;
        }
    }

    // phrase 2, reconnect
    cnt = 0;
    while (cnt < COMM_TIMES) {
        len = ::recvfrom(fd,
                         buffer,
                         1024,
                         0,
                         (struct sockaddr*)&sockaddr,
                         (socklen_t*)&socklenth);
        if (len > 0) {
            len = ::sendto(fd, buffer, len, 0,
                           (struct sockaddr*)&sockaddr,
                           (socklen_t) sizeof(sockaddr));
            if (len <= 0) {
                std::cout << "udp server send data failed, seq " << cnt << "." << std::endl;
            }
            ++cnt;
        }
    }

    return ;
}

static void tcp_server() {
    struct sockaddr_in addr;
    struct sockaddr_in addr_client;
    socklen_t  addrlen;
    int reuseaddr = 1;
    int fd = ::socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
    if (fd < 0) {
        std::cout << "create tcp socker failed." << std::endl;
        return ;
    }

    if (setsockopt(fd, SOL_SOCKET, SO_REUSEADDR, &reuseaddr, sizeof(reuseaddr)) < 0) {
        std::cout << "Set listen socket reuse addr failed: " <<  strerror(errno) << std::endl;
        return ;
    }

    bzero(&addr, sizeof(addr));
    addr.sin_family = AF_INET;
    addr.sin_addr.s_addr = INADDR_ANY;
    addr.sin_port = htons(TCP_PORT);
    if (::bind(fd, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
        std::cout << "bind tcp socker failed." << std::endl;
        ::close(fd);
        return;
    }

    if (::listen(fd, 1024) < 0) {
        std::cout << "listen tcp socker failed." << std::endl;
        return ;
    }
    s_server_ready = 1;

    int connectfd = -1;
    while (connectfd < 0) {
        connectfd = accept(fd, (struct sockaddr*)&addr_client, &addrlen);
    }

    uint8_t buffer[1024];
    ssize_t len = -1;
    int cnt = 0;
    while (cnt < COMM_TIMES) {
        len = ::recv(connectfd, buffer, 1024, 0);
        if (len > 0) {
            len = ::send(connectfd, buffer, len, 0);
            if (len <= 0) {
                std::cout << "tcp server send data failed, seq " << cnt << "." << std::endl;
            }
            ++cnt;
        }
    }
    ::close(connectfd);
    connectfd = -1;

    // phrase 2, reconnect
    while (connectfd < 0) {
        connectfd = accept(fd, (struct sockaddr*)&addr_client, &addrlen);
    }

    cnt = 0;
    while (cnt < COMM_TIMES) {
        len = ::recv(connectfd, buffer, 1024, 0);
        if (len > 0) {
            len = ::send(connectfd, buffer, len, 0);
            if (len <= 0) {
                std::cout << "tcp server send data failed, seq " << cnt << "." << std::endl;
            }
            ++cnt;
        }
    }

    // close
    ::close(connectfd);
    ::close(fd);
    return ;
}

TEST(Stream, TcpUdp) {
    bool result = false;
    boost::thread thread_tcpserver(&tcp_server);

    while (s_server_ready == 0) {
        ;
    }

    ::apollo::drivers::gnss::Stream *tcp_stream =
                       ::apollo::drivers::gnss::Stream::create_tcp("127.0.0.1", TCP_PORT, 1000);

    result = stream_comm_test(tcp_stream);
    EXPECT_EQ(result, true);

    // reconnect test
    result = stream_comm_test(tcp_stream);
    EXPECT_EQ(result, true);

    // timeout test
    // socket error test
    // connect shutdown test
    thread_tcpserver.join();
    delete tcp_stream;

    s_server_ready = 0;
    boost::thread thread_udpserver(&udp_server);

    while (s_server_ready == 0) {
        ;
    }

    ::apollo::drivers::gnss::Stream *udp_stream =
                       ::apollo::drivers::gnss::Stream::create_udp("127.0.0.1", UDP_PORT, 1000);
    result = stream_comm_test(udp_stream);
    EXPECT_EQ(result, true);

    // reconnect test
    result = stream_comm_test(udp_stream);
    EXPECT_EQ(result, true);
    thread_udpserver.join();
    delete udp_stream;

    s_server_ready = 0;
    ::apollo::drivers::gnss::Stream *searial_stream =
                       ::apollo::drivers::gnss::Stream::create_serial(
                           FLAGS_serial_port.c_str(), ::apollo::drivers::gnss::BAUDRATE_9600, 1000);
    result = stream_comm_test(searial_stream);
    EXPECT_EQ(result, true);

    // reconnect test
    result = stream_comm_test(searial_stream);
    EXPECT_EQ(result, true);
    delete searial_stream;

}

} // namespace gnss_ut
} // namespace drivers
} // namespace apollo

int main(int argc, char** argv) {
    testing::InitGoogleTest(&argc, argv);

    //FLAGS_log_dir = ".";
    FLAGS_log_dir="./log/";
    google::ParseCommandLineFlags(&argc, &argv, true);
    google::InitGoogleLogging(argv[0]);
    return RUN_ALL_TESTS();
}
