/******************************************************************************
 * Copyright 2019 The Apollo Authors. All Rights Reserved.

 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at

 * http://www.apache.org/licenses/LICENSE-2.0

 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *****************************************************************************/

#include <arpa/inet.h>
#include <fcntl.h>
#include <poll.h>
#include <sys/file.h>
#include <sys/socket.h>
#include <unistd.h>

#include <cerrno>
#include <memory>

#include "cyber/cyber.h"
#include "modules/drivers/video/input.h"
#include "modules/drivers/video/socket_input.h"

namespace apollo {
namespace drivers {
namespace video {

////////////////////////////////////////////////////////////////////////
// InputSocket class implementation
////////////////////////////////////////////////////////////////////////

/** @brief constructor
 *
 *  @param private_nh private node handle for driver
 *  @param udpport UDP port number to connect
 */
SocketInput::SocketInput() : sockfd_(-1), port_(0) {
  pkg_num_ = 0;
  bytes_num_ = 0;
  frame_id_ = 0;
}

/** @brief destructor */
SocketInput::~SocketInput() {
  if (buf_) {
    delete[] buf_;
  }
  if (pdu_) {
    delete[] pdu_;
  }
  (void)close(sockfd_);
}

void SocketInput::Init(uint32_t port) {
  if (sockfd_ != -1) {
    (void)close(sockfd_);
  }

  sockfd_ = socket(AF_INET, SOCK_DGRAM, 0);
  if (sockfd_ < 0) {
    AERROR << "Failed to create socket fd.";
    return;
  }

  // Connect to camera UDP port
  AINFO << "Opening UDP socket on port: " << uint16_t(port);
  port_ = port;
  sockaddr_in my_addr;
  memset(&my_addr, '\0', sizeof(my_addr));
  my_addr.sin_family = AF_INET;
  my_addr.sin_port = htons(uint16_t(port));
  my_addr.sin_addr.s_addr = INADDR_ANY;
  if (bind(sockfd_, reinterpret_cast<sockaddr *>(&my_addr),
           sizeof(sockaddr_in)) < 0) {
    AERROR << "Failed to bind socket on local address.";
  }

  if (fcntl(sockfd_, F_SETFL, O_NONBLOCK | FASYNC) < 0) {
    AERROR << "Failed to enable non-blocking I/O.";
  }

  const int rbuf = 4 * 1024 * 1024;
  if (setsockopt(sockfd_, SOL_SOCKET, SO_RCVBUF, &rbuf, sizeof(int)) < 0) {
    AERROR << "Failed to enable socket receive buffer.";
  }

  int enable = 1;
  if (setsockopt(sockfd_, SOL_SOCKET, SO_REUSEADDR, &enable, sizeof(int)) < 0) {
    AERROR << "Failed to enable socket reuseable address.";
  }

  buf_ = new uint8_t[H265_FRAME_PACKAGE_SIZE];
  if (!buf_) {
    AERROR << "Failed to allocate H265 frame package buffer.";
  }
  pdu_ = new uint8_t[H265_PDU_SIZE];
  if (!pdu_) {
    AERROR << "Failed to allocate H265 PDU buffer.";
  }
  // _metric_controller->Init();
  AINFO << "Camera socket fd: " << sockfd_ << ", port: " << port_;
}

/** @brief Get one camera packet. */
int SocketInput::GetFramePacket(std::shared_ptr<CompressedImage> h265Pb) {
  uint8_t *frame_data = &buf_[0];
  uint8_t *pdu_data = &pdu_[0];
  int total = 0;
  int pkg_len = 0;
  size_t frame_len = 0;
  uint16_t pre_seq = 0;

  do {
    if (!InputAvailable(POLL_TIMEOUT)) {
      return SOCKET_TIMEOUT;
    }
    // Receive packets that should now be available from the socket using
    // a blocking read.
    ssize_t pdu_len = recvfrom(sockfd_, pdu_data, H265_PDU_SIZE, 0, NULL, NULL);
    if (pdu_len < 0) {
      if (errno != EWOULDBLOCK) {
        AERROR << "Failed to receive package from port: " << port_;
        return RECIEVE_FAIL;
      }
    }

    AINFO << "Received pdu length: " << pdu_len << " from port: " << port_;
    HwPduPacket *pdu_pkg = reinterpret_cast<HwPduPacket *>(&pdu_[0]);
    uint16_t local_seq = ntohs(pdu_pkg->rtp_header.seq);
    AINFO << "Package seq number: " << local_seq;
    if (local_seq - pre_seq != 1 && pre_seq > 1 && local_seq > 0) {
      AERROR << "Error! port: " << port_
             << ", package sequence is wrong. curent/pre " << local_seq << "/"
             << pre_seq;
    }
    pre_seq = local_seq;

    if (ntohl(pdu_pkg->header.magic0) == HW_CAMERA_MAGIC0 &&
        ntohl(pdu_pkg->header.magic1) == HW_CAMERA_MAGIC1) {
      // Receive camera frame head
      if (total) {
        AERROR << "Error! lost package for last frame, left bytes: " << total;
      }
      AINFO << "Received new frame from port: " << port_;

      uint32_t frame_id = ntohl(pdu_pkg->header.frame_id);
      if (frame_id - frame_id_ != 1 && frame_id_ > 1 && frame_id > 1) {
        AERROR << "Error! port: " << port_
               << ", lose Frame. pre_frame_id/frame_id " << frame_id_ << "/"
               << frame_id;
      }
      frame_id_ = frame_id;

      cyber::Time image_time(ntohl(pdu_pkg->header.ts_sec),
                             1000 * ntohl(pdu_pkg->header.ts_usec));
      // AINFO << "image_time second: " << ntohl(pdu_pkg->header.ts_sec) <<
      // " usec: " << ntohl(pdu_pkg->header.ts_usec);
      uint64_t camera_timestamp = image_time.ToNanosecond();
      h265Pb->mutable_header()->set_camera_timestamp(camera_timestamp);
      h265Pb->set_measurement_time(image_time.ToSecond());
      h265Pb->set_format("h265");
      h265Pb->set_frame_type(static_cast<int>(pdu_pkg->header.frame_type));
      AINFO << "Port: " << port_
            << ", received frame size: " << ntohl(pdu_pkg->header.frame_size)
            << ", frame type: " << static_cast<int>(pdu_pkg->header.frame_type)
            << ", PhyNo: " << static_cast<int>(pdu_pkg->header.PhyNo)
            << ", frame id: " << frame_id;

      frame_len = ntohl(pdu_pkg->header.frame_size);
      total = static_cast<int>(frame_len);
      frame_data = &buf_[0];
      continue;
    }
    // Receive camera frame data
    if (total > 0) {
      pkg_len = static_cast<int>(pdu_len - sizeof(RtpHeader));
      memcpy(frame_data, pdu_data + sizeof(RtpHeader), pkg_len);
      total -= pkg_len;
      frame_data += pkg_len;
      // AINFO << "receive pkg data " << pkg_len << "/" << total << "/"
      // << frame_len;
    }
    if (total <= 0) {
      total = 0;
      // AINFO << "receive frame data " << pkg_len << "/" << total << "/"
      // << frame_len;
      if (frame_len > 0) {
        h265Pb->set_data(buf_, frame_len);
        break;
      }
      AERROR << "Error! frame info is wrong. frame length: " << frame_len;
    }
  } while (true);

  return 0;
}

bool SocketInput::InputAvailable(int timeout) {
  (void)timeout;
  struct pollfd fds[1];
  fds[0].fd = sockfd_;
  fds[0].events = POLLIN;
  // Unfortunately, the Linux kernel recvfrom() implementation
  // uses a non-interruptible sleep() when waiting for data,
  // which would cause this method to hang if the device is not
  // providing data.  We poll() the device first to make sure
  // the recvfrom() will not block.
  // Note that, however, there is a known Linux kernel bug:
  // Under Linux, select() may report a socket file descriptor
  // as "ready for reading", while nevertheless a subsequent
  // read blocks.  This could for example happen when data has
  // arrived but upon examination has wrong checksum and is
  // discarded.  There may be other circumstances in which a
  // file descriptor is spuriously reported as ready.  Thus it
  // may be safer to use O_NONBLOCK on sockets that should not
  // block.

  // poll() until input available
  do {
    int ret = poll(fds, 1, POLL_TIMEOUT);
    if (ret < 0) {
      if (errno != EINTR) {
        AERROR << "H265 camera port: " << port_
               << "poll() error: " << strerror(errno);
      }
      return false;
    }

    // Timeout
    if (ret == 0) {
      return false;
    }

    if ((fds[0].revents & POLLERR) || (fds[0].revents & POLLHUP) ||
        (fds[0].revents & POLLNVAL)) {
      AERROR << "Error! poll failed on H265 camera port: " << port_;
      return false;
    }
  } while (!(fds[0].revents & POLLIN));

  return true;
}

}  // namespace video
}  // namespace drivers
}  // namespace apollo
