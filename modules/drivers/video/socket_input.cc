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

#include <unistd.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <poll.h>
#include <errno.h>
#include <fcntl.h>
#include <sys/file.h>

#include <memory>
#include "cyber/cyber.h"
#include "modules/drivers/video/socket_input.h"
#include "modules/drivers/video/input.h"

namespace apollo {
namespace drivers {
namespace video {

////////////////////////////////////////////////////////////////////////
// InputSocket class implementation
////////////////////////////////////////////////////////////////////////

/** @brief constructor
 
 *  @param private_nh private node handle for driver
 *  @param udp_port UDP port number to connect
 */
SocketInput::SocketInput() : _sockfd(-1), _port(0) {
  _pkgNum = 0;
  _bytesNum = 0;
  _frame_id = 0;
}

/** @brief destructor */
SocketInput::~SocketInput(void) {
  if (_buf) delete [] _buf;
  if (_pdu) delete [] _pdu;
  (void)close(_sockfd);
}

void SocketInput::Init(uint32_t port) {
  if (_sockfd != -1) {
    (void)close(_sockfd);
  }

  // connect to camera UDP port
  AINFO << "Opening UDP socket: port " << uint16_t(port);
  _port = port;
  _sockfd = socket(AF_INET, SOCK_DGRAM, 0);
  if (_sockfd < 0) {
    AERROR << "create socket error.";
    return;
  }

  if (_sockfd == -1) {
    AERROR << "Init socket failed, UDP port is " << port;
  }

  sockaddr_in my_addr;                       // my address information
  memset(&my_addr, 0, sizeof(my_addr));      // initialize to zeros
  my_addr.sin_family = AF_INET;              // host byte order
  my_addr.sin_port = htons(uint16_t(port));  // short, in network byte order
  my_addr.sin_addr.s_addr = INADDR_ANY;      // automatically fill in my IP

  if (bind(_sockfd, reinterpret_cast<sockaddr*>(&my_addr),
        sizeof(sockaddr_in)) == -1) {
    AERROR << "Socket bind failed! Port " << _port;
  }

  if (fcntl(_sockfd, F_SETFL, O_NONBLOCK | FASYNC) < 0) {
    AERROR << "non-block! Port " << _port;
  }

  const int nRecvBuf = 4*1024*1024;
  if (setsockopt(_sockfd, SOL_SOCKET, SO_RCVBUF, &nRecvBuf, sizeof(int))) {
    AERROR << "set socket recieve buff error";
  }


  int enable = 1;
  if (setsockopt(_sockfd, SOL_SOCKET, SO_REUSEADDR, &enable, sizeof(int)) < 0) {
    AERROR << "set socket reuse addr error";
  }

  _buf = new uint8_t[H265_FRAME_PACKAGE_SIZE];
  if (!_buf) {
    AERROR << "error alloc buf failed";
  }
  _pdu = new uint8_t[H265_PDU_SIZE];
  if (!_pdu) {
    AERROR << "error alloc buf failed";
  }
  // _metric_controller->Init();
  AINFO << "camera socket fd is " << _sockfd << ", port " << _port;
}

/** @brief Get one camera packet. */
int SocketInput::get_frame_packet(std::shared_ptr<CompressedImage> h265Pb) {
  uint8_t *frame_data = &_buf[0];
  uint8_t *pdu_data = &_pdu[0];
  int total = 0;
  int pkg_len = 0;
  size_t frame_len = 0;
  uint16_t pre_seq = 0;

  while (true) {
    if (!input_available(POLL_TIMEOUT)) {
      return SOCKET_TIMEOUT;
    }
    // Receive packets that should now be available from the
    // socket using a blocking read.
    ssize_t pdu_len =
      recvfrom(_sockfd, pdu_data, H265_PDU_SIZE, 0, NULL, NULL);

    if (pdu_len < 0) {
      if (errno != EWOULDBLOCK) {
        AERROR << "recvfail from port " << _port;
        return RECIEVE_FAIL;
      }
    }

    // AINFO << "recive " << pdu_len << " from port " << _port;
    hw_pdu_packet *pdu_pkg = reinterpret_cast<hw_pdu_packet *>(&_pdu[0]);
    uint16_t local_seq = ntohs(pdu_pkg->rtp_header.seq);
    // AINFO << "package seq number " << local_seq;
    if (local_seq - pre_seq != 1 && (pre_seq > 1) && (local_seq > 0)) {
      AERROR << "error: port " << _port <<
        " package sequence is wrong, curent/pre " << local_seq << "/"
        << pre_seq;
    }
    pre_seq = local_seq;

    if (ntohl(pdu_pkg->header.magic0) == HW_CAMERA_MAGIC0 &&
        ntohl(pdu_pkg->header.magic1) == HW_CAMERA_MAGIC1) {
      // receive camera frame head
      if (total) {
        AERROR << "error: lost package for last frame, left bytes = " << total;
      }
      // AINFO << "receive new frame from " << _port;

      uint32_t frame_id = ntohl(pdu_pkg->header.FrameId);
      if (frame_id - _frame_id != 1 && _frame_id > 1 && frame_id > 1) {
        AERROR << "error: port " << _port <<
          " Frame losing, pre_frame_id/frame_id " << _frame_id << "/"
          << frame_id;
      }
      _frame_id = frame_id;

      cyber::Time image_time(ntohl(pdu_pkg->header.ts_sec),
          1000 * ntohl(pdu_pkg->header.ts_usec));
      // AINFO << "image_time second: " << ntohl(pdu_pkg->header.ts_sec) <<
      // " usec: " << ntohl(pdu_pkg->header.ts_usec);
      uint64_t camera_timestamp = image_time.ToNanosecond();
      h265Pb->mutable_header()->set_camera_timestamp(camera_timestamp);
      h265Pb->set_measurement_time(image_time.ToSecond());
      h265Pb->set_format("h265");
      h265Pb->set_frame_type(static_cast<int>(pdu_pkg->header.FrameType));
      AINFO << "port " << _port << ", receive frameSize = " <<
        ntohl(pdu_pkg->header.FrameSize) << " frame type = " <<
        static_cast<int>(pdu_pkg->header.FrameType) << " PhyNo = " <<
        static_cast<int>(pdu_pkg->header.PhyNo) << " frame_id = " << frame_id;

      frame_len = ntohl(pdu_pkg->header.FrameSize);
      total = static_cast<int>(frame_len);
      frame_data = &_buf[0];
      continue;
    }  else {
      // receive camera frame data
      if (total > 0) {
        pkg_len = static_cast<int>(pdu_len - sizeof(RTP_Header));
        memcpy(frame_data, pdu_data + sizeof(RTP_Header), pkg_len);
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
          h265Pb->set_data(_buf, frame_len);
          break;
        } else {
          AERROR << "error: frame info is error, frame len = " <<  frame_len;
          continue;
        }
      }
    }
  }

  return 0;
}

bool SocketInput::input_available(int timeout) {
  (void)timeout;
  struct pollfd fds[1];
  fds[0].fd = _sockfd;
  fds[0].events = POLLIN;
  // Unfortunately, the Linux kernel recvfrom() implementation
  // uses a non-interruptible sleep() when waiting for data,
  // which would cause this method to hang if the device is not
  // providing data.  We poll() the device first to make sure
  // the recvfrom() will not block.
  // Note, however, that there is a known Linux kernel bug:
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
        AERROR << "hwcamera h265 port " << _port
          << "poll() error: " << strerror(errno);
      }
      return false;
    }

    if (retval == 0) {  // poll() timeout?
      return false;
    }

    if ((fds[0].revents & POLLERR) || (fds[0].revents & POLLHUP) ||
        (fds[0].revents & POLLNVAL)) {  // device error?
      AERROR << "h265 camera port " << _port  <<
        "poll() reports h265camera error";
      return false;
    }
  } while ((fds[0].revents & POLLIN) == 0);

  return true;
}

}  // namespace video
}  // namespace drivers
}  // namespace apollo




