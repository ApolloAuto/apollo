/******************************************************************************
 * Copyright 2019 The CiDi Authors. All Rights Reserved.
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

#ifndef DRIVERS_STREAM_TCP_STREAM_H
#define DRIVERS_STREAM_TCP_STREAM_H

namespace apollo {
namespace drivers {
namespace cidiv2x {

class TcpStream : public Stream {
  typedef uint16_t be16_t;
  typedef uint32_t be32_t;

 public:
  TcpStream(const char *address, uint16_t port, uint32_t timeout_usec,
            bool auto_reconnect = true);
  ~TcpStream();

  virtual bool Connect();
  virtual bool Disconnect();
  virtual size_t read(uint8_t *buffer, size_t max_length, uint8_t flag = 0);
  virtual size_t write(const uint8_t *data, size_t length, uint8_t flag = 0);

 private:
  bool Reconnect();
  bool Readable(uint32_t timeout_us);
  TcpStream() {}
  void open();
  void close();
  bool InitSocket();
  be16_t peer_port_ = 0;
  be32_t peer_addr_ = 0;
  uint32_t timeout_usec_ = 0;
  int sockfd_ = -1;
  int errno_ = 0;
  bool auto_reconnect_ = false;
};

}  // namespace cidiv2x
}  // namespace drivers
}  // namespace apollo

#endif
