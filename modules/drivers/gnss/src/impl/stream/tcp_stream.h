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

#ifndef DRIVERS_GNSS_TCP_STREAM_H
#define DRIVERS_GNSS_TCP_STREAM_H

namespace apollo {
namespace drivers {
namespace gnss {

class TcpStream : public Stream {
  typedef uint16_t be16_t;
  typedef uint32_t be32_t;

 public:
  TcpStream(const char *address, uint16_t port,
            uint32_t timeout_usec, bool auto_reconnect = true);
  ~TcpStream();

  virtual bool connect();
  virtual bool disconnect();
  virtual size_t read(uint8_t *buffer, size_t max_length);
  virtual size_t write(const uint8_t *data, size_t length);

 private:
  bool reconnect();
  bool readable(uint32_t timeout_us);
  TcpStream() {}
  void open();
  void close();
  bool init_socket();
  be16_t _peer_port = 0;
  be32_t _peer_addr = 0;
  uint32_t _timeout_usec = 0;
  int _sockfd = -1;
  int _errno = 0;
  bool _auto_reconnect = false;
};

}  // namespace gnss
}  // namespace drivers
}  // namespace apollo

#endif
