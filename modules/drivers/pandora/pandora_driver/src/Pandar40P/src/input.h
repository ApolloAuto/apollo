/******************************************************************************
 * Copyright 2018 The Apollo Authors. All Rights Reserved.
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

#ifndef SRC_INPUT_H_
#define SRC_INPUT_H_

#include <netinet/in.h>
#include <pcap.h>
#include <stdio.h>
#include <unistd.h>
#include <string>

namespace apollo {
namespace drivers {
namespace hesai {

#define ETHERNET_MTU (1500)

typedef struct PandarPacket_s {
  double stamp;
  uint8_t data[ETHERNET_MTU];
  uint32_t size;
} PandarPacket;

class Input {
 public:
  Input(uint16_t port, uint16_t gpsPort);
  ~Input();
  Input(std::string filePath, int type);
  int getPacket(PandarPacket *pkt);
  int getPacketFromPcap(PandarPacket *pkt);

 private:
  int socketForLidar;
  int socketForGPS;
  int socketNumber;
};

}  // namespace hesai
}  // namespace drivers
}  // namespace apollo

#endif  // SRC_INPUT_H_
