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

#ifndef LIDAR_HESAI_SRC_INPUT_H_
#define LIDAR_HESAI_SRC_INPUT_H_

#include <netinet/in.h>
#include <stdio.h>
#include <unistd.h>
#include <string>

#include "cyber/cyber.h"
#include "modules/drivers/hesai/const_var.h"
#include "modules/drivers/hesai/type_defs.h"

namespace apollo {
namespace drivers {
namespace hesai {

class Input {
 public:
  Input(uint16_t port, uint16_t gpsPort);
  ~Input();
  int GetPacket(HesaiPacket *pkt);

 private:
  int socketForLidar = -1;
  int socketForGPS = -1;
  int socketNumber = -1;
};

}  // namespace hesai
}  // namespace drivers
}  // namespace apollo

#endif  // SRC_INPUT_H_
