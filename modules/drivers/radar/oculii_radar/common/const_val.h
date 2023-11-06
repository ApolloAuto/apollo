/******************************************************************************
 * Copyright 2023 The Apollo Authors. All Rights Reserved.
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

#include <cmath>

#ifndef RADAR_OCULII_CONST_VAL_H
#define RADAR_OCULII_CONST_VAL_H

namespace apollo {
namespace drivers {
namespace radar {
  const int OCULII_QUEUE_SIZE = 65536;
  const int OCULII_MAX_MESSAGE_LENGTH = 256;
  const int OCULII_HANDSHAKE_MAGIC_LENGTH = 8;
  const int OCULII_HANDSHAKE_RESERVED_LENGTH = 12;
  const int OCULII_HANDSHAKE_SIZE = 24;

  const int OCULII_HEADER_MAGIC_LENGTH = 8;
  const int OCULII_HEADER_RESERVED_LENGTH = 8;
  const int OCULII_HEADER_RESERVED_TAIL_LENGTH = 6;
  const int OCULII_HEADER_SIZE = 48;

  const int OCULII_DECTION_BLOCK_SIZE = 8;
  enum DETECTION_FLAG {
    USING_IDX_HEADER = 0x00,
    USING_IDX_FOOTER = 0x40
  };

  const int OCULII_TRACKER_BLOCK_SIZE = 32;
  const int OCULII_TRACKER_RESERVED_LENGTH = 6;
  const int OCULII_TRACKER_RESERVED_TAIL_LENGTH = 4;

  const int OCULII_DETECTION_BLOCK_SIZE = 32;

  const int OCULII_FOOTER_BLOCK_LENGTH = 32;
  const int OCULII_FOOTER_RESERVED_LENGTH = 8;
  const int OCULII_FOOTER_RESERVED_TAIL_LENGTH = 16;

  const int ETHERNET_MTU = 1500;

  const float DEG2RAD = std::atan(1) * 4 / 180;

}  // namespace radar
}  // namespace drivers
}  // namespace apollo

#endif
