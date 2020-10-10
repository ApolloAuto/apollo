/******************************************************************************
 * Copyright 2019 The Apollo Authors. All Rights Reserved.
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

#pragma once

#include <cstdint>

namespace apollo {
namespace drivers {
namespace video {

#ifdef __cplusplus
extern "C" {
#endif

typedef struct RtpHeader {
  uint16_t csrc_count : 4;
  uint16_t extension : 1;
  uint16_t padding : 1;
  uint16_t version : 2;
  uint16_t payloadtype : 7;
  uint16_t marker : 1;
  uint16_t seq;
  uint32_t timestamp;
  uint32_t ssrc;
} RtpHeader;

#define HW_CAMERA_MAGIC0 0xBBAABBAA
#define HW_CAMERA_MAGIC1 0xAABBAABB

typedef struct FrameHeader {
  uint32_t magic0; /* 0xBBAABBAA */
  uint32_t magic1; /* 0xAABBAABB */
  //  uint32_t ChanNo;
  uint8_t PhyNo;
  uint8_t frame_type; /* IDR: 1, other: 0 */
  uint8_t error;      /* error:1, other: 0 */
  uint8_t resv;
  uint32_t frame_size;
  uint32_t frame_id;
  uint32_t ts_sec;
  uint32_t ts_usec;
  uint16_t height; /* 1920 */
  uint16_t width;  /* 1080 */
  uint32_t format; /* H265 */
  uint32_t resv0;
  uint32_t resv1;
} FrameHeader;

typedef struct HwPduPacket {
  RtpHeader rtp_header;
  FrameHeader header;
} HwPduPacket;

#ifdef __cplusplus
}
#endif

}  // namespace video
}  // namespace drivers
}  // namespace apollo
