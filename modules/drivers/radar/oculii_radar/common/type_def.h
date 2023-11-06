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

#ifndef RADAR_OCULII_TYPE_DEF_H
#define RADAR_OCULII_TYPE_DEF_H

#include <memory>
#include "modules/drivers/radar/oculii_radar/common/const_val.h"

namespace apollo {
namespace drivers {
namespace radar {

typedef struct OculiiHandshake {
  unsigned char magic[OCULII_HANDSHAKE_MAGIC_LENGTH];
  uint32_t data_length;
  unsigned char reserved[OCULII_HANDSHAKE_RESERVED_LENGTH];
} Oculii_handshake;

typedef struct OculiiHeader {
  unsigned char magic[OCULII_HEADER_MAGIC_LENGTH];
  uint32_t frame_number;
  uint32_t version_number;
  uint16_t dections_number;
  uint16_t tracks_number;
  int16_t host_speed;
  int16_t host_angle;
  unsigned char reserved[OCULII_HEADER_RESERVED_LENGTH];
  uint16_t range_accuracy_idx;
  uint16_t doppler_accuracy_idx;
  uint16_t azimuth_accuracy_idx;
  uint16_t elevation_accuracy_idx;
  unsigned char dsp_workload;
  unsigned char arm_workload;
  unsigned char tail_reserved[OCULII_HEADER_RESERVED_TAIL_LENGTH];
} Oculii_header;

typedef struct OculiiDetection {
  unsigned char raw[OCULII_DETECTION_BLOCK_SIZE];  // need mask 0x40
} OculiiDetection;

typedef struct OculiiTrack {
  uint32_t id;
  int16_t x_pos;
  int16_t y_pos;
  uint16_t z_pos;
  int16_t x_dot;
  int16_t y_dot;
  int16_t z_dot;
  unsigned char reserved[OCULII_DECTION_BLOCK_SIZE];
  uint16_t internal_track_flag;
  uint16_t track_class;
  uint16_t confidence;
  unsigned char tail_reserved[OCULII_TRACKER_RESERVED_TAIL_LENGTH];
} OculiiTrack;

typedef struct OculiiFooter {
  long long reserved;   // NOLINT
  uint16_t range_accuracy_idx;
  uint16_t doppler_accuracy_idx;
  uint16_t azimuth_accuracy_idx;
  uint16_t elevation_accuracy_idx;
  unsigned char tail_reserved[OCULII_FOOTER_RESERVED_TAIL_LENGTH];
} OculiiFooter;

}  // namespace radar
}  // namespace drivers
}  // namespace apollo

#endif
