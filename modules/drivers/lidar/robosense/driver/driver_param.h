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
#pragma once
#include <map>
#include <string>
namespace apollo {
namespace drivers {
namespace robosense {
enum LidarType  ///< The lidar type
{ RSAUTO = 0,   ///< If LidarType is set to RSAUTO, the driver will check the
                ///< lidar type automatically.(Only support the LiDARs of latest
                ///< version, not support Bpearl & RS80 yet.)
  RS16 = 1,
  RS32 = 2,
  RSBP = 3,
  RS128 = 4,
  RS80 = 5 };

enum SplitFrameMode {
  SPLIT_BY_ANGLE = 1,
  SPLIT_BY_FIXED_PKTS = 2,
  SPLIT_BY_CUSTOM_PKTS = 3
};

typedef struct RSCameraTriggerParam {
  std::map<double, std::string>
      trigger_map;  ///< The map stored the trigger angle and camera frame id
} RSCameraTriggerParam;

typedef struct RSDecoderParam {
  float max_distance = 200.0f;  ///< The max distance of point cloud range
  float min_distance = 0.2f;    ///< The minimum distance of point cloud range
  float start_angle = 0.0f;     ///< The start angle of point cloud
  float end_angle = 360.0f;     ///< The end angle of point cloud
  SplitFrameMode split_frame_mode =
      SplitFrameMode::SPLIT_BY_ANGLE;  ///< 1: Split frames by cut_angle; 2:
                                       ///< Split frames by fixed number of
                                       ///< packets; 3: Split frames by  custom
                                       ///< number of packets (num_pkts_split)
  uint32_t num_pkts_split = 1;  ///< The number of packets in one frame, only be
                                ///< used when split_frame_mode=3
  float cut_angle = 0.0f;  ///< The cut angle(degree) used to split frame, only
                           ///< be used when split_frame_mode=1
  bool use_lidar_clock =
      false;  ///< true: lidar message timestamp is the lidar clock
              ///< false: timestamp is the computer system clock
  RSCameraTriggerParam trigger_param;  ///< The parameter used to trigger camera
} RSDecoderParam;

typedef struct RSInputParam {
  std::string device_ip = "192.168.1.200";  ///< The ip of lidar
  uint16_t msop_port = 6699;                ///< The msop packet port number
  uint16_t difop_port = 7788;               ///< The difop packet port number
  bool read_pcap =
      false;  ///< true: The driver will process the pcap through pcap_path.
              ///< false: The driver will get data from online lidar
  double pcap_rate = 1;            ///< The rate to read the pcap file
  bool pcap_repeat = true;         ///< true: The pcap bag will repeat play
  std::string pcap_path = "null";  ///< The absolute path of pcap file
} RSInputParam;

typedef struct RSDriverParam {
  RSInputParam input_param;      ///< The input parameter
  RSDecoderParam decoder_param;  ///< The decoder parameter
  std::string angle_path =
      "null";  ///< The path of angle calibration files(angle.csv)
               ///< For the latest version lidar, this file is not needed
  std::string frame_id = "rslidar";        ///< The frame id of lidar message
  LidarType lidar_type = LidarType::RS16;  ///< Lidar type
  bool wait_for_difop =
      true;  ///< true: start sending point cloud until receive difop packet
  LidarType strToLidarType(const std::string& type) {
    if (type == "RS16") {
      return LidarType::RS16;
    } else if (type == "RS32") {
      return LidarType::RS32;
    } else if (type == "RSBP") {
      return LidarType::RSBP;
    } else if (type == "RS128") {
      return LidarType::RS128;
    } else if (type == "RS80") {
      return LidarType::RS80;
    } else if (type == "RSAUTO") {
      return LidarType::RSAUTO;
    } else {
      // RS_ERROR << "Wrong lidar type: " << type << RS_REND;
      // RS_ERROR << "Please setup the correct type: RS16, RS32, RSBP, RS128,
      // RS80, RSAUTO" << RS_REND;
      exit(-1);
    }
  }
} RSDriverParam;
}  // namespace robosense
}  // namespace drivers
}  // namespace apollo
