/******************************************************************************
 * Copyright 2021 The Apollo Authors. All Rights Reserved.
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
#include "modules/drivers/lidar/robosense/parser/robosense16p_parser.h"

#include <fstream>
#include <memory>
#include <string>

#include <pcl/common/time.h>

#include "cyber/cyber.h"

namespace apollo {
namespace drivers {
namespace robosense {

Robosense16PParser::Robosense16PParser(
    const apollo::drivers::suteng::SutengConfig& config)
    : RobosenseParser(config) {}

void Robosense16PParser::setup() {
  RobosenseParser::setup();
  init_params();
}

void Robosense16PParser::init_params() {}

bool Robosense16PParser::pkt_start = true;
uint64_t Robosense16PParser::base_stamp = 0;

void Robosense16PParser::generate_pointcloud(
    const std::shared_ptr<apollo::drivers::suteng::SutengScan const>& scan_msg,
    const std::shared_ptr<apollo::drivers::PointCloud>& out_msg) {
  out_msg->mutable_header()->set_frame_id(scan_msg->header().frame_id());
  out_msg->mutable_header()->set_lidar_timestamp(
      scan_msg->header().lidar_timestamp());
  out_msg->mutable_header()->set_timestamp_sec(
      apollo::cyber::Time().Now().ToSecond());
  out_msg->set_height(1);

  point_index_ = 0;
  uint32_t nan_pts = 0;

  for (int i = 0; i < scan_msg->positioning_pkts_size(); ++i) {
    unpack_params(scan_msg->positioning_pkts(i));
  }

  for (int i = 0; i < scan_msg->firing_pkts_size(); ++i) {  // 75pkts
    unpack_robosense(scan_msg->firing_pkts(i), out_msg, &nan_pts);
    last_time_stamp_ = out_msg->header().timestamp_sec();
  }

  if (out_msg->point_size() == 0) {
    // we discard this pointcloud if empty
    AERROR << " All points is NAN!Please check suteng:" << config_.model();
  } else {
    uint64_t timestamp = out_msg->point(point_index_ - 1).timestamp();
    double d_time = apollo::cyber::Time(timestamp).ToSecond();
    out_msg->set_height(point_index_ / SCANS_PER_FIRING);
    out_msg->set_width(SCANS_PER_FIRING);
    out_msg->set_is_dense(false);
    out_msg->set_measurement_time(d_time);
    out_msg->mutable_header()->set_lidar_timestamp(d_time * 1e9);
    out_msg->mutable_header()->set_timestamp_sec(
        apollo::cyber::Time().Now().ToSecond());
  }
}

float Robosense16PParser::parse_angle(angle_16p_t angle_pkt) {
  float angle;
  int32_t v;
  if (angle_pkt.sign == 0xFF) {
    AERROR << "Get err in angle sign";
    return 0.0f;
  }

  v = ntohs(angle_pkt.value);
  if (angle_pkt.sign != 0) v = -v;
  angle = static_cast<float>(v * DEGREE_TO_RADIAN);

  return angle;
}

void Robosense16PParser::unpack_params(
    const apollo::drivers::suteng::SutengPacket& pkt) {
  if (last_difop_time_ < pkt.stamp()) {
    last_difop_time_ = pkt.stamp();
    vertical_angles_16p_t* verti_angles =
        reinterpret_cast<vertical_angles_16p_t*>(
            const_cast<char*>(&pkt.data().c_str()[468]));
    horizontal_angles_16p_t* hori_angles =
        reinterpret_cast<horizontal_angles_16p_t*>(
            const_cast<char*>(&pkt.data().c_str()[564]));
    for (int i = 0; i < SCANS_PER_FIRING; i++) {
      SUTENG_VERT_16P_[i] = parse_angle(verti_angles->angles[i]);
      cor_hori_angles[i] = parse_angle(hori_angles->angles[i]);
    }
  }
}

void Robosense16PParser::unpack_robosense(
    const apollo::drivers::suteng::SutengPacket& pkt,
    const std::shared_ptr<apollo::drivers::PointCloud>& cloud,
    uint32_t* nan_pts) {
  float azimuth = 0.f;  // 0.01degree
  float intensity = 0.f;
  float distance = 0.f;
  float azimuth_diff = 0.f;
  float azimuth_corrected_f = 0.f;
  int azimuth_corrected = 0;
  uint64_t pkt_stamp = pkt.stamp();

  const raw_packet_16p_t* raw =
      (const raw_packet_16p_t*)&pkt.data().c_str()[42];

  for (int block = 0; block < BLOCKS_PER_PACKET; ++block) {
    if (UPPER_BANK != raw->blocks[block].header) {
      AINFO << "skipping Suteng_liar DIFOP packet！"
            << raw->blocks[block].header;
      break;
    }

    azimuth = static_cast<float>(ntohs(raw->blocks[block].rotation));

    if (block < (BLOCKS_PER_PACKET - 1)) {
      int azi1, azi2;
      azi1 = ntohs(raw->blocks[block + 1].rotation);
      azi2 = ntohs(raw->blocks[block].rotation);
      azimuth_diff = static_cast<float>((36000 + azi1 - azi2) % 36000);
      if (azimuth_diff <= 0.0 || azimuth_diff > 75.0) {
        continue;
      }
    } else {  // the last two blocks in packet
      int azi1, azi2;
      azi1 = ntohs(raw->blocks[block].rotation);
      azi2 = ntohs(raw->blocks[block - 1].rotation);
      azimuth_diff = static_cast<float>((36000 + azi1 - azi2) % 36000);
      if (azimuth_diff <= 0.0 || azimuth_diff > 75.0) {
        continue;
      }
    }
    for (int firing = 0; firing < FIRINGS_PER_BLOCK; firing++) {
      for (int dsr = 0; dsr < SCANS_PER_FIRING; dsr++) {
        int idx = firing * SCANS_PER_FIRING + dsr;
        azimuth_corrected_f = azimuth + azimuth_diff *
                                            (TIME_OFFSET_16P[idx][block] -
                                             TIME_OFFSET_16P[0][block]) /
                                            TIME_OFFSET_16P[0][1];

        azimuth_corrected = (static_cast<int>(round(azimuth_corrected_f))) %
                            36000;  // convert to integral

        distance = ntohs(raw->blocks[block].channel_data[idx].distance) *
                   DISTANCE_RESOLUTION_16P;

        // time of each point
        // block 0-11  firing 0-1   dsr 0-15
        uint64_t time_pt = TIME_OFFSET_16P[idx][block];

        uint64_t timestamp = pkt_stamp + time_pt * 1e3;

        if (distance == 0 || distance < config_.min_range() ||
            distance > config_.max_range()) {
          if (config_.organized()) {
            apollo::drivers::PointXYZIT* point = cloud->add_point();
            point->set_x(nan);
            point->set_y(nan);
            point->set_z(nan);
            point->set_timestamp(timestamp);
            point->set_intensity(0);
            ++point_index_;
            ++nan_pts;
          }
          continue;
        }
        apollo::drivers::PointXYZIT* point = cloud->add_point();
        point->set_timestamp(timestamp);

        intensity = raw->blocks[block].channel_data[idx].reflectivity;
        point->set_intensity(intensity);

        // angle
        float arg_hori =
            static_cast<float>(azimuth_corrected * DEGREE_TO_RADIAN);
        float arg_hori_cor = arg_hori + cor_hori_angles[dsr];
        float arg_vert = SUTENG_VERT_16P_[dsr];

        float y =
            static_cast<float>(-distance * cos(arg_vert) * sin(arg_hori_cor) -
                               RX_16P * sin(arg_hori));
        float x =
            static_cast<float>(distance * cos(arg_vert) * cos(arg_hori_cor) +
                               RX_16P * cos(arg_hori));
        float z = static_cast<float>(distance * sin(arg_vert));

        if (filter_set_.size() > 0) {
          std::string key =
              std::to_string(static_cast<float>(x * 100 / filter_grading_)) +
              "+" +
              std::to_string(static_cast<float>(y * 100 / filter_grading_));
          if (filter_set_.find(key) != filter_set_.end()) {
            x = NAN;
            y = NAN;
            z = NAN;
            ++nan_pts;
          }
        }
        point->set_x(x);
        point->set_y(y);
        point->set_z(z);
        point->set_intensity(intensity);
        ++point_index_;
      }
    }
  }
}
}  // namespace robosense
}  // namespace drivers
}  // namespace apollo
