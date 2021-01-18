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
#include <pcl/common/time.h>

#include <fstream>
#include <memory>
#include <string>

#include "cyber/cyber.h"
#include "modules/drivers/lidar/lidar_robosense/include/parser/robosense_parser.h"

namespace apollo {
namespace drivers {
namespace robosense {

Robosense16Parser::Robosense16Parser(
    const apollo::drivers::suteng::SutengConfig& config)
    : RobosenseParser(config), _previous_packet_stamp(0), _gps_base_usec(0) {
  _need_two_pt_correction = false;
}

void Robosense16Parser::setup() {
  RobosenseParser::setup();
  init_orderindex();
  init_setup();
}
void Robosense16Parser::init_setup() {
  pic.col = 0;
  pic.distance.resize(RS16_DATA_NUMBER_PER_SCAN);
  pic.intensity.resize(RS16_DATA_NUMBER_PER_SCAN);
  pic.azimuthforeachP.resize(RS16_DATA_NUMBER_PER_SCAN);
  pic.timestamp.resize(RS16_DATA_NUMBER_PER_SCAN);
}

void Robosense16Parser::init_orderindex() {
  for (uint32_t i = 0; i < VLP16_POINT_SIZE; ++i) {
    order_map_[i] = getOrderIndex(i);
  }
}

bool Robosense16Parser::pkt_start = true;
uint64_t Robosense16Parser::base_stamp = 0;

void Robosense16Parser::generate_pointcloud(
    const std::shared_ptr<apollo::drivers::suteng::SutengScan const>& scan_msg,
    const std::shared_ptr<apollo::drivers::PointCloud>& out_msg) {
  out_msg->mutable_header()->set_frame_id(scan_msg->header().frame_id());
  out_msg->mutable_header()->set_lidar_timestamp(
      apollo::cyber::Time().Now().ToNanosecond());
  out_msg->mutable_header()->set_timestamp_sec(
      apollo::cyber::Time().Now().ToSecond());
  out_msg->set_height(1);
  _gps_base_usec = scan_msg->basetime();  // * 1000000UL;

  point_index_ = 0;
  // bool finish_packets_parse = false;

  uint32_t nan_pts = 0;

  for (int i = 0; i < scan_msg->firing_pkts_size(); ++i) {  // 84pkts 0-83
    // if (i == (scan_msg->firing_pkts_size() - 1)) {
    //  finish_packets_parse = true;
    // }
    unpack_robosense(scan_msg->firing_pkts(i), out_msg, &nan_pts);
    _last_time_stamp = out_msg->header().timestamp_sec();
  }

  if (out_msg->point_size() == 0) {
    // we discard this pointcloud if empty
    AERROR << " All points is NAN!Please check suteng:" << _config.model();
  } else {
    // auto size = out_msg->point_size();
    // AINFO<<"points per frame:"<<point_index_;

    uint64_t timestamp = out_msg->point(point_index_ - 1).timestamp();
    double d_time = apollo::cyber::Time(timestamp).ToSecond();
    // AINFO<<"d_time: "<<d_time;

    // out_msg->set_ratio_pass(static_cast<double>(point_index_ - nan_pts) /
    // point_index_);
    out_msg->set_height(point_index_ / RS16_SCANS_PER_FIRING);
    out_msg->set_width(RS16_SCANS_PER_FIRING);
    out_msg->set_is_dense(false);
    out_msg->set_measurement_time(d_time);
    out_msg->mutable_header()->set_lidar_timestamp(d_time * 1e9);
    out_msg->mutable_header()->set_timestamp_sec(
        apollo::cyber::Time().Now().ToSecond());
  }
}

uint64_t Robosense16Parser::get_timestamp(double base_time, float time_offset,
                                          uint16_t block_id) {
  (void)block_id;
  double t =
      base_time -
      time_offset;  //时秒 base_time 0-1h us微妙    time_offset 0-1278.72us
  uint64_t timestamp = Robosense16Parser::get_gps_stamp(
      t, &_previous_packet_stamp,
      &_gps_base_usec);  // _gps_base_usec gps基准时间 精度s
  // AINFO<<"timestamp       : "<<timestamp;
  return timestamp;
}

void Robosense16Parser::unpack_robosense(
    const apollo::drivers::suteng::SutengPacket& pkt,
    const std::shared_ptr<apollo::drivers::PointCloud>& cloud,
    uint32_t* nan_pts) {
  float azimuth = 0.f;  // 0.01degree
  float intensity = 0.f;
  float azimuth_diff = 0.f;
  float azimuth_corrected_f = 0.f;
  int azimuth_corrected = 0;
  uint64_t pkt_stamp = 0;

  const raw_packet_t* raw = (const raw_packet_t*)&pkt.data().c_str()[42];

  if (pkt_start) {
    pkt_start = false;
    base_stamp = _gps_base_usec;
    AINFO << "base_stamp timestamp: [" << base_stamp
          << "], which is same as first POS-GPS-timestamp";
  }
  pkt_stamp = pkt.stamp() + static_cast<uint64_t>(1e9);

  for (int block = 0; block < BLOCKS_PER_PACKET; ++block) {
    // AINFO<<"------block: "<<block;
    if (UPPER_BANK != raw->blocks[block].header) {
      AINFO << "skipping Suteng_liar DIFOP packet！";
      break;
    }

    if (temp_packet_num < 20000 && temp_packet_num > 0) {
      temp_packet_num++;
    } else {
      temper =
          compute_temperature(pkt.data().c_str()[38], pkt.data().c_str()[39]);
      temp_packet_num = 1;
    }
    azimuth = static_cast<float>(256 * raw->blocks[block].rotation_1 +
                                 raw->blocks[block].rotation_2);
    // AINFO<<"azimuth: "<<azimuth;

    if (block < (BLOCKS_PER_PACKET - 1)) {
      int azi1, azi2;
      azi1 = 256 * raw->blocks[block + 1].rotation_1 +
             raw->blocks[block + 1].rotation_2;
      azi2 =
          256 * raw->blocks[block].rotation_1 + raw->blocks[block].rotation_2;
      azimuth_diff = static_cast<float>((36000 + azi1 - azi2) % 36000);

      // AINFO<<"azimuth_diff: "<<azimuth_diff;
      if (azimuth_diff <= 0.0 || azimuth_diff > 75.0) {
        continue;
      }
    } else {  // packet的最后两个blocks=
      int azi1, azi2;
      azi1 =
          256 * raw->blocks[block].rotation_1 + raw->blocks[block].rotation_2;
      azi2 = 256 * raw->blocks[block - 1].rotation_1 +
             raw->blocks[block - 1].rotation_2;
      azimuth_diff = static_cast<float>((36000 + azi1 - azi2) % 36000);
      // AINFO<<"azimuth_diff last : "<<azimuth_diff;
      if (azimuth_diff <= 0.0 || azimuth_diff > 75.0) {
        continue;
      }
    }
    for (int firing = 0, k = 0; firing < RS16_FIRINGS_PER_BLOCK; firing++) {
      for (int dsr = 0; dsr < RS16_SCANS_PER_FIRING;
           dsr++, k += RAW_SCAN_SIZE) {
        // the azimuth of current point
        azimuth_corrected_f =
            azimuth +
            (azimuth_diff *
             ((dsr * RS16_DSR_TOFFSET) + (firing * RS16_FIRING_TOFFSET)) /
             RS16_BLOCK_TDURATION);
        azimuth_corrected = (static_cast<int>(round(azimuth_corrected_f))) %
                            36000;  // convert to integral
                                    // value... //use

        // AINFO<<"azimuth_corrected:"<<azimuth_corrected/100;
        union two_bytes raw_dist;
        raw_dist.bytes[1] = raw->blocks[block].data[k];
        raw_dist.bytes[0] = raw->blocks[block].data[k + 1];
        float distance = raw_dist.uint;

        // read intensity

        intensity = raw->blocks[block].data[k + 2];
        // intensity = CalibIntensity(intensity, dsr, distance, temper); //use
        // AINFO<<"intensity: "<<intensity;
        // distance
        // float distance2 = PixelToDistance(distance, dsr, temper);
        float distance2 = distance;
        distance2 = distance2 * DISTANCE_RESOLUTION;  // * 6.4/5.4; //use
        // AINFO<<"distance2: "<<distance2;
        // time of each point
        // block 0-11  firing 0-1   dsr 0-15
        uint64_t time_pt = (block * 2 + firing) * 50 + dsr * 3;  // us
        uint64_t timestamp = pkt_stamp + time_pt * 1e3;
        // uint64_t timestamp = get_poitn_timestamp(basetime, time_pt);
        if (block == BLOCKS_PER_PACKET - 1 &&
            firing == RS16_FIRINGS_PER_BLOCK - 1 &&
            dsr == RS16_SCANS_PER_FIRING - 1) {
          cloud->mutable_header()->set_lidar_timestamp(timestamp);
          cloud->set_measurement_time(static_cast<double>((timestamp) / 1e9));
        }

        if (raw_dist.uint == 0 || distance2 < _config.min_range() ||
            distance2 > _config.max_range()) {
          if (_config.organized()) {
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
        // AINFO<<"timestamp: "<<timestamp;
        apollo::drivers::PointXYZIT* point = cloud->add_point();
        point->set_timestamp(timestamp);
        point->set_intensity(intensity);

        // angle
        float arg_hori = static_cast<float>(azimuth_corrected / 18000.f * M_PI);
        float arg_vert = SUTENG_VERT[dsr];
        float y =
            static_cast<float>(-distance2 * cos(arg_vert) * sin(arg_hori));
        float x = static_cast<float>(distance2 * cos(arg_vert) * cos(arg_hori));
        float z = static_cast<float>(distance2 * sin(arg_vert));
        if (_filter_set.size() > 0) {
          std::string key =
              std::to_string(static_cast<float>(x * 100 / _filter_grading)) +
              "+" +
              std::to_string(static_cast<float>(y * 100 / _filter_grading));
          if (_filter_set.find(key) != _filter_set.end()) {
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
        // AINFO << "RoboUnpack->point x:" << point->x() << "  y:" << point->y()
        //   << "  z:" << point->z()
        //   << "  intensity:" << int(point->intensity());
      }
    }
  }
}

uint32_t Robosense16Parser::GetPointSize() { return VLP16_POINT_SIZE; }

uint32_t Robosense16Parser::getOrderIndex(uint32_t index) {
  uint32_t width = 16;
  uint32_t height_index = index / width;
  uint32_t width_index = index % width;
  uint32_t order_index =
      height_index * width + robosense::REORDER_16[width_index];
  return order_index;
}

void Robosense16Parser::order(
    const std::shared_ptr<apollo::drivers::PointCloud>& cloud) {
  int width = 16;
  cloud->set_width(width);
  int height = cloud->point_size() / cloud->width();
  cloud->set_height(height);
}

}  // namespace robosense
}  // namespace drivers
}  // namespace apollo
