/******************************************************************************
 * copyright 2020 The Apollo Authors. All Rights Reserved.
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

#include <fstream>
#include <memory>
#include <string>

#include <pcl/common/time.h>

#include "cyber/cyber.h"
#include "modules/drivers/lidar_surestar/parser/surestar_parser.h"

namespace apollo {
namespace drivers {
namespace surestar {

Surestar16Parser::Surestar16Parser(
    const apollo::drivers::surestar::SurestarConfig& config)
    : SurestarParser(config), _previous_packet_stamp(0), _gps_base_usec(0) {
  _inner_time = &surestar::INNER_TIME_16;
  _need_two_pt_correction = false;
}

void Surestar16Parser::setup() {
  SurestarParser::setup();
  init_orderindex();
}

void Surestar16Parser::init_orderindex() {
  for (uint32_t i = 0; i < RFANS16_POINT_SIZE; ++i) {
    order_map_[i] = getOrderIndex(i);
  }
}

void Surestar16Parser::generate_pointcloud(
    const std::shared_ptr<apollo::drivers::Surestar::SurestarScan const>&
        scan_msg,
    const std::shared_ptr<apollo::drivers::PointCloud>& out_msg) {
  // allocate a point cloud with same time and frame ID as raw data
  out_msg->mutable_header()->set_frame_id(scan_msg->header().frame_id());
  // out_msg->mutable_header()->set_lidar_timestamp(apollo::cyber::Time().Now().ToNanosecond());
  out_msg->mutable_header()->set_timestamp_sec(
      apollo::cyber::Time().Now().ToSecond());
  out_msg->set_height(1);
  _gps_base_usec = scan_msg->basetime() * 1000000UL;

  point_index_ = 0;
  uint32_t nan_pts = 0;
  for (int i = 0; i < scan_msg->firing_pkts_size(); ++i) {
    unpack(scan_msg->firing_pkts(i), out_msg, &nan_pts);
    _last_time_stamp = out_msg->header().lidar_timestamp();
  }

  if (out_msg->point_size() == 0) {
    // we discard this pointcloud if empty
    AERROR << " All points is NAN!Please check surestar:" << _config.model();
  } else {
    out_msg->point(point_index_ - 1).timestamp();
    out_msg->set_measurement_time(_last_time_stamp / 1e9);
    out_msg->mutable_header()->set_lidar_timestamp(_last_time_stamp);
  }
}

uint64_t Surestar16Parser::get_timestamp(double base_time, float time_offset,
                                         uint16_t block_id) {
  (void)block_id;
  double t =
      base_time -
      time_offset;  // 时秒 base_time 0-1h us微妙    time_offset 0-1278.72us
  uint64_t timestamp = Surestar16Parser::get_gps_stamp(
      t, &_previous_packet_stamp,
      &_gps_base_usec);  // _gps_base_usec gps基准时间 精度s
  return timestamp;
}

/** @brief convert raw packet to point cloud
 *
 *  @param pkt raw packet to unpack
 *  @param pc shared pointer to point cloud (points are appended)
 */
void Surestar16Parser::unpack(
    const apollo::drivers::Surestar::SurestarPacket& pkt,
    const std::shared_ptr<apollo::drivers::PointCloud>& pc, uint32_t* nan_pts) {
  float azimuth = 0.0;
  float azimuth_diff = 0.0;
  float last_azimuth_diff = 0.0;
  float azimuth_corrected_f = 0.0;
  int azimuth_corrected = 0.0;

  const RawPacket* raw = (const RawPacket*)pkt.data().c_str();
  double basetime = raw->gps_timestamp;  // usec   0-1h us微妙

  // for 0-12
  for (int block = 0; block < BLOCKS_PER_PACKET; block++) {
    azimuth = static_cast<float>(raw->blocks[block].rotation);
    if (block < (BLOCKS_PER_PACKET - 1)) {
      azimuth_diff =
          static_cast<float>((36000 + raw->blocks[block + 1].rotation -
                              raw->blocks[block].rotation) %
                             36000);
      last_azimuth_diff = azimuth_diff;
    } else {
      azimuth_diff = last_azimuth_diff;
    }

    float tmpAngleDif = azimuth_diff / SCANS_PER_BLOCK;
    // for 0-2
    for (int firing = 0, k = 0; firing < RFANS16_FIRINGS_PER_BLOCK; ++firing) {
      // for 0-16
      for (int dsr = 0; dsr < RFANS16_SCANS_PER_FIRING;
           ++dsr, k += RAW_SCAN_SIZE) {
        // LaserCorrection& corrections = _calibration._laser_corrections[dsr];

        /** Position Calculation */
        union RawDistance raw_distance;
        raw_distance.bytes[0] = raw->blocks[block].data[k];
        raw_distance.bytes[1] = raw->blocks[block].data[k + 1];

        /** correct for the laser rotation as a function of timing during the
         * firings **/
        azimuth_corrected_f =
            azimuth +
            tmpAngleDif * ((static_cast<float>(dsr)) +  // dsr [0-15]
                           (static_cast<float>(firing) *
                            RFANS16_SCANS_PER_FIRING));  // firing [0-1]
        azimuth_corrected =
            static_cast<int>(round(fmod(azimuth_corrected_f, 36000.0)));

        // set 4th param to LOWER_BANK, only use _lower_gps_base_usec and
        // _lower_previous_packet_stamp
        uint64_t timestamp = get_timestamp(
            basetime,
            (*_inner_time)[block][firing * RFANS16_SCANS_PER_FIRING + dsr],
            LOWER_BANK);  // _inner_time[12][32] 一个packet里面每个点的时间

        if (block == BLOCKS_PER_PACKET - 1 &&
            firing == RFANS16_FIRINGS_PER_BLOCK - 1 &&
            dsr == RFANS16_SCANS_PER_FIRING - 1) {
          // set header stamp before organize the point cloud
          pc->mutable_header()->set_lidar_timestamp(timestamp);
          pc->set_measurement_time(static_cast<double>(timestamp) / 1e9);
        }

        float distance = raw_distance.raw_distance * DISTANCE_RESOLUTION;

        if (raw_distance.raw_distance == 0 ||
            !is_scan_valid(azimuth_corrected, distance)) {
          // if orgnized append a nan point to the cloud
          if (_config.organized()) {
            apollo::drivers::PointXYZIT* point =
                pc->mutable_point(order_map_[point_index_]);
            point->set_x(nan);
            point->set_y(nan);
            point->set_z(nan);
            point->set_timestamp(timestamp);
            point->set_intensity(0);
            ++point_index_;
            (*nan_pts)++;
          }
          continue;
        }

        apollo::drivers::PointXYZIT* point =
            pc->mutable_point(order_map_[point_index_]);
        point->set_timestamp(timestamp);

        compute_coords_beike(distance, static_cast<uint16_t>(azimuth_corrected),
                             dsr, point, nan_pts);  // dsr [0-15]
        point->set_intensity(raw->blocks[block].data[k + 2]);
        ++point_index_;
      }
    }
  }
}

uint32_t Surestar16Parser::GetPointSize() { return RFANS16_POINT_SIZE; }

uint32_t Surestar16Parser::getOrderIndex(uint32_t index) {
  uint32_t width = 16;
  uint32_t height_index = index / width;
  uint32_t width_index = index % width;
  uint32_t order_index =
      height_index * width + surestar::REORDER_16[width_index];
  return order_index;
}

void Surestar16Parser::order(
    const std::shared_ptr<apollo::drivers::PointCloud>& cloud) {
  int width = 16;
  cloud->set_width(width);
  int height = cloud->point_size() / cloud->width();
  cloud->set_height(height);
}

}  // namespace surestar
}  // namespace drivers
}  // namespace apollo
