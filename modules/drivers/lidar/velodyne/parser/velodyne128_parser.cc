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

#include "modules/drivers/lidar/velodyne/parser/velodyne_parser.h"

namespace apollo {
namespace drivers {
namespace velodyne {

Velodyne128Parser::Velodyne128Parser(const Config& config)
    : VelodyneParser(config), previous_packet_stamp_(0), gps_base_usec_(0) {
  inner_time_ = &velodyne::INNER_TIME_128;
  need_two_pt_correction_ = false;
}

void Velodyne128Parser::GeneratePointcloud(
    const std::shared_ptr<VelodyneScan>& scan_msg,
    std::shared_ptr<PointCloud> out_msg) {
  // allocate a point cloud with same time and frame ID as raw data
  out_msg->mutable_header()->set_frame_id(scan_msg->header().frame_id());
  out_msg->mutable_header()->set_timestamp_sec(cyber::Time().Now().ToSecond());
  out_msg->set_height(1);

  // us
  gps_base_usec_ = scan_msg->basetime();

  for (int i = 0; i < scan_msg->firing_pkts_size(); ++i) {
    Unpack(scan_msg->firing_pkts(i), out_msg);
    last_time_stamp_ = out_msg->measurement_time();
  }

  size_t size = out_msg->point_size();
  if (size == 0) {
    // we discard this pointcloud if empty
    AERROR << "All points is NAN!Please check velodyne:" << config_.model();
    return;
  } else {
    const auto timestamp =
        out_msg->point(static_cast<int>(size) - 1).timestamp();
    out_msg->set_measurement_time(static_cast<double>(timestamp) / 1e9);
    out_msg->mutable_header()->set_lidar_timestamp(timestamp);
  }
  out_msg->set_width(out_msg->point_size());
}

uint64_t Velodyne128Parser::GetTimestamp(double base_time, float time_offset,
                                         uint16_t block_id) {
  (void)block_id;
  double t = base_time + time_offset;
  uint64_t timestamp = GetGpsStamp(t, &previous_packet_stamp_, &gps_base_usec_);
  return timestamp;
}

void Velodyne128Parser::Order(std::shared_ptr<PointCloud> cloud) {
  (void)cloud;
}

void Velodyne128Parser::Unpack(const VelodynePacket& pkt,
                               std::shared_ptr<PointCloud> pc) {
  float azimuth_diff, azimuth_corrected_f;
  float last_azimuth_diff = 0.0f;
  uint16_t azimuth = 0;
  uint16_t azimuth_next = 0;
  uint16_t azimuth_corrected = 0;
  const RawPacket* raw = (const RawPacket*)pkt.data().c_str();
  double basetime = raw->gps_timestamp;

  for (int block = 0; block < BLOCKS_PER_PACKET; block++) {
    // Calculate difference between current and next block's azimuth angle.
    if (block == 0) {
      azimuth = raw->blocks[block].rotation;
    } else {
      azimuth = azimuth_next;
    }
    if (block < (BLOCKS_PER_PACKET - 1)) {
      azimuth_next = raw->blocks[block + 1].rotation;
      azimuth_diff =
          static_cast<float>((36000 + azimuth_next - azimuth) % 36000);
      last_azimuth_diff = azimuth_diff;
    } else {
      azimuth_diff = last_azimuth_diff;
    }

    /*condition added to avoid calculating points which are not
      in the interesting defined area (min_angle < area < max_angle)*/
    for (int j = 0, k = 0; j < SCANS_PER_BLOCK; j++, k += RAW_SCAN_SIZE) {
      uint8_t group = static_cast<uint8_t>(block % 4);
      uint8_t chan_id = static_cast<uint8_t>(j + group * 32);
      uint8_t firing_order = chan_id / 8;
      firing_order = 0;
      LaserCorrection& corrections = calibration_.laser_corrections_[chan_id];
      // distance extraction
      // union two_bytes tmp;
      union RawDistance raw_distance;
      raw_distance.bytes[0] = raw->blocks[block].data[k];
      raw_distance.bytes[1] = raw->blocks[block].data[k + 1];

      float real_distance =
          raw_distance.raw_distance * VSL128_DISTANCE_RESOLUTION;
      float distance = real_distance + corrections.dist_correction;

      uint64_t timestamp = static_cast<uint64_t>(GetTimestamp(
          basetime, (*inner_time_)[block][j], static_cast<uint16_t>(block)));
      if (!is_scan_valid(azimuth, distance)) {
        // todo organized
        if (config_.organized()) {
          apollo::drivers::PointXYZIT* point_new = pc->add_point();
          point_new->set_x(nan);
          point_new->set_y(nan);
          point_new->set_z(nan);
          point_new->set_timestamp(timestamp);
          point_new->set_intensity(0);
        }
        continue;
      }

      // if (pointInRange(distance)) {
      int intensity = static_cast<int>(raw->blocks[block].data[k + 2]);

      /** correct for the laser rotation as a function of timing during the
       * firings **/
      azimuth_corrected_f =
          azimuth +
          (azimuth_diff * (firing_order * CHANNEL_TDURATION) / SEQ_TDURATION);
      azimuth_corrected =
          (static_cast<uint16_t>(round(azimuth_corrected_f))) % 36000;

      // add new point
      PointXYZIT* point_new = pc->add_point();

      // compute time , time offset is zero
      point_new->set_timestamp(timestamp);
      ComputeCoords(real_distance, corrections, azimuth_corrected, point_new);

      intensity = IntensityCompensate(corrections, raw_distance.raw_distance,
                                      intensity);
      point_new->set_intensity(intensity);
    }
    // }
  }
}

int Velodyne128Parser::IntensityCompensate(const LaserCorrection& corrections,
                                           const uint16_t raw_distance,
                                           int intensity) {
  float focal_offset = 256 * (1 - corrections.focal_distance / 13100) *
                       (1 - corrections.focal_distance / 13100);
  float focal_slope = corrections.focal_slope;

  intensity += static_cast<int>(
      focal_slope *
      static_cast<float>(std::abs(
          focal_offset -
          256.0f * (1.0f - static_cast<float>(raw_distance) / 65535.0f) *
              (1.0f - static_cast<float>(raw_distance) / 65535.0f))));

  if (intensity < corrections.min_intensity) {
    intensity = corrections.min_intensity;
  }

  if (intensity > corrections.max_intensity) {
    intensity = corrections.max_intensity;
  }
  return intensity;
}

}  // namespace velodyne
}  // namespace drivers
}  // namespace apollo
