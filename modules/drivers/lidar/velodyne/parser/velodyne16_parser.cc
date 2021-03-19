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

Velodyne16Parser::Velodyne16Parser(const Config& config)
    : VelodyneParser(config), previous_packet_stamp_(0), gps_base_usec_(0) {
  inner_time_ = &velodyne::INNER_TIME_16;
  need_two_pt_correction_ = false;
}

void Velodyne16Parser::GeneratePointcloud(
    const std::shared_ptr<VelodyneScan>& scan_msg,
    std::shared_ptr<PointCloud> out_msg) {
  // allocate a point cloud with same time and frame ID as raw data
  out_msg->mutable_header()->set_frame_id(scan_msg->header().frame_id());
  out_msg->set_height(1);
  out_msg->mutable_header()->set_sequence_num(
      scan_msg->header().sequence_num());
  gps_base_usec_ = scan_msg->basetime();

  size_t packets_size = scan_msg->firing_pkts_size();
  for (size_t i = 0; i < packets_size; ++i) {
    Unpack(scan_msg->firing_pkts(static_cast<int>(i)), out_msg);
    last_time_stamp_ = out_msg->measurement_time();
    ADEBUG << "stamp: " << std::fixed << last_time_stamp_;
  }

  if (out_msg->point().empty()) {
    // we discard this pointcloud if empty
    AERROR << "All points is NAN!Please check velodyne:" << config_.model();
  }

  // set default width
  out_msg->set_width(out_msg->point_size());
}

uint64_t Velodyne16Parser::GetTimestamp(double base_time, float time_offset,
                                        uint16_t block_id) {
  double t = base_time - time_offset;
  uint64_t timestamp = Velodyne16Parser::GetGpsStamp(t, &previous_packet_stamp_,
                                                     &gps_base_usec_);
  return timestamp;
}

/** @brief convert raw packet to point cloud
 *
 *  @param pkt raw packet to Unpack
 *  @param pc shared pointer to point cloud (points are appended)
 */
void Velodyne16Parser::Unpack(const VelodynePacket& pkt,
                              std::shared_ptr<PointCloud> pc) {
  float azimuth_diff = 0.0f;
  float last_azimuth_diff = 0.0f;
  float azimuth_corrected_f = 0.0f;
  int azimuth_corrected = 0;

  // const RawPacket* raw = (const RawPacket*)&pkt.data[0];
  const RawPacket* raw = (const RawPacket*)pkt.data().c_str();
  double basetime = raw->gps_timestamp;  // usec

  for (int block = 0; block < BLOCKS_PER_PACKET; block++) {
    float azimuth = static_cast<float>(raw->blocks[block].rotation);
    if (block < (BLOCKS_PER_PACKET - 1)) {
      azimuth_diff =
          static_cast<float>((36000 + raw->blocks[block + 1].rotation -
                              raw->blocks[block].rotation) %
                             36000);
      last_azimuth_diff = azimuth_diff;
    } else {
      azimuth_diff = last_azimuth_diff;
    }

    for (int firing = 0, k = 0; firing < VLP16_FIRINGS_PER_BLOCK; ++firing) {
      for (int dsr = 0; dsr < VLP16_SCANS_PER_FIRING;
           ++dsr, k += RAW_SCAN_SIZE) {
        LaserCorrection& corrections = calibration_.laser_corrections_[dsr];

        /** Position Calculation */
        union RawDistance raw_distance;
        raw_distance.bytes[0] = raw->blocks[block].data[k];
        raw_distance.bytes[1] = raw->blocks[block].data[k + 1];

        /** correct for the laser rotation as a function of timing during the
         * firings **/
        azimuth_corrected_f =
            azimuth + (azimuth_diff *
                       ((static_cast<float>(dsr) * VLP16_DSR_TOFFSET) +
                        (static_cast<float>(firing) * VLP16_FIRING_TOFFSET)) /
                       VLP16_BLOCK_TDURATION);
        azimuth_corrected =
            static_cast<int>(round(fmod(azimuth_corrected_f, 36000.0)));

        // set 4th param to LOWER_BANK, only use lower_gps_base_usec_ and
        // lower_previous_packet_stamp_
        uint64_t timestamp = GetTimestamp(
            basetime,
            (*inner_time_)[block][firing * VLP16_SCANS_PER_FIRING + dsr],
            LOWER_BANK);

        if (block == BLOCKS_PER_PACKET - 1 &&
            firing == VLP16_FIRINGS_PER_BLOCK - 1 &&
            dsr == VLP16_SCANS_PER_FIRING - 1) {
          // set header stamp before organize the point cloud
          pc->set_measurement_time(static_cast<double>(timestamp) / 1e9);
        }

        float real_distance = raw_distance.raw_distance * DISTANCE_RESOLUTION;
        float distance = real_distance + corrections.dist_correction;

        if (raw_distance.raw_distance == 0 ||
            !is_scan_valid(azimuth_corrected, distance)) {
          // if organized append a nan point to the cloud
          if (config_.organized()) {
            PointXYZIT* point = pc->add_point();
            point->set_x(nan);
            point->set_y(nan);
            point->set_z(nan);
            point->set_timestamp(timestamp);
            point->set_intensity(0);
          }

          continue;
        }
        PointXYZIT* point = pc->add_point();
        point->set_timestamp(timestamp);
        // append this point to the cloud
        ComputeCoords(real_distance, corrections,
                      static_cast<uint16_t>(azimuth_corrected), point);
        point->set_intensity(raw->blocks[block].data[k + 2]);
        // append this point to the cloud

        if (block == 0 && firing == 0) {
          ADEBUG << "point x:" << point->x() << "  y:" << point->y()
                 << "  z:" << point->z()
                 << "  intensity:" << int(point->intensity());
        }
      }
    }
  }
}

void Velodyne16Parser::Order(std::shared_ptr<PointCloud> cloud) {
  int width = 16;
  cloud->set_width(width);
  int height = cloud->point_size() / cloud->width();
  cloud->set_height(height);

  std::shared_ptr<PointCloud> cloud_origin = std::make_shared<PointCloud>();
  cloud_origin->CopyFrom(*cloud);

  for (int i = 0; i < width; ++i) {
    int col = velodyne::ORDER_16[i];

    for (int j = 0; j < height; ++j) {
      // make sure offset is initialized, should be init at setup() just once
      int target_index = j * width + i;
      int origin_index = j * width + col;
      cloud->mutable_point(target_index)
          ->CopyFrom(cloud_origin->point(origin_index));
    }
  }
}

}  // namespace velodyne
}  // namespace drivers
}  // namespace apollo
