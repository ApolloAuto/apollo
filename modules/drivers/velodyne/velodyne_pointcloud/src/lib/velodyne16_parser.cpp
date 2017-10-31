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

#include "velodyne_pointcloud/velodyne_parser.h"

#include <ros/ros.h>

namespace apollo {
namespace drivers {
namespace velodyne {

Velodyne16Parser::Velodyne16Parser(Config config)
    : VelodyneParser(config), gps_base_usec_(0), previous_packet_stamp_(0) {
  inner_time_ = &velodyne::INNER_TIME_16;
  need_two_pt_correction_ = false;
}

void Velodyne16Parser::generate_pointcloud(
    const velodyne_msgs::VelodyneScanUnified::ConstPtr& scan_msg,
    VPointCloud::Ptr& out_msg) {
  // allocate a point cloud with same time and frame ID as raw data
  out_msg->header.frame_id = scan_msg->header.frame_id;
  out_msg->height = 1;
  out_msg->header.seq = scan_msg->header.seq;
  out_msg->reserve(20000);
  gps_base_usec_ = scan_msg->basetime;

  for (size_t i = 0; i < scan_msg->packets.size(); ++i) {
    unpack(scan_msg->packets[i], *out_msg);
    last_time_stamp_ = out_msg->header.stamp;
    ROS_DEBUG_STREAM("stamp: " << std::fixed << out_msg->header.stamp);
  }

  if (out_msg->empty()) {
    // we discard this pointcloud if empty
    ROS_ERROR_STREAM(
        "All points is NAN!Please check velodyne:" << config_.model);
  }
}

double Velodyne16Parser::get_timestamp(double base_time, float time_offset,
                                       uint16_t block_id) {
  double t = base_time - time_offset;
  double timestamp = Velodyne16Parser::get_gps_stamp(t, previous_packet_stamp_,
                                                     gps_base_usec_);
  return timestamp;
}

/** @brief convert raw packet to point cloud
 *
 *  @param pkt raw packet to unpack
 *  @param pc shared pointer to point cloud (points are appended)
 */
void Velodyne16Parser::unpack(const velodyne_msgs::VelodynePacket& pkt,
                              VPointCloud& pc) {
  float azimuth = 0.0;
  float azimuth_diff = 0.0;
  float last_azimuth_diff = 0.0;
  float azimuth_corrected_f = 0.0;
  int azimuth_corrected = 0.0;

  const RawPacket* raw = (const RawPacket*)&pkt.data[0];
  double basetime = raw->gps_timestamp;  // usec

  for (int block = 0; block < BLOCKS_PER_PACKET; block++) {
    azimuth = (float)(raw->blocks[block].rotation);
    if (block < (BLOCKS_PER_PACKET - 1)) {
      azimuth_diff = (float)((36000 + raw->blocks[block + 1].rotation -
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
            azimuth + (azimuth_diff * ((dsr * VLP16_DSR_TOFFSET) +
                                       (firing * VLP16_FIRING_TOFFSET)) /
                       VLP16_BLOCK_TDURATION);
        azimuth_corrected = (int)round(fmod(azimuth_corrected_f, 36000.0));

        // set 4th param to LOWER_BANK, only use lower_gps_base_usec_ and
        // lower_previous_packet_stamp_
        double timestamp = get_timestamp(
            basetime,
            (*inner_time_)[block][firing * VLP16_SCANS_PER_FIRING + dsr],
            LOWER_BANK);

        if (block == BLOCKS_PER_PACKET - 1 &&
            firing == VLP16_FIRINGS_PER_BLOCK - 1 &&
            dsr == VLP16_SCANS_PER_FIRING - 1) {
          // set header stamp before organize the point cloud
          pc.header.stamp = static_cast<uint64_t>(timestamp * 1e6);
        }

        VPoint point;
        point.timestamp = timestamp;
        float distance = raw_distance.raw_distance * DISTANCE_RESOLUTION +
                         corrections.dist_correction;

        if (raw_distance.raw_distance == 0 ||
            !is_scan_valid(azimuth_corrected, distance)) {
          // if orgnized append a nan point to the cloud
          if (config_.organized) {
            pc.points.push_back(get_nan_point(timestamp));
            ++pc.width;
          }

          continue;
        }

        // append this point to the cloud
        compute_coords(raw_distance, corrections, azimuth_corrected, point);
        point.intensity = raw->blocks[block].data[k + 2];
        // append this point to the cloud
        pc.points.push_back(point);
        ++pc.width;

        if (block == 0 && firing == 0) {
          ROS_DEBUG_STREAM_ONCE(
              "point x:" << point.x << "  y:" << point.y << "  z:" << point.z
                         << "  intensity:" << int(point.intensity));
        }
      }
    }
  }
}

void Velodyne16Parser::order(VPointCloud::Ptr& cloud) {
  int width = 16;
  cloud->width = width;
  cloud->height = cloud->size() / cloud->width;
  int height = cloud->height;

  VPointCloud target;
  target.header = cloud->header;
  target.resize(cloud->size());
  target.width = width;
  target.height = height;

  for (int i = 0; i < width; ++i) {
    int col = velodyne::ORDER_16[i];

    for (int j = 0; j < height; ++j) {
      target.at(i, j) = cloud->at(col, j);
    }
  }
  *cloud = target;
}

}  // namespace velodyne
}  // namespace drivers
}  // namespace apollo
