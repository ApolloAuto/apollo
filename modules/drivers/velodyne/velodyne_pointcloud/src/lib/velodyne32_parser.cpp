/******************************************************************************
 * Copyright 2018 The Apollo Authors. All Rights Reserved.
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

#include <pcl/common/time.h>
#include <ros/package.h>
#include <ros/ros.h>

namespace apollo {
namespace drivers {
namespace velodyne {

Velodyne32Parser::Velodyne32Parser(Config config)
    : VelodyneParser(config), gps_base_usec_(0), previous_packet_stamp_(0) {
  inner_time_ = &velodyne::INNER_TIME_HDL32E;
  need_two_pt_correction_ = false;
}

void Velodyne32Parser::generate_pointcloud(
    velodyne_msgs::VelodyneScanUnified::ConstPtr scan_msg,
    VPointCloud::Ptr& out_msg) {
  // allocate a point cloud with same time and frame ID as raw data
  out_msg->header.frame_id = scan_msg->header.frame_id;
  out_msg->height = 1;
  out_msg->header.seq = scan_msg->header.seq;
  out_msg->reserve(80000);
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

double Velodyne32Parser::get_timestamp(double base_time, float time_offset,
                                       uint16_t block_id) {
  double t = base_time - time_offset;
  double timestamp = get_gps_stamp(t, previous_packet_stamp_, gps_base_usec_);
  return timestamp;
}

void Velodyne32Parser::unpack(const velodyne_msgs::VelodynePacket& pkt,
                              VPointCloud& pc) {
  const RawPacket* raw = (const RawPacket*)&pkt.data[0];
  double basetime = raw->gps_timestamp;  // usec

  for (int i = 0; i < BLOCKS_PER_PACKET; i++) {  // 12
    for (int laser_id = 0, k = 0; laser_id < SCANS_PER_BLOCK;
         ++laser_id, k += RAW_SCAN_SIZE) {  // 32, 3
      LaserCorrection& corrections = calibration_.laser_corrections_[laser_id];

      union RawDistance raw_distance;
      raw_distance.bytes[0] = raw->blocks[i].data[k];
      raw_distance.bytes[1] = raw->blocks[i].data[k + 1];

      // compute time
      double timestamp =
          get_timestamp(basetime, (*inner_time_)[i][laser_id], i);

      if (laser_id == SCANS_PER_BLOCK - 1) {
        // set header stamp before organize the point cloud
        pc.header.stamp = static_cast<uint64_t>(timestamp * 1e6);
      }

      int rotation = (int)raw->blocks[i].rotation;
      float distance = raw_distance.raw_distance * DISTANCE_RESOLUTION +
                       corrections.dist_correction;

      if (raw_distance.raw_distance == 0 ||
          !is_scan_valid(rotation, distance)) {
        // if organized append a nan point to the cloud
        if (config_.organized) {
          pc.points.emplace_back(get_nan_point(timestamp));
          ++pc.width;
        }
        continue;
      }

      VPoint point;
      point.timestamp = timestamp;
      // Position Calculation, append this point to the cloud
      compute_coords(raw_distance, corrections, rotation, point);
      point.intensity = raw->blocks[i].data[k + 2];
      // append this point to the cloud
      pc.points.emplace_back(point);
      ++pc.width;
    }
  }
}

void Velodyne32Parser::order(VPointCloud::Ptr& cloud) {
  int width = 32;
  cloud->width = width;
  cloud->height = cloud->size() / cloud->width;
  int height = cloud->height;

  VPointCloud target;
  target.header = cloud->header;
  target.resize(cloud->size());
  target.width = width;
  target.height = height;

  for (int i = 0; i < width; ++i) {
    int col = ORDER_HDL32E[i];

    for (int j = 0; j < height; ++j) {
      target.at(i, j) = cloud->at(col, j);
    }
  }
  *cloud = target;
}

}  // namespace velodyne
}  // namespace drivers
}  // namespace apollo
