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

#include "modules/drivers/velodyne/parser/velodyne_parser.h"

namespace apollo {
namespace drivers {
namespace velodyne {

// std::string toBinary(int n) {
//   std::string r;
//   while (n != 0) {
//     r = (n % 2 == 0 ? "0" : "1") + r;
//     n /= 2;
//   }
//   while (r.length() != 8) {
//     r = '0' + r;
//   }
//   return r;
// }
//
// double convertBinaryToDecimal(std::string binaryString) {
//   double value = 0;
//   int indexCounter = 0;
//   for (int i = binaryString.length() - 1; i >= 0; i--) {
//     if (binaryString[i] == '1') {
//       value += pow(2, indexCounter);
//     }
//     indexCounter++;
//   }
//   return value;
// }
//
// double computeTimeStamp(const adu::common::sensor::VelodynePacket& pkt) {
//   const uint8_t *pkt_data = (const uint8_t*)pkt.data().data();
//   std::string digit4 = toBinary(pkt_data[1203]);
//   std::string digit3 = toBinary(pkt_data[1202]);
//   std::string digit2 = toBinary(pkt_data[1201]);
//   std::string digit1 = toBinary(pkt_data[1200]);
//   std::string digit =
//       digit4 + digit3 + digit2 + digit1;  // string concatenation
//   double value = convertBinaryToDecimal(digit);
//   // compute the seconds from the beginning of that hour to when the data being
//   // captured
//   double time_stamp = (double)value / 1000000;
//   return time_stamp;
// }

Velodyne128Parser::Velodyne128Parser(Config& config)
    : VelodyneParser(config), previous_packet_stamp_(0), gps_base_usec_(0) {
  //TODO: wait for lidar128 manual
  inner_time_ = &velodyne::INNER_TIME_HDL32E;
  need_two_pt_correction_ = false;
}

void Velodyne128Parser::generate_pointcloud(
    const std::shared_ptr<VelodyneScan>& scan_msg,
    std::shared_ptr<PointCloud>& out_msg) {

  // allocate a point cloud with same time and frame ID as raw data
  out_msg->mutable_header()->set_frame_id(scan_msg->header().frame_id());      
  out_msg->mutable_header()->set_timestamp_sec(cybertron::Time().Now().ToSecond());      
  out_msg->set_height(1);

  //us
  gps_base_usec_ = scan_msg->basetime();

  for (int i = 0; i < scan_msg->firing_pkts_size(); ++i) {
    unpack(scan_msg->firing_pkts(i), out_msg);
    last_time_stamp_ = out_msg->measurement_time();      
  }

  size_t size = out_msg->point_size();
  if (size == 0) {
    // we discard this pointcloud if empty
    AERROR << "All points is NAN!Please check velodyne:" << config_.model();
    return;
  } else {
    uint64_t timestamp = out_msg->point(size - 1).timestamp();
    out_msg->set_measurement_time(timestamp / 1e9);
    out_msg->mutable_header()->set_lidar_timestamp(timestamp);
  }
  out_msg->set_width(out_msg->point_size());
}

uint64_t Velodyne128Parser::get_timestamp(double base_time, float time_offset,
                                         uint16_t block_id) {
  (void)block_id;
  double t = base_time - time_offset;
  uint64_t timestamp = get_gps_stamp(t, previous_packet_stamp_, gps_base_usec_);
  return timestamp;
}

//TODO: No manual about order for lidar128 by now.
void Velodyne128Parser::order(
    std::shared_ptr<PointCloud>& cloud) {
  (void)cloud;
}

void Velodyne128Parser::unpack(
    const VelodynePacket& pkt,
    std::shared_ptr<PointCloud>& pc) {
  float azimuth_diff, azimuth_corrected_f;
  float last_azimuth_diff = 0;
  uint16_t azimuth, azimuth_next, azimuth_corrected;
  // float x_coord, y_coord, z_coord;
  float distance;
  int intensity;

  // const raw_packet_t *raw = (const raw_packet_t *)&pkt.data[0];
  const RawPacket *raw = (const RawPacket *)pkt.data().c_str();
  double basetime = raw->gps_timestamp;
  // double basetime_2 = computeTimeStamp(pkt);
  // LOG_INFO << "basetime1: " << basetime / 1000000.0  << ", basetime2: " << basetime_2;
  //
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
    // if ((config_.min_angle < config_.max_angle &&
    //      azimuth >= config_.min_angle && azimuth <= config_.max_angle) ||
    //     (config_.min_angle > config_.max_angle)) {
    for (int j = 0, k = 0; j < SCANS_PER_BLOCK; j++, k += RAW_SCAN_SIZE) {
      uint8_t group = block % 4;
      uint8_t chan_id = j + group * 32;
      uint8_t firing_order = chan_id / 8;
      firing_order = 0;
      LaserCorrection& corrections = calibration_.laser_corrections_[chan_id];
      // distance extraction
      // union two_bytes tmp;
      union RawDistance raw_distance;
      raw_distance.bytes[0] = raw->blocks[block].data[k];
      raw_distance.bytes[1] = raw->blocks[block].data[k + 1];
      distance = raw_distance.raw_distance * VSL128_DISTANCE_RESOLUTION;

      uint64_t timestamp = get_timestamp(basetime, 0, block);
      if (!is_scan_valid(azimuth, distance)) {
        //todo orgnized
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
      intensity = static_cast<float>(raw->blocks[block].data[k + 2]);

      /** correct for the laser rotation as a function of timing during the
       * firings **/
      azimuth_corrected_f =
          azimuth +
          (azimuth_diff * (firing_order * CHANNEL_TDURATION) / SEQ_TDURATION);
      azimuth_corrected =
          (static_cast<uint16_t>(round(azimuth_corrected_f))) % 36000;

      //add new point
      PointXYZIT* point_new = pc->add_point(); 

      //compute time , time offset is zero
      point_new->set_timestamp(timestamp);

      // LOG_INFO << "intensity: " << intensity << ", azimuth: " << azimuth
      //          << ", _gps_base_usec: " << _gps_base_usec
      //          << ", basetime:"  << basetime << ",timestamp: " << timestamp
      //          << ", distance: " << distance;
      //
      // apply calibration file and convert polar coordinates to Euclidean
      // XYZ
      // compute_xyzi(chan_id, azimuth_corrected, distance, &intensity,
      // &x_coord,
      //              &y_coord, &z_coord);
      compute_coords(distance, corrections, azimuth_corrected, point_new);

      intensity = intensity_compensate(corrections, raw_distance.raw_distance, intensity);
      point_new->set_intensity(intensity);
    }
    // }
  }
}

int Velodyne128Parser::intensity_compensate(const LaserCorrection& corrections,
                                            const uint16_t raw_distance,
                                            int intensity) {
  float focal_offset = 256 * (1 - corrections.focal_distance / 13100) *
                       (1 - corrections.focal_distance / 13100);
  float focal_slope = corrections.focal_slope;

  intensity += focal_slope * (abs(focal_offset -
                         256 * (1 - static_cast<float>(raw_distance) / 65535) *
                             (1 - static_cast<float>(raw_distance) / 65535)));

  if (intensity < corrections.min_intensity) {
    intensity = corrections.min_intensity;
  }

  if (intensity > corrections.max_intensity) {
     intensity = corrections.max_intensity;
  }
  return intensity;
}

}  // namespace velodyne_data
}
}
