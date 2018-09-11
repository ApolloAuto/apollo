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

// #include <ros/ros.h>

namespace apollo {
namespace drivers {
namespace velodyne {

Velodyne128Parser::Velodyne128Parser(Config config) : VelodyneParser(config) {
  for (int i = 0; i < 4; i++) {
    gps_base_usec_[i] = 0;
    previous_packet_stamp_[i] = 0;
  }
  need_two_pt_correction_ = true;
  // init unpack function and order function by model.
  if (config_.model == "64E_S2") {
    inner_time_ = &velodyne::INNER_TIME_64;
    is_s2_ = true;
  } else {  // 64E_S3
    inner_time_ = &velodyne::INNER_TIME_64E_S3;
    is_s2_ = false;
  }

  if (config_.model == "64E_S3D_LAST") {
    mode_ = LAST;
  } else if (config_.model == "64E_S3D_DUAL") {
    mode_ = DUAL;
  }
}

void Velodyne128Parser::setup() {
  VelodyneParser::setup();
  // if (!config_.calibration_online && config_.organized) {
  //   init_offsets();
  // }
}

void Velodyne128Parser::set_base_time_from_packets(const VelodynePacket& pkt) {
  // const RawPacket* raw = (const RawPacket*)&pkt.data[0];
  const RawPacket* raw = (const RawPacket*)pkt.data().c_str();
  StatusType status_type = StatusType(raw->status_type);
  char status_value = raw->status_value;

  static int year = -1, month = -1, day = -1, hour = -1, minute = -1,
             second = -1;
  static int gps_status = 0;
  static tm time;

  switch (status_type) {
    case YEAR:
      year = status_value + 2000;
      break;
    case MONTH:
      month = status_value;
      break;
    case DATE:
      day = status_value;
      break;
    case HOURS:
      hour = status_value;
      break;
    case MINUTES:
      minute = status_value;
      break;
    case SECONDS:
      second = status_value;
      break;
    case GPS_STATUS:
      gps_status = status_value;
      break;
    default:
      break;
  }

  LOG_INFO_FORMAT("Get base time from packets. Obtained (%d.%d.%d %d:%d:%d)",
                  year, month, day, hour, minute, second);

  if (status_type == GPS_STATUS && year > 0 && month > 0 && day > 0 &&
      hour >= 0 && minute >= 0 && second >= 0) {
    if (gps_status != 65) {
      AWARN << "Sync failed because Velodyne-GPS Sync is NOT good! "
                      << "Status: " << (int)gps_status
                      << " (65 = both; 86 = gps only; 80 = PPS only; 0 "
                      << "= GPS not connected)";
    }

    time.tm_year = year - 1900;
    time.tm_mon = month - 1;
    time.tm_mday = day;
    time.tm_hour = hour;
    time.tm_min = 0;
    time.tm_sec = 0;

    AINFO << "Set base unix time: (%d.%d.%d %d:%d:%d)", time.tm_year,
        time.tm_mon, time.tm_mday, time.tm_hour, time.tm_min, time.tm_sec;

    uint64_t unix_base = static_cast<uint64_t>(timegm(&time));
    for (int i = 0; i < 4; ++i) {
      gps_base_usec_[i] = unix_base * 1000000;
    }
  }
}

void Velodyne128Parser::check_gps_status(const VelodynePacket& pkt) {
  // const RawPacket* raw = (const RawPacket*)&pkt.data[0];
  const RawPacket* raw = (const RawPacket*)pkt.data().c_str();
  StatusType status_type = StatusType(raw->status_type);
  char status_value = raw->status_value;

  if (status_type == StatusType::GPS_STATUS) {
    if (status_value != 65) {
      AWARN << "Sync failed because Velodyne-GPS Sync is NOT good! "
            << "Status: " << (int)status_value
            << " (65 = both; 86 = gps only; 80 = PPS only; "
            << "0 = GPS not connected)";
    }
  }
}

// void Velodyne128Parser::init_offsets() {
//   int width = 64;
//   // pre compute col offsets
//   for (int i = 0; i < width; ++i) {
//     int col = velodyne::ORDER_64[i];
//     // compute offset, NOTICE: std::map doesn't have const [] since [] may
//     // insert new values into map
//     const LaserCorrection& corrections = calibration_.laser_corrections_[col];
//     int offset = int(corrections.rot_correction / ANGULAR_RESOLUTION + 0.5);
//     offsets_[i] = offset;
//   }
// }

void Velodyne128Parser::generate_pointcloud(
    const std::shared_ptr<VelodyneScanUnified>& scan_msg,
    VPointCloud::Ptr& pointcloud) {
  // if (config_.calibration_online && !calibration_.initialized_) {
  //   if (online_calibration_.decode(scan_msg) == -1) {
  //     return;
  //   }
  //   calibration_ = online_calibration_.calibration();
  //   if (config_.organized) {
  //     init_offsets();
  //   }
  // }
  //

  // allocate a point cloud with same time and frame ID as raw data
  pointcloud->header.frame_id = scan_msg->header().frame_id();
  pointcloud->height = 1;
  pointcloud->header.seq = scan_msg->header().seq();

  pointcloud->reserve(140000);

  bool skip = false;
  size_t packets_size = scan_msg->packets_size();
  for (size_t i = 0; i < packets_size; ++i) {
    unpack(scan_msg->packets(i), *pointcloud);
    // if (gps_base_usec_[0] == 0) {
    //   // only set one time type when call this function, so cannot break
    //   set_base_time_from_packets(scan_msg->packets(i));
    //   // If base time not ready then set empty_unpack true
    //   skip = true;
    // } else {
    //   check_gps_status(scan_msg->packets(i));
    //   unpack(scan_msg->packets(i), *pointcloud);
    //   last_time_stamp_ = pointcloud->header.stamp;
    //   ADEBUG << "stamp: " << std::fixed << pointcloud->header.stamp;
    // }
  }

  if (skip) {
    pointcloud->clear();
  } else {
    if (pointcloud->empty()) {
      // we discard this pointcloud if empty
      AERROR << "All points is NAN! Please check velodyne:" << config_.model;
    }
    pointcloud->width = pointcloud->size();
  }
}

double Velodyne128Parser::get_timestamp(double base_time, float time_offset,
                                       uint16_t block_id) {
  double t = base_time - time_offset;
  double timestamp = 0;
  int index = 0;

  if (is_s2_) {
    index = block_id & 1;  // % 2
    double& previous_packet_stamp = previous_packet_stamp_[index];
    uint64_t& gps_base_usec = gps_base_usec_[index];
    timestamp = get_gps_stamp(t, previous_packet_stamp, gps_base_usec);
  } else {                 // 64E_S3
    index = block_id & 3;  // % 4
    double& previous_packet_stamp = previous_packet_stamp_[index];
    uint64_t& gps_base_usec = gps_base_usec_[index];
    timestamp = get_gps_stamp(t, previous_packet_stamp, gps_base_usec);
  }
  return timestamp;
}

int Velodyne128Parser::intensity_compensate(const LaserCorrection& corrections,
                                           const uint16_t raw_distance,
                                           int intensity) {
  float tmp = 1 - static_cast<float>(raw_distance) / 65535;
  intensity += corrections.focal_slope *
               (fabs(corrections.focal_offset - 256 * tmp * tmp));

  if (intensity < corrections.min_intensity) {
    intensity = corrections.min_intensity;
  }

  if (intensity > corrections.max_intensity) {
    intensity = corrections.max_intensity;
  }
  return intensity;
}

// void Velodyne128Parser::unpack(const VelodynePacket& pkt, VPointCloud& pc) {
//   ADEBUG << "Received packet, time: " << pkt.stamp();
//
//   // const RawPacket* raw = (const RawPacket*)&pkt.data[0];
//   const RawPacket* raw = (const RawPacket*)pkt.data().c_str();
//   double basetime = raw->gps_timestamp;  // usec
//
//   for (int i = 0; i < BLOCKS_PER_PACKET; ++i) {  // 12
//     if (mode_ != DUAL && !is_s2_ && ((i & 3) >> 1) > 0) {
//       // i%4/2  even-numbered block contain duplicate data
//       continue;
//     }
//
//     // upper bank lasers are numbered [0..31], lower bank lasers are [32..63]
//     // NOTE: this is a change from the old velodyne_common implementation
//     int bank_origin = (raw->blocks[i].laser_block_id == LOWER_BANK) ? 32 : 0;
//
//     for (int j = 0, k = 0; j < SCANS_PER_BLOCK;
//          ++j, k += RAW_SCAN_SIZE) {  // 32, 3
//       // One point
//       uint8_t laser_number = j + bank_origin;  // hardware laser number
//       LaserCorrection& corrections =
//           calibration_.laser_corrections_[laser_number];
//
//       union RawDistance raw_distance;
//       raw_distance.bytes[0] = raw->blocks[i].data[k];
//       raw_distance.bytes[1] = raw->blocks[i].data[k + 1];
//
//       // compute time
//       double timestamp = get_timestamp(basetime, (*inner_time_)[i][j], i);
//
//       if (j == SCANS_PER_BLOCK - 1) {
//         // set header stamp before organize the point cloud
//         pc.header.stamp = static_cast<uint64_t>(timestamp * 1000000);
//       }
//
//       float distance = raw_distance.raw_distance * DISTANCE_RESOLUTION +
//                        corrections.dist_correction;
//
//       if (raw_distance.raw_distance == 0 ||
//           !is_scan_valid(raw->blocks[i].rotation, distance)) {
//         // if organized append a nan point to the cloud
//         if (config_.organized) {
//           pc.points.emplace_back(get_nan_point(timestamp));
//         }
//         continue;
//       }
//
//       VPoint point;
//       point.timestamp = timestamp;
//       // Position Calculation, append this point to the cloud
//       compute_coords(raw_distance, corrections, raw->blocks[i].rotation, point);
//       point.intensity = intensity_compensate(
//           corrections, raw_distance.raw_distance, raw->blocks[i].data[k + 2]);
//       // append this point to the cloud
//       pc.points.emplace_back(point);
//     }
//   }
// }

// void Velodyne128Parser::order(VPointCloud::Ptr& cloud) {
//   int width = 64;
//   cloud->width = width;
//   cloud->height = cloud->size() / cloud->width;
//   int height = cloud->height;
//
//   VPointCloud target;
//   target.header = cloud->header;
//   target.resize(cloud->size());
//   target.width = width;
//   target.height = height;
//
//   for (int i = 0; i < width; ++i) {
//     int col = velodyne::ORDER_64[i];
//
//     for (int j = 0; j < height; ++j) {
//       // make sure offset is initialized, should be init at setup() just once
//       int row = (j + offsets_[i] + height) % height;
//       target.at(i, j) = cloud->at(col, row);
//     }
//   }
//   *cloud = target;
// }

void RawData::unpack(const VelodynePacket& pkt, VPointCloud& pc) {
  float azimuth_diff, azimuth_corrected_f;
  float last_azimuth_diff = 0;
  uint16_t azimuth, azimuth_next, azimuth_corrected;
  float x_coord, y_coord, z_coord;
  float distance, intensity;

  // typedef struct vls128_raw_block {
  //   uint16_t header;    ///< UPPER_BANK or LOWER_BANK
  //   uint16_t rotation;  ///< 0-35999, divide by 100 to get degrees
  //   uint8_t data[VLS128_BLOCK_DATA_SIZE];
  // } vls128_raw_block_t;
  // typedef struct vls128_raw_packet {
  //   vls128_raw_block_t blocks[3];
  //   uint16_t revolution;
  //   uint8_t status[PACKET_STATUS_SIZE];
  // } vls128_raw_packet_t;

  // const raw_packet_t *raw = (const raw_packet_t *)&pkt.data[0];
  const RawPacket* raw = (const RawPacket *) pkt.data().c_str();

  for (int block = 0; block < BLOCKS_PER_PACKET; block++) {
    // Calculate difference between current and next block's azimuth angle.
    if (block == 0) {
      azimuth = raw->blocks[block].rotation;
    } else {
      azimuth = azimuth_next;
    }
    if (block < (NUM_BLOCKS_PER_PACKET - 1)) {
      azimuth_next = raw->blocks[block + 1].rotation;
      azimuth_diff = (float)((36000 + azimuth_next - azimuth) % 36000);
      last_azimuth_diff = azimuth_diff;
    } else {
      azimuth_diff = last_azimuth_diff;
    }

    /*condition added to avoid calculating points which are not
      in the interesting defined area (min_angle < area < max_angle)*/
    if ((config_.min_angle < config_.max_angle &&
         azimuth >= config_.min_angle && azimuth <= config_.max_angle) ||
        (config_.min_angle > config_.max_angle)) {
      // for (int j = 0, k = 0; j < NUM_CHANS_PER_BLOCK; j++, k += CHANNEL_SIZE) {
      for (int j = 0, k = 0; j < SCANS_PER_BLOCK; j++, k += RAW_SCAN_SIZE) {
        uint8_t group = block % 4;
        uint8_t chan_id = j + group * 32;
        uint8_t firing_order = chan_id / 8;
        firing_order = 0;

        velodyne_pointcloud::LaserCorrection &corrections =
            calibration_.laser_corrections[chan_id];

        // distance extraction
        union RawDistance 
        tmp.bytes[0] = raw->blocks[block].data[k];
        tmp.bytes[1] = raw->blocks[block].data[k + 1];
        distance = (float)tmp.raw_distance * VLP32_DISTANCE_RESOLUTION;
        distance += corrections.dist_correction;

        if (!s_scan_valid(azimuth, distance)) {
          //todo organized
          continue;
        }

        intensity = (float)raw->blocks[block].data[k + 2];

        /** correct for the laser rotation as a function of timing during the
         * firings **/
        azimuth_corrected_f = azimuth +
            (azimuth_diff * (firing_order * VLS128_CHANNEL_TDURATION) / VLS128_SEQ_TDURATION);
        azimuth_corrected = ((uint16_t)round(azimuth_corrected_f)) % 36000;

        // apply calibration file and convert polar coordinates to Euclidean
        // XYZ
        compute_xyzi(chan_id, azimuth_corrected, distance, intensity, x_coord,
                     y_coord, z_coord);

        // append this point to the cloud
        VPoint point;
        point.ring = corrections.laser_ring;
        point.x = x_coord;
        point.y = y_coord;
        point.z = z_coord;
        point.intensity = intensity;

        pc.points.push_back(point);
        ++pc.width;
      }
    }
  }
}

void RawData::compute_xyzi(const uint8_t chan_id, const uint16_t azimuth_uint,
                           const float distance, float& intensity,
                           float& x_coord, float& y_coord, float& z_coord) {
  float x, y, z;
  velodyne_pointcloud::LaserCorrection& corrections =
      calibration_.laser_corrections[chan_id];

  // convert polar coordinates to Euclidean XYZ
  float cos_vert_angle = corrections.cos_vert_correction;
  float sin_vert_angle = corrections.sin_vert_correction;
  float cos_rot_correction = corrections.cos_rot_correction;
  float sin_rot_correction = corrections.sin_rot_correction;

  // cos(a-b) = cos(a)*cos(b) + sin(a)*sin(b)
  // sin(a-b) = sin(a)*cos(b) - cos(a)*sin(b)
  float cos_rot_angle = cos_rot_table_[azimuth_uint] * cos_rot_correction +
                        sin_rot_table_[azimuth_uint] * sin_rot_correction;
  float sin_rot_angle = sin_rot_table_[azimuth_uint] * cos_rot_correction -
                        cos_rot_table_[azimuth_uint] * sin_rot_correction;

  float horiz_offset = corrections.horiz_offset_correction;
  float vert_offset = corrections.vert_offset_correction;

  // Compute the distance in the xy plane (w/o accounting for rotation)
  /**the new term of 'vert_offset * sin_vert_angle'
   * was added to the expression due to the mathemathical
   * model we used.
   */
  float xy_distance = distance * cos_vert_angle - vert_offset * sin_vert_angle;

  // Calculate temporal X, use absolute value.
  float xx = xy_distance * sin_rot_angle - horiz_offset * cos_rot_angle;
  // Calculate temporal Y, use absolute value
  float yy = xy_distance * cos_rot_angle + horiz_offset * sin_rot_angle;
  if (xx < 0) xx = -xx;
  if (yy < 0) yy = -yy;

  // Get 2points calibration values,Linear interpolation to get distance
  // correction for X and Y, that means distance correction use
  // different value at different distance
  float distance_corr_x = 0;
  float distance_corr_y = 0;
  if (need_two_pt_correction_) {
    distance_corr_x =
        (corrections.dist_correction - corrections.dist_correction_x) *
            (xx - 2.4) / (25.04 - 2.4) +
        corrections.dist_correction_x;
    distance_corr_x -= corrections.dist_correction;
    distance_corr_y =
        (corrections.dist_correction - corrections.dist_correction_y) *
            (yy - 1.93) / (25.04 - 1.93) +
        corrections.dist_correction_y;
    distance_corr_y -= corrections.dist_correction;
  }

  float distance_x = distance + distance_corr_x;
  /**the new term of 'vert_offset * sin_vert_angle'
   * was added to the expression due to the mathemathical
   * model we used.
   */
  xy_distance = distance_x * cos_vert_angle - vert_offset * sin_vert_angle;
  x = xy_distance * sin_rot_angle - horiz_offset * cos_rot_angle;

  float distance_y = distance + distance_corr_y;
  /**the new term of 'vert_offset * sin_vert_angle'
   * was added to the expression due to the mathemathical
   * model we used.
   */
  xy_distance = distance_y * cos_vert_angle - vert_offset * sin_vert_angle;
  y = xy_distance * cos_rot_angle + horiz_offset * sin_rot_angle;

  // Using distance_y is not symmetric, but the velodyne manual
  // does this.
  /**the new term of 'vert_offset * cos_vert_angle'
   * was added to the expression due to the mathemathical
   * model we used.
   */
  z = distance_y * sin_vert_angle + vert_offset * cos_vert_angle;

  /** Use standard ROS coordinate system (right-hand rule) */
  x_coord = y;
  y_coord = -x;
  z_coord = z;

  /** Intensity Calculation */
  // float min_intensity = corrections.min_intensity;
  // float max_intensity = corrections.max_intensity;
  float min_intensity = 

  float focal_offset = 256 * (1 - corrections.focal_distance / 13100) *
                       (1 - corrections.focal_distance / 13100);
  float focal_slope = corrections.focal_slope;
  intensity +=
      focal_slope * (abs(focal_offset -
                         256 * (1 - static_cast<float>(azimuth_uint) / 65535) *
                             (1 - static_cast<float>(azimuth_uint) / 65535)));
  intensity = (intensity < min_intensity) ? min_intensity : intensity;
  intensity = (intensity > max_intensity) ? max_intensity : intensity;
}

}  // namespace velodyne
}  // namespace drivers
}  // namespace apollo
