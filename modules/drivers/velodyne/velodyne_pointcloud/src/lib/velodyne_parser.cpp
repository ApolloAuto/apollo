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

#include <pcl/common/time.h>
#include <ros/package.h>
#include <ros/ros.h>

#include "velodyne_pointcloud/util.h"

namespace apollo {
namespace drivers {
namespace velodyne {

double VelodyneParser::get_gps_stamp(double current_packet_stamp,
                                     double &previous_packet_stamp,
                                     uint64_t &gps_base_usec) {
  if (current_packet_stamp < previous_packet_stamp) {
    // plus 3600 when large jump back, discard little jump back for wrong time
    // in lidar
    if (std::abs(previous_packet_stamp - current_packet_stamp) > 3599000000) {
      gps_base_usec += 3600 * 1e6;
      ROS_INFO_STREAM("Base time plus 3600s. Model: "
                      << config_.model << std::fixed
                      << ". current:" << current_packet_stamp
                      << ", last time:" << previous_packet_stamp);
    } else {
      ROS_WARN_STREAM("Currrnt stamp:" << std::fixed << current_packet_stamp
                                       << " less than previous stamp:"
                                       << previous_packet_stamp
                                       << ". GPS time stamp maybe incorrect!");
    }
  } else if (previous_packet_stamp != 0 &&
             current_packet_stamp - previous_packet_stamp > 100000) {  // 100ms
    ROS_ERROR_STREAM("Current stamp:"
                     << std::fixed << current_packet_stamp
                     << " ahead previous stamp:" << previous_packet_stamp
                     << " over 100ms. GPS time stamp incorrect!");
  }

  previous_packet_stamp = current_packet_stamp;
  double gps_stamp = gps_base_usec + current_packet_stamp;

  gps_stamp /= 1e6;
  return gps_stamp;
}

VPoint VelodyneParser::get_nan_point(double timestamp) {
  VPoint nan_point;
  nan_point.timestamp = timestamp;
  nan_point.x = nan;
  nan_point.y = nan;
  nan_point.z = nan;
  nan_point.intensity = 0;
  return nan_point;
}

VelodyneParser::VelodyneParser(Config config)
    : last_time_stamp_(0), config_(config), mode_(STRONGEST) {}

void VelodyneParser::init_angle_params(double view_direction,
                                       double view_width) {
  // converting angle parameters into the velodyne reference (rad)
  double tmp_min_angle = view_direction + view_width / 2;
  double tmp_max_angle = view_direction - view_width / 2;

  // computing positive modulo to keep theses angles into [0;2*M_PI]
  tmp_min_angle = fmod(fmod(tmp_min_angle, 2 * M_PI) + 2 * M_PI, 2 * M_PI);
  tmp_max_angle = fmod(fmod(tmp_max_angle, 2 * M_PI) + 2 * M_PI, 2 * M_PI);

  // converting into the hardware velodyne ref (negative yaml and degrees)
  // adding 0.5 perfomrs a centered double to int conversion
  config_.min_angle = 100 * (2 * M_PI - tmp_min_angle) * 180 / M_PI + 0.5;
  config_.max_angle = 100 * (2 * M_PI - tmp_max_angle) * 180 / M_PI + 0.5;
  if (config_.min_angle == config_.max_angle) {
    // avoid returning empty cloud if min_angle = max_angle
    config_.min_angle = 0;
    config_.max_angle = 36000;
  }
}

/** Set up for on-line operation. */
void VelodyneParser::setup() {
  if (!config_.calibration_online) {
    calibration_.read(config_.calibration_file);

    if (!calibration_.initialized_) {
      ROS_FATAL_STREAM(
          "Unable to open calibration file: " << config_.calibration_file);
      ROS_BREAK();
    }
  }

  // setup angle parameters.
  init_angle_params(config_.view_direction, config_.view_width);
  init_sin_cos_rot_table(sin_rot_table_, cos_rot_table_, ROTATION_MAX_UNITS,
                         ROTATION_RESOLUTION);
}

bool VelodyneParser::is_scan_valid(int rotation, float range) {
  // check range first
  if (range < config_.min_range || range > config_.max_range) {
    return false;
  }
  // condition added to avoid calculating points which are not
  // in the interesting defined area (min_angle < area < max_angle)
  // not used now
  // if ((config_.min_angle > config_.max_angle && (rotation <=
  // config_.max_angle || rotation >= config_.min_angle))
  //     || (config_.min_angle < config_.max_angle && rotation >=
  //     config_.min_angle
  //         && rotation <= config_.max_angle)) {
  //     return true;
  // }
  return true;
}

void VelodyneParser::compute_coords(const union RawDistance &raw_distance,
                                    const LaserCorrection &corrections,
                                    const uint16_t &rotation, VPoint &point) {
  ROS_ASSERT_MSG(rotation < 36000, "rotation must between 0 and 35999");
  double x = 0.0;
  double y = 0.0;
  double z = 0.0;

  double distance1 = raw_distance.raw_distance * DISTANCE_RESOLUTION;
  double distance = distance1 + corrections.dist_correction;

  // cos(a-b) = cos(a)*cos(b) + sin(a)*sin(b)
  // sin(a-b) = sin(a)*cos(b) - cos(a)*sin(b)
  double cos_rot_angle =
      cos_rot_table_[rotation] * corrections.cos_rot_correction +
      sin_rot_table_[rotation] * corrections.sin_rot_correction;
  double sin_rot_angle =
      sin_rot_table_[rotation] * corrections.cos_rot_correction -
      cos_rot_table_[rotation] * corrections.sin_rot_correction;

  double vert_offset = corrections.vert_offset_correction;

  // Compute the distance in the xy plane (w/o accounting for rotation)
  double xy_distance = distance * corrections.cos_vert_correction;

  // Calculate temporal X, use absolute value.
  double xx = fabs(xy_distance * sin_rot_angle -
                   corrections.horiz_offset_correction * cos_rot_angle);
  // Calculate temporal Y, use absolute value
  double yy = fabs(xy_distance * cos_rot_angle +
                   corrections.horiz_offset_correction * sin_rot_angle);

  // Get 2points calibration values,Linear interpolation to get distance
  // correction for X and Y, that means distance correction use
  // different value at different distance
  double distance_corr_x = 0;
  double distance_corr_y = 0;

  if (need_two_pt_correction_ && distance1 <= 2500) {
    distance_corr_x =
        (corrections.dist_correction - corrections.dist_correction_x) *
            (xx - 2.4) / 22.64 +
        corrections.dist_correction_x;  // 22.64 = 25.04 - 2.4
    distance_corr_y =
        (corrections.dist_correction - corrections.dist_correction_y) *
            (yy - 1.93) / 23.11 +
        corrections.dist_correction_y;  // 23.11 = 25.04 - 1.93
  } else {
    distance_corr_x = distance_corr_y = corrections.dist_correction;
  }

  double distance_x = distance1 + distance_corr_x;

  xy_distance = distance_x * corrections.cos_vert_correction;
  // xy_distance = distance_x * cos_vert_correction - vert_offset *
  // sin_vert_correction;

  x = xy_distance * sin_rot_angle -
      corrections.horiz_offset_correction * cos_rot_angle;

  double distance_y = distance1 + distance_corr_y;
  xy_distance = distance_y * corrections.cos_vert_correction;
  // xy_distance = distance_y * cos_vert_correction - vert_offset *
  // sin_vert_correction;
  y = xy_distance * cos_rot_angle +
      corrections.horiz_offset_correction * sin_rot_angle;

  z = distance * corrections.sin_vert_correction +
      corrections.vert_offset_correction;
  // z = distance * sin_vert_correction + vert_offset * cos_vert_correction;

  /** Use standard ROS coordinate system (right-hand rule) */
  point.x = float(y);
  point.y = float(-x);
  point.z = float(z);
}

VelodyneParser *VelodyneParserFactory::create_parser(Config config) {
  if (config.model == "VLP16") {
    config.calibration_online = false;
    return new Velodyne16Parser(config);
  } else if (config.model == "64E_S2" || config.model == "64E_S3S" ||
             config.model == "64E_S3D_STRONGEST" ||
             config.model == "64E_S3D_LAST" || config.model == "64E_S3D_DUAL") {
    return new Velodyne64Parser(config);
  } else {
    ROS_ERROR_STREAM("invalid model, must be 64E_S2|64E_S3S"
                     << "|64E_S3D_STRONGEST|64E_S3D_LAST|64E_S3D_DUAL");
    return nullptr;
  }
}

}  // namespace velodyne
}  // namespace drivers
}  // namespace apollo
