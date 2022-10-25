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

#include "modules/drivers/lidar_surestar/parser/surestar_parser.h"

#include <fstream>
#include <string>

#include <pcl/common/time.h>

#include "cyber/cyber.h"

namespace apollo {
namespace drivers {
namespace surestar {

uint64_t SurestarParser::get_gps_stamp(double current_packet_stamp,
                                       double* previous_packet_stamp,
                                       uint64_t* gps_base_usec) {
  if (std::abs(*previous_packet_stamp - current_packet_stamp) >
      3599000000) {                                       // 微妙 us
    *gps_base_usec += static_cast<uint64_t>(3600 * 1e6);  // + 1小时单位微妙
    AINFO << "gps_base+1---current_stamp:" << current_packet_stamp
          << "previous_stamp:" << previous_packet_stamp;
  }
  if (current_packet_stamp >= *previous_packet_stamp) {
    // AINFO<<"curr - pre:"<<(current_packet_stamp -
    // previous_packet_stamp)*1e-3;
    if (*previous_packet_stamp != 0 &&
        current_packet_stamp - *previous_packet_stamp > 100000) {
      AERROR << "Current stamp:" << std::fixed << current_packet_stamp
             << " ahead previous stamp:" << *previous_packet_stamp
             << " over 100ms. GPS time stamp incorrect!";
    }
  }

  *previous_packet_stamp = current_packet_stamp;
  uint64_t gps_stamp =
      *gps_base_usec +
      static_cast<uint64_t>(current_packet_stamp + 1000000);  // us

  gps_stamp = gps_stamp * 1000;
  return gps_stamp;
}

apollo::drivers::PointXYZIT SurestarParser::get_nan_point(uint64_t timestamp) {
  apollo::drivers::PointXYZIT nan_point;
  nan_point.set_timestamp(timestamp);
  nan_point.set_x(nan);
  nan_point.set_y(nan);
  nan_point.set_z(nan);
  nan_point.set_intensity(0);
  return nan_point;
}

SurestarParser::SurestarParser(
    const apollo::drivers::surestar::SurestarConfig& config)
    : _config(config), _last_time_stamp(0), _mode(STRONGEST) {}

void SurestarParser::init_angle_params(double view_direction,
                                       double view_width) {
  // converting angle parameters into the surestar reference (rad)
  double tmp_min_angle = view_direction + view_width / 2;
  double tmp_max_angle = view_direction - view_width / 2;

  // computing positive modulo to keep theses angles into [0;2*M_PI]
  tmp_min_angle = fmod(fmod(tmp_min_angle, 2 * M_PI) + 2 * M_PI, 2 * M_PI);
  tmp_max_angle = fmod(fmod(tmp_max_angle, 2 * M_PI) + 2 * M_PI, 2 * M_PI);

  // converting into the hardware surestar ref (negative yaml and degrees)
  // adding 0.5 perfomrs a centered double to int conversion
  _config.set_min_angle(100 * (2 * M_PI - tmp_min_angle) * 180 / M_PI + 0.5);
  _config.set_max_angle(100 * (2 * M_PI - tmp_max_angle) * 180 / M_PI + 0.5);
  if (_config.min_angle() == _config.max_angle()) {
    // avoid returning empty cloud if min_angle = max_angle
    _config.set_min_angle(0);
    _config.set_max_angle(36000);
  }
}

/** Set up for on-line operation. */
void SurestarParser::setup() {
  _calibration.read(_config.calibration_file());

  if (!_calibration.initialized_) {
    AERROR << " Unable to open calibration file: "
           << _config.calibration_file();
  }

  // setup angle parameters.
  init_angle_params(_config.view_direction(), _config.view_width());
  init_sin_cos_rot_table(_sin_rot_table, _cos_rot_table, ROTATION_MAX_UNITS,
                         ROTATION_RESOLUTION);

  // get lidars_filter_config and put them into _filter_set
  if (!_config.lidars_filter_config_path().empty()) {
    // read config file
    apollo::drivers::surestar::LidarsFilter lidarsFilter;
    if (!apollo::cyber::common::GetProtoFromFile(
            _config.lidars_filter_config_path(), &lidarsFilter)) {
      AERROR << "Failed to load config file";
      return;
    }
    AINFO << "lidarsFilter Config:" << lidarsFilter.DebugString();

    for (int i = 0; i < lidarsFilter.lidar_size(); i++) {
      if (lidarsFilter.lidar(i).frame_id() == _config.frame_id()) {
        apollo::drivers::surestar::Lidar lidar(lidarsFilter.lidar(i));
        _filter_grading = lidar.grading();
        for (int j = 0; j < lidar.point_size(); j++) {
          _filter_set.insert(lidar.point(j));
        }
        break;
      }
    }
  }
}

bool SurestarParser::is_scan_valid(int rotation, float range) {
  // check range first
  if (range < _config.min_range() || range > _config.max_range()) {
    return false;
  }
  (void)rotation;
  return true;
}

void SurestarParser::compute_coords_beike(const float& raw_distance,
                                          const uint16_t rotation,
                                          const int laserChannel,
                                          apollo::drivers::PointXYZIT* point,
                                          uint32_t* nan_pts) {
  assert(rotation <= 36000);
  double x = 0.0;
  double y = 0.0;
  double z = 0.0;

  double distance = raw_distance;
  double tmpBeta = rotation * ROTATION_RESOLUTION + HANGLE_V6K_16[laserChannel];
  double tmpTheta = VANGLE_V6K_16[laserChannel];
  if (tmpBeta > 360.0f) {
    tmpBeta -= 360.0f;
  }
  if (tmpBeta < 0) {
    tmpBeta += 360.0f;
  }

  x = -distance * cos(tmpTheta * M_PI / 180.0) * cos(-tmpBeta * M_PI / 180.0);
  y = -distance * cos(tmpTheta * M_PI / 180.0) * sin(-tmpBeta * M_PI / 180.0);
  z = distance * sin(tmpTheta * M_PI / 180.0);

  if (_filter_set.size() > 0) {
    std::string key =
        std::to_string(static_cast<int>(x * 100 / _filter_grading)) + "+" +
        std::to_string(static_cast<int>(y * 100 / _filter_grading));
    if (_filter_set.find(key) != _filter_set.end()) {
      x = NAN;
      y = NAN;
      z = NAN;
      (*nan_pts)++;
    }
  }

  point->set_x(static_cast<float>(x));
  point->set_y(static_cast<float>(y));
  point->set_z(static_cast<float>(z));
}

void SurestarParser::timestamp_check(double timestamp) { (void)timestamp; }

void SurestarParser::init_sin_cos_rot_table(float* sin_rot_table,
                                            float* cos_rot_table,
                                            uint16_t rotation,
                                            float rotation_resolution) {
  for (uint16_t rot_index = 0; rot_index < rotation; ++rot_index) {
    float rotation = rotation_resolution * rot_index * M_PI / 180.0;
    cos_rot_table[rot_index] = cosf(rotation);
    sin_rot_table[rot_index] = sinf(rotation);
  }
}

SurestarParser* SurestarParserFactory::create_parser(
    const apollo::drivers::surestar::SurestarConfig& config) {
  if (config.model() == apollo::drivers::Surestar::RFANS16) {
    return new Surestar16Parser(config);
  } else {
    AERROR << " invalid model, must be RFANS16";
    return nullptr;
  }
}

}  // namespace surestar
}  // namespace drivers
}  // namespace apollo
