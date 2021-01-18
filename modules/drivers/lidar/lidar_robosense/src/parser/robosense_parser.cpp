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
#include "modules/drivers/lidar/lidar_robosense/include/parser/robosense_parser.h"

#include <pcl/common/time.h>

#include <algorithm>
#include <fstream>

#include "cyber/cyber.h"

namespace apollo {
namespace drivers {
namespace robosense {

uint64_t RobosenseParser::get_gps_stamp(double current_packet_stamp,
                                        double* previous_packet_stamp,
                                        uint64_t* gps_base_usec) {
  if (std::abs(*previous_packet_stamp - current_packet_stamp) >
      3599000000) {                                       //微妙 us
    *gps_base_usec += static_cast<uint64_t>(3600 * 1e6);  //+ 1小时   单位微妙
    AINFO << "gps_base+1---current_stamp:" << current_packet_stamp
          << "previous_stamp:" << previous_packet_stamp;
  }
  if (current_packet_stamp >= *previous_packet_stamp) {
    // AINFO<<"curr - pre:"<<(current_packet_stamp -
    // previous_packet_stamp)*1e-3;
    if (*previous_packet_stamp != 0 &&
        current_packet_stamp - *previous_packet_stamp > 100000) {
      AERROR << "Current stamp:" << std::fixed << current_packet_stamp
             << " ahead previous stamp:" << previous_packet_stamp
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

apollo::drivers::PointXYZIT RobosenseParser::get_nan_point(uint64_t timestamp) {
  apollo::drivers::PointXYZIT nan_point;
  nan_point.set_timestamp(timestamp);
  nan_point.set_x(nan);
  nan_point.set_y(nan);
  nan_point.set_z(nan);
  nan_point.set_intensity(0);
  return nan_point;
}

RobosenseParser::RobosenseParser(
    const apollo::drivers::suteng::SutengConfig& config)
    : _config(config), _last_time_stamp(0), _mode(STRONGEST) {}

void RobosenseParser::init_angle_params(double view_direction,
                                        double view_width) {
  // converting angle parameters into the suteng reference (rad)
  double tmp_min_angle = view_direction + view_width / 2;
  double tmp_max_angle = view_direction - view_width / 2;

  // computing positive modulo to keep theses angles into [0;2*M_PI]
  tmp_min_angle = fmod(fmod(tmp_min_angle, 2 * M_PI) + 2 * M_PI, 2 * M_PI);
  tmp_max_angle = fmod(fmod(tmp_max_angle, 2 * M_PI) + 2 * M_PI, 2 * M_PI);

  // converting into the hardware suteng ref (negative yaml and degrees)
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
void RobosenseParser::setup() {
  _calibration.read(_config.calibration_file());

  if (!_calibration._initialized) {
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
    apollo::drivers::suteng::LidarsFilter lidarsFilter;
    if (!apollo::cyber::common::GetProtoFromFile(
            _config.lidars_filter_config_path(), &lidarsFilter)) {
      AERROR << "Failed to load config file";
      return;
    }
    AINFO << "lidarsFilter Config:" << lidarsFilter.DebugString();

    for (int i = 0; i < lidarsFilter.lidar_size(); i++) {
      if (lidarsFilter.lidar(i).frame_id() == _config.frame_id()) {
        apollo::drivers::suteng::Lidar lidar(lidarsFilter.lidar(i));
        _filter_grading = lidar.grading();
        for (int j = 0; j < lidar.point_size(); j++) {
          _filter_set.insert(lidar.point(j));
        }
        break;
      }
    }
  }
}

bool RobosenseParser::is_scan_valid(int rotation, float range) {
  // check range first
  if (range < _config.min_range() || range > _config.max_range()) {
    return false;
  }
  (void)rotation;
  return true;
}

void RobosenseParser::timestamp_check(double timestamp) { (void)timestamp; }

void RobosenseParser::init_sin_cos_rot_table(float* sin_rot_table,
                                             float* cos_rot_table,
                                             uint16_t rotation,
                                             float rotation_resolution) {
  for (uint16_t rot_index = 0; rot_index < rotation; ++rot_index) {
    float rotation = rotation_resolution * rot_index * M_PI / 180.0;
    cos_rot_table[rot_index] = cosf(rotation);
    sin_rot_table[rot_index] = sinf(rotation);
  }
}

RobosenseParser* RobosenseParserFactory::create_parser(
    const apollo::drivers::suteng::SutengConfig& config) {
  if (config.model() == apollo::drivers::suteng::VLP16) {
    return new Robosense16Parser(config);
  } else {
    AERROR << " invalid model, must be VLP16";
    return nullptr;
  }
}
int RobosenseParser::EstimateTemperature(float temperature) {
  int temp = static_cast<int>(floor(temperature + 0.5));
  if (temp < TEMPERATURE_MIN) {
    temp = TEMPERATURE_MIN;
  } else if (temp > TEMPERATURE_MIN + TEMPERATURE_RANGE) {
    temp = TEMPERATURE_MIN + TEMPERATURE_RANGE;
  }
  return temp;
}
float RobosenseParser::CalibIntensity(float intensity,
                                      int calIdx,  // 1-16
                                      int distance, float temper) {
  int algDist;
  int sDist;
  int uplimitDist;
  float realPwr;
  float refPwr;
  float tempInten;
  float distance_f;
  float endOfSection1;

  int temp = EstimateTemperature(temper);
  realPwr = std::max(
      static_cast<float>(
          intensity / (1 + static_cast<float>(temp - TEMPERATURE_MIN) / 24.0f)),
      1.0f);

  if (static_cast<int>(realPwr < 126)) {
    realPwr = realPwr * 4.0f;
  } else {
    if (static_cast<int>(realPwr >= 126) && static_cast<int>(realPwr < 226)) {
      realPwr = (realPwr - 125.0f) * 16.0f + 500.0f;
    } else {
      realPwr = (realPwr - 225.0f) * 256.0f + 2100.0f;
    }
  }

  int indexTemper = EstimateTemperature(temper) - TEMPERATURE_MIN;
  uplimitDist = CHANNEL_NUM[calIdx][indexTemper] + 20000;
  // limit sDist
  sDist = (distance > CHANNEL_NUM[calIdx][indexTemper])
              ? distance
              : CHANNEL_NUM[calIdx][indexTemper];
  sDist = (sDist < uplimitDist) ? sDist : uplimitDist;
  // minus the static offset (this data is For the intensity cal useage only)
  algDist = sDist - CHANNEL_NUM[calIdx][indexTemper];

  // calculate intensity ref curves
  float refPwr_temp = 0.0f;
  int order = 3;
  endOfSection1 = 500.0f;
  distance_f = static_cast<float>(algDist);
  if (distance_f <= endOfSection1) {
    refPwr_temp = INTENSITY_CAL[0][calIdx] *
                      static_cast<float>(
                          exp(INTENSITY_CAL[1][calIdx] -
                              INTENSITY_CAL[2][calIdx] * distance_f / 100.0f)) +
                  INTENSITY_CAL[3][calIdx];
  } else {
    for (int i = 0; i < order; i++) {
      refPwr_temp +=
          static_cast<float>(INTENSITY_CAL[i + 4][calIdx] *
                             pow(distance_f / 100.0f, order - 1 - i));
    }
  }

  refPwr = std::max(std::min(refPwr_temp, 500.0f), 4.0f);
  tempInten = (51 * refPwr) / realPwr;

  tempInten = static_cast<int>(tempInten > 255 ? 255.0f : tempInten);
  return tempInten;
}
float RobosenseParser::PixelToDistance(int pixelValue, int passageway,
                                       float temper) {
  temper = 31.0;
  float distance_value;
  int indexTemper = EstimateTemperature(temper) - TEMPERATURE_MIN;
  if (pixelValue <= CHANNEL_NUM[passageway][indexTemper]) {
    distance_value = 0.0;
  } else {
    distance_value =
        static_cast<float>(pixelValue - CHANNEL_NUM[passageway][indexTemper]);
  }
  return distance_value;
}

float RobosenseParser::compute_temperature(unsigned char bit1,
                                           unsigned char bit2) {
  float Temp;
  float bitneg = bit2 & 128;   // 10000000
  float highbit = bit2 & 127;  // 01111111
  float lowbit = bit1 >> 3;
  if (bitneg == 128) {
    Temp = -1 * (highbit * 32 + lowbit) * 0.0625f;
  } else {
    Temp = (highbit * 32 + lowbit) * 0.0625f;
  }

  return Temp;
}
}  // namespace robosense
}  // namespace drivers
}  // namespace apollo
