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

#pragma once
#include <map>
#include <memory>
#include <string>

namespace apollo {
namespace drivers {
namespace robosense {

struct LaserCorrection {
  /** parameters in db.xml */
  float rot_correction;
  float vert_correction;
  float dist_correction;
  float dist_correction_x;
  float dist_correction_y;
  float vert_offset_correction;
  float horiz_offset_correction;
  int max_intensity;
  int min_intensity;
  float focal_distance;
  float focal_slope;
  float focal_offset;

  float cos_rot_correction;   ///< cached cosine of rot_correction
  float sin_rot_correction;   ///< cached sine of rot_correction
  float cos_vert_correction;  ///< cached cosine of vert_correction
  float sin_vert_correction;  ///< cached sine of vert_correction

  int laser_ring;  ///< ring number for this laser
};

class Calibration {
 public:
  std::map<int, LaserCorrection> laser_corrections_;
  int num_lasers_;
  bool initialized_;

 public:
  Calibration() : initialized_(false) {}
  explicit Calibration(const std::string& calibration_file) {
    read(calibration_file);
  }

  void read(const std::string& calibration_file);
  void write(const std::string& calibration_file);
};

}  // namespace robosense
}  // namespace drivers
}  // namespace apollo
