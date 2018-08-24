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

/**
 * \file  calibration.h
 *
 * \author  Piyush Khandelwal (piyushk@cs.utexas.edu)
 * Copyright (C) 2012, Austin Robot Technology, University of Texas at Austin
 *
 * License: Modified BSD License
 *
 * Id:02/14/201211:25:34AMpiyushk
 */

#ifndef MODULES_DRIVERS_LIDAR_VELODYNE_POINTCLOUD_CALIBRATION_H_
#define MODULES_DRIVERS_LIDAR_VELODYNE_POINTCLOUD_CALIBRATION_H_

#include <map>
#include <string>

namespace apollo {
namespace drivers {
namespace lidar_velodyne {

/** \brief correction values for a single laser
 *
 * Correction values for a single laser (as provided by db.xml from
 * Velodyne).  Includes parameters for Velodyne HDL-64E S2.1.
 *
 * http://velodynelidar.com/lidar/products/manual/63-HDL64E%20S2%20Manual_Rev%20D_2011_web.pdf
 */

/** \brief Correction information for a single laser. */
struct LaserCorrection {
  /** parameters in db.xml */
  float rot_correction;
  float vert_correction;
  float dist_correction;
  bool two_pt_correction_available;
  float dist_correction_x;
  float dist_correction_y;
  float vert_offset_correction;
  float horiz_offset_correction;
  int max_intensity;
  int min_intensity;
  float focal_distance;
  float focal_slope;

  /** cached values calculated when the calibration file is read */
  float cos_rot_correction;   ///< cosine of rot_correction
  float sin_rot_correction;   ///< sine of rot_correction
  float cos_vert_correction;  ///< cosine of vert_correction
  float sin_vert_correction;  ///< sine of vert_correction

  int laser_ring;  ///< ring number for this laser
};

/** \brief Calibration information for the entire device. */
class Calibration {
 public:
  std::map<int, LaserCorrection> laser_corrections;
  int num_lasers;
  bool initialized;
  bool ros_info;

 public:
  explicit Calibration(bool info = true) : initialized(false), ros_info(info) {}
  explicit Calibration(const std::string& calibration_file, bool info = true)
      : ros_info(info) {
    read(calibration_file);
  }

  void read(const std::string& calibration_file);
  void write(const std::string& calibration_file);
};

}  // namespace lidar_velodyne
}  // namespace drivers
}  // namespace apollo

#endif  // MODULES_DRIVERS_LIDAR_VELODYNE_POINTCLOUD_CALIBRATION_H_
