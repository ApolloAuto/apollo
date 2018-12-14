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
 * @file
 * @brief This file provides the declaration of the class
 * "TrajectoryConverter".
 */

#ifndef MODULES_TOOLS_NAVI_GENERATOR_BACKEND_UTIL_TRAJECTORY_CONVERTER_H_
#define MODULES_TOOLS_NAVI_GENERATOR_BACKEND_UTIL_TRAJECTORY_CONVERTER_H_

#include <string>
#include <thread>
#include <utility>
#include <vector>

#include "modules/localization/msf/common/util/frame_transform.h"
#include "modules/planning/reference_line/reference_point.h"
#include "modules/tools/navi_generator/backend/util/trajectory_smoother.h"

/**
 * @namespace apollo::planning
 * @brief apollo::planning
 */
namespace apollo {
namespace navi_generator {
namespace util {

class TrajectoryConverter {
 public:
  TrajectoryConverter() = default;
  virtual ~TrajectoryConverter() = default;

 public:
  /**
   * @brief The trajectory points with the specified length is extracted from
   * a bag file, and the length value of -1.0 indicates that all the data is
   * extracted.
   * @param bag_filename The trajectory's bag filename.
   * @param length The data with specified length is extracted and -1.0
   * indicates that all the data is extracted.
   * @param need_clear_buffer The data buffer needs to be cleared before saving
   * the new data?
   * @param is_written_to_file The extracted data is written to a TXT file?
   * @return Return true for success.
   */
  bool ExtractTrajectoryPoints(const std::string& bag_filename,
                               double length = -1.0,
                               bool need_clear_buffer = true,
                               bool is_written_to_file = false);

  /**
   * @brief Extract all the trajectory points from the first bag and some points
   * with the specified length from the second one. Next, the two pieces of data
   * are stitched together in order.
   * @param first_bag_filename The first trajectory's bag filename.
   * @param second_bag_filename The second trajectory's bag filename.
   * @param length_from_second_bag The data with specified length is extracted
   * from the second bag and -1.0 indicates that all the data is extracted.
   * @return Return true for success.
   */
  bool ExtractTrajectoryPointsFromTwoBags(
      const std::string& first_bag_filename,
      const std::string& second_bag_filename, double length_from_second_bag);
  /**
   * @brief Smooth the extracted trajectory points.
   * @return Return true for success.
   */
  bool SmoothTrajectoryPoints();

  /**
   * @brief Save the raw trajectory points to a disk file.
   * @return Return true for success.
   */
  bool SaveRawTrajectoryPoints();
  /**
   * @brief Save the smoothed trajectory points to a disk file.
   * @return Return true for success.
   */
  bool SaveSmoothedTrajectoryPoints();

  /**
   * @brief Get UTM zone.
   */
  inline std::size_t GetUTMZone(double lon) {
    return static_cast<int>(((lon + 180.0) / 6) + 1);
  }

  const std::string& GetSmoothedFileName() const { return smoothed_filename_; }

  bool GetSmoothedTrajectoryWGS84Points(
      std::vector<apollo::localization::msf::WGS84Corr>* const waypoints);

  bool ConvertSmoothedTrajectoryPointsToWGS84(
      const std::vector<planning::ReferencePoint>* const smoothed_points,
      std::vector<apollo::localization::msf::WGS84Corr>* const waypoints);

 private:
  TrajectorySmoother trajectory_smoother_;
  std::size_t local_utm_zone_id_ = 49;
  std::string extracted_filename_;
  std::string smoothed_filename_;
  std::vector<common::math::Vec2d> raw_points_;
  std::vector<planning::ReferencePoint> smoothed_points_;
};

}  // namespace util
}  // namespace navi_generator
}  // namespace apollo

#endif  // MODULES_TOOLS_NAVI_GENERATOR_BACKEND_UTIL_TRAJECTORY_CONVERTER_H_
