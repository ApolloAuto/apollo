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
 * @brief This file provides the declaration of the class "TrajectorySmoother".
 */

#ifndef MODULES_TOOLS_NAVI_GENERATOR_BACKEND_UTIL_TRAJECTORY_SMOOTHER_H_
#define MODULES_TOOLS_NAVI_GENERATOR_BACKEND_UTIL_TRAJECTORY_SMOOTHER_H_

#include <string>
#include <vector>

#include "modules/planning/proto/reference_line_smoother_config.pb.h"
#include "modules/planning/reference_line/reference_line.h"
#include "modules/planning/reference_line/reference_line_smoother.h"
#include "modules/planning/reference_line/reference_point.h"
#include "modules/tools/navi_generator/proto/trajectory_util_config.pb.h"

/**
 * @namespace apollo::navi_generator::util
 * @brief apollo::navi_generator::util
 */
namespace apollo {
namespace navi_generator {
namespace util {

class TrajectorySmoother {
 public:
  TrajectorySmoother();
  ~TrajectorySmoother() = default;

 public:
  /**
   * @brief Read the raw trajectory points which need be smoothed from a disk
   * file.
   * @param filename The raw trajectory's filename.
   * @return  Return true for success.
   */
  bool Import(const std::string& filename);
  /**
   * @brief Smooth raw data.
   * @return  Return true for success.
   */
  bool Smooth();
  /**
   * @brief Save the smoothed trajectory points to a disk file.
   * @param filename The smoothed trajectory's filename.
   * @return  Return true for success.
   */
  bool Export(const std::string& filename);
  /**
   * @brief Get the smoothed data. It donesn't modify any member variables in
   * this class.
   * @param filename The smoothed trajectory's filename.
   * @return  Return the smoothed data.
   */
  const std::vector<apollo::planning::ReferencePoint>& smoothed_points() const {
    return ref_points_;
  }

 private:
  /**
   * @brief Create some anchor points for the purpose of smoothing.
   * @param init_point The starting point of the line segment where the
   * specified anchor point is to be created.
   * @param ref_line The line segment where the specified anchor point is to be
   * created.
   * @return  Return true for success.
   */
  bool CreateAnchorPoints(
      const apollo::planning::ReferencePoint& init_point,
      const apollo::planning::ReferenceLine& ref_line,
      std::vector<apollo::planning::AnchorPoint>* anchor_points);

 private:
  std::string filename_;
  std::vector<common::math::Vec2d> raw_points_;
  std::vector<apollo::planning::ReferencePoint> ref_points_;
  apollo::planning::ReferenceLine smoothed_ref_;
  apollo::planning::ReferenceLineSmootherConfig smoother_config_;
  apollo::navi_generator::TrajectorySmootherConfig traj_smoother_config_;
};

}  // namespace util
}  // namespace navi_generator
}  // namespace apollo

#endif  // MODULES_TOOLS_NAVI_GENERATOR_BACKEND_UTIL_TRAJECTORY_SMOOTHER_H_
