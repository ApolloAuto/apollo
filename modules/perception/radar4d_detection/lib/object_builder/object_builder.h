/******************************************************************************
 * Copyright 2023 The Apollo Authors. All Rights Reserved.
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

#include <memory>
#include <string>
#include <vector>

#include "modules/perception/common/base/object.h"
#include "modules/perception/common/base/point.h"
#include "modules/perception/common/base/point_cloud.h"
#include "modules/perception/common/lib/registerer/registerer.h"
#include "modules/perception/common/radar/common/radar_frame.h"
#include "modules/perception/radar4d_detection/interface/base_detector.h"

namespace apollo {
namespace perception {
namespace radar4d {

using apollo::perception::base::RadarPointCloud;

class ObjectBuilder {
 public:
  /**
   * @brief Construct a new Object Builder object
   * 
   */
  ObjectBuilder() = default;

  /**
   * @brief Destroy the Object Builder object
   * 
   */
  ~ObjectBuilder() = default;

  /**
   * @brief Init the Object Builder object with initialization options
   * 
   * @param options 
   * @return true 
   * @return false 
   */
  bool Init(
      const DetectorInitOptions& options = DetectorInitOptions());

  /**
   * @brief Calculate and fill object size, center, directions.
   * 
   * @param options object builder options
   * @param frame radar frame
   * @return true 
   * @return false 
   */
  bool Build(const DetectorOptions& options, RadarFrame* frame);

  /**
   * @brief Name of Object Builder
   * 
   * @return std::string 
   */
  std::string Name() const { return "ObjectBuilder"; }

 private:
  // @brief: calculate 2d polygon.
  //         and fill the convex hull vertices in object->polygon.
  // @param [in/out]: ObjectPtr.
  void ComputePolygon2D(
      std::shared_ptr<apollo::perception::base::Object> object);

  // @brief: calculate the size, center of polygon.
  // @param [in/out]: ObjectPtr.
  void ComputePolygonSizeCenter(
      std::shared_ptr<apollo::perception::base::Object> object);

  // @brief: calculate and fill timestamp and anchor_point.
  // @param [in/out]: ObjectPtr.
  void ComputeOtherObjectInformation(
      std::shared_ptr<apollo::perception::base::Object> object);

  // @brief: calculate and fill default polygon value.
  // @param [in]: min and max point.
  // @param [in/out]: ObjectPtr.
  void SetDefaultValue(
      const Eigen::Vector3f& min_pt, const Eigen::Vector3f& max_pt,
      std::shared_ptr<apollo::perception::base::Object> object);

  // @brief: decide whether input cloud is on the same line.
  //         if ture, add perturbation.
  // @param [in/out]: radarpointcloud.
  // @param [out]: is line: true, not line: false.
  bool LinePerturbation(
    RadarPointCloud<apollo::perception::base::RadarPointF>*
    cloud);

  // @brief: calculate 3D min max point
  // @param [in]: point cloud.
  // @param [in/out]: min and max points.
  void GetMinMax3D(const apollo::perception::base::RadarPointCloud<
                       apollo::perception::base::RadarPointF>& cloud,
                   Eigen::Vector3f* min_pt, Eigen::Vector3f* max_pt);
};  // class ObjectBuilder

}  // namespace radar4d
}  // namespace perception
}  // namespace apollo
