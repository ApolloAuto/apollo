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

#pragma once

#include <memory>
#include <string>
#include <vector>

#include "modules/perception/common/base/object.h"
#include "modules/perception/common/base/point.h"
#include "modules/perception/common/base/point_cloud.h"
#include "modules/perception/common/lidar/common/lidar_frame.h"

namespace apollo {
namespace perception {
namespace lidar {

static const float kEpsilon = 1e-6f;
static const float kEpsilonForSize = 1e-2f;
static const float kEpsilonForLine = 1e-3f;

using apollo::perception::base::PointD;
using apollo::perception::base::PointF;

using ObjectPtr = std::shared_ptr<apollo::perception::base::Object>;
using PointFCloud = apollo::perception::base::PointCloud<PointF>;
using PolygonDType = apollo::perception::base::PointCloud<PointD>;

struct ObjectBuilderInitOptions {};

struct ObjectBuilderOptions {
  Eigen::Vector3d ref_center = Eigen::Vector3d(0, 0, 0);
};

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
      const ObjectBuilderInitOptions& options = ObjectBuilderInitOptions());

  /**
   * @brief Calculate and fill object size, center, directions.
   * 
   * @param options object builder options
   * @param frame lidar frame
   * @return true 
   * @return false 
   */
  bool Build(const ObjectBuilderOptions& options, LidarFrame* frame);

  /**
   * @brief Name of Object Builder
   * 
   * @return std::string 
   */
  std::string Name() const { return "ObjectBuilder"; }

 public:
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

  // @brief: calculate height_above_ground.
  // @param [in/out]: ObjectPtr.
  void ComputeHeightAboveGround(
      std::shared_ptr<apollo::perception::base::Object> object);

  // @brief: calculate and fill front-critical.
  // @param [in/out]: ObjectPtr and pose
  void JudgeFrontCritical(
      std::shared_ptr<apollo::perception::base::Object> object,
      Eigen::Affine3d& lidar2novatel_pose);

  // @brief: calculate and fill default polygon value.
  // @param [in]: min and max point.
  // @param [in/out]: ObjectPtr.
  void SetDefaultValue(
      const Eigen::Vector3f& min_pt, const Eigen::Vector3f& max_pt,
      std::shared_ptr<apollo::perception::base::Object> object);

  // @brief: decide whether input cloud is on the same line.
  //         if ture, add perturbation.
  // @param [in/out]: pointcloud.
  // @param [out]: is line: true, not line: false.
  bool LinePerturbation(
      apollo::perception::base::PointCloud<apollo::perception::base::PointF>*
          cloud);

  // @brief: calculate 3D min max point
  // @param [in]: point cloud.
  // @param [in/out]: min and max points.
  void GetMinMax3D(const apollo::perception::base::PointCloud<
                       apollo::perception::base::PointF>& cloud,
                   Eigen::Vector3f* min_pt, Eigen::Vector3f* max_pt);
};  // class ObjectBuilder

}  // namespace lidar
}  // namespace perception
}  // namespace apollo
