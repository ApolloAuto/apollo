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

#include "modules/perception/base/object.h"
#include "modules/perception/base/point.h"
#include "modules/perception/base/point_cloud.h"
#include "modules/perception/lib/registerer/registerer.h"
#include "modules/perception/lidar/common/lidar_frame.h"
#include "modules/perception/pipeline/stage.h"

namespace apollo {
namespace perception {
namespace lidar {

struct ObjectBuilderInitOptions {};

struct ObjectBuilderOptions {
  Eigen::Vector3d ref_center = Eigen::Vector3d(0, 0, 0);
};

class ObjectBuilder : public pipeline::Stage {
 public:
  using DataFrame = pipeline::DataFrame;
  using Plugin = pipeline::Plugin;
  using PluginType = pipeline::PluginType;
  using StageConfig = pipeline::StageConfig;

 public:
  ObjectBuilder() = default;
  ~ObjectBuilder() = default;

  // @brief: initialization. Get orientation estimator instance.
  // @param [in]: ObjectBuilderInitOptions.
  bool Init(
      const ObjectBuilderInitOptions& options = ObjectBuilderInitOptions());

  // @brief: calculate and fill object size, center, directions.
  // @param [in]: ObjectBuilderOptions.
  // @param [in/out]: LidarFrame*.
  bool Build(const ObjectBuilderOptions& options, LidarFrame* frame);

  bool Init(const StageConfig& stage_config) override;

  bool Process(DataFrame* data_frame) override;

  bool IsEnabled() const override { return enable_; }

  std::string Name() const override { return name_; }

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
