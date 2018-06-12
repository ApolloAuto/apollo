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

#ifndef MODULES_PERCEPTION_OBSTACLE_LIDAR_VISUALIZER_OPENGL_VISUALIZER_H_
#define MODULES_PERCEPTION_OBSTACLE_LIDAR_VISUALIZER_OPENGL_VISUALIZER_H_

#include <boost/shared_ptr.hpp>
#include <sstream>
#include <string>
#include <vector>

#include "modules/perception/common/pcl_types.h"
#include "modules/perception/obstacle/lidar/visualizer/opengl_visualizer/glfw_viewer.h"

namespace apollo {
namespace perception {

class OpenglVisualizer {
 public:
  OpenglVisualizer();
  virtual ~OpenglVisualizer() = default;

  virtual bool Init();

  virtual std::string Name() const { return name_; }

  void UpdateCameraSystem(FrameContent *content);

  virtual void Render(const FrameContent &content);

 private:
  void SetSize(int w, int h);
  void SetBackgroundColor(float r, float g, float b, float a);
  void SetVelodyneHeight(float h);
  void SetMainCarPoints();
  void SetCameraPosition();

  float velodyne_height_;
  pcl_util::Point camera_center_velodyne_;
  pcl_util::Point view_point_velodyne_;
  pcl_util::Point up_velodyne_;
  pcl_util::Point forward_velodyne_;
  pcl_util::PointCloud main_car_points_velodyne_;

  pcl_util::Point camera_center_world_;
  pcl_util::Point view_point_world_;
  pcl_util::Point up_world_;
  pcl_util::Point forward_world_;
  pcl_util::PointCloud main_car_points_world_;

  boost::shared_ptr<GLFWViewer> opengl_vs_;
  std::string name_;
};

}  // namespace perception
}  // namespace apollo

#endif  //  MODULES_PERCEPTION_OBSTACLE_LIDAR_VISUALIZER_OPENGL_VISUALIZER_H_
