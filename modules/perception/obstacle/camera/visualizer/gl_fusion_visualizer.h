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

#ifndef MODULES_PERCEPTION_OBSTACLE_CAMERA_VISUALIZER_GL_FUSION_VISUALIZER_H_
#define MODULES_PERCEPTION_OBSTACLE_CAMERA_VISUALIZER_GL_FUSION_VISUALIZER_H_
#include <memory>
#include <sstream>
#include <string>
#include <vector>
#include "modules/perception/obstacle/camera/visualizer/base_visualizer.h"
#include "modules/perception/obstacle/camera/visualizer/frame_content.h"
#include "modules/perception/obstacle/camera/visualizer/glfw_fusion_viewer.h"

namespace apollo {
namespace perception {
namespace lowcostvisualizer {

class GLFusionVisualizer : public BaseVisualizer {
 public:
  GLFusionVisualizer();

  virtual ~GLFusionVisualizer() = default;

  bool init() override;

  std::string name() const override { return name_; }

  void update_camera_system(FrameContent *content) override;

  void render(FrameContent *content) override;

 private:
  void set_background_color(float r, float g, float b, float a);

  void set_main_car_points();

  void set_camera_position();

  Eigen::Vector3d camera_center_velodyne_;
  Eigen::Vector3d view_point_velodyne_;
  Eigen::Vector3d up_velodyne_;
  Eigen::Vector3d forward_velodyne_;
  std::vector<Eigen::Vector3d> main_car_;

  Eigen::Vector3d camera_center_world_;
  Eigen::Vector3d view_point_world_;
  Eigen::Vector3d up_world_;
  Eigen::Vector3d forward_world_;
  std::vector<Eigen::Vector3d> main_car_world_;

  std::shared_ptr<GLFWFusionViewer> opengl_vs_;
  std::string name_;
  bool init_ = false;
  DISALLOW_COPY_AND_ASSIGN(GLFusionVisualizer);

  //    boost::circular_buffer<VehicleStatus> motion_buffer_;
};

REGISTER_VISUALIZER(GLFusionVisualizer);

}  // namespace lowcostvisualizer
}  // namespace perception
}  // namespace apollo

#endif  //  APOLLO_PERCEPTION_OBSTACLE_VISUALIZER_PCL_VISUALIZER_H
