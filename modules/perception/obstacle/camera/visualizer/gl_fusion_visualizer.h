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
#include <sstream>
#include <string>
#include <vector>
#include <memory>
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

  std::string name() const override {
    return _name;
  }
  //    void set_motion_buffer(MotionBuffer &motion_buffer) override {
  //        _motion_buffer = motion_buffer;
  //    }

  void update_camera_system(FrameContent *content) override;

  void render(FrameContent *content) override;

 private:
  void set_background_color(float r, float g, float b, float a);

  void set_main_car_points();

  void set_camera_position();

  Eigen::Vector3d _camera_center_velodyne;
  Eigen::Vector3d _view_point_velodyne;
  Eigen::Vector3d _up_velodyne;
  Eigen::Vector3d _forward_velodyne;
  std::vector<Eigen::Vector3d> _main_car;

  Eigen::Vector3d _camera_center_world;
  Eigen::Vector3d _view_point_world;
  Eigen::Vector3d _up_world;
  Eigen::Vector3d _forward_world;
  std::vector<Eigen::Vector3d> _main_car_world;

  std::shared_ptr<GLFWFusionViewer> _opengl_vs;
  std::string _name;
  bool _init = false;
  DISALLOW_COPY_AND_ASSIGN(GLFusionVisualizer);

  //    boost::circular_buffer<VehicleStatus> _motion_buffer;
};

}  // namespace lowcostvisualizer
}  // namespace perception
}  // namespace apollo

#endif  //  APOLLO_PERCEPTION_OBSTACLE_VISUALIZER_PCL_VISUALIZER_H
