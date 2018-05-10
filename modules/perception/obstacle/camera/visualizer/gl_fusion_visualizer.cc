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

#include "modules/perception/obstacle/camera/visualizer/gl_fusion_visualizer.h"
#include "modules/common/log.h"

namespace apollo {
namespace perception {
namespace lowcostvisualizer {

GLFusionVisualizer::GLFusionVisualizer() : name_("GLFusionVisualizer") {}

bool GLFusionVisualizer::init() {
  opengl_vs_ = std::shared_ptr<GLFWFusionViewer>(new GLFWFusionViewer());

  if (opengl_vs_ == nullptr) {
    AINFO << "Failed to create opengl viewer";
    return false;
  }

  if (opengl_vs_->initialize() == false) {
    AINFO << "Failed to initialize opengl viewer";
    return false;
  }
  set_background_color(0.05, 0.05, 0.05, 0.0f);
  set_camera_position();
  set_main_car_points();
  AINFO << "Initialize GLFusionVisualizer successfully";
  return true;
}

void GLFusionVisualizer::render(FrameContent *content) {
  opengl_vs_->set_camera_para(camera_center_velodyne_, view_point_velodyne_,
                              up_velodyne_);
  opengl_vs_->set_forward_dir(forward_world_);
  opengl_vs_->set_main_car(main_car_world_);
  opengl_vs_->set_frame_content(content);
  opengl_vs_->spin_once();

  AINFO << "GLFusionVisualizer spin_once";
}

void GLFusionVisualizer::set_background_color(float r, float g, float b,
                                              float a) {
  opengl_vs_->set_background_color(Eigen::Vector3d(r, g, b));
}

void GLFusionVisualizer::set_camera_position() {
  up_velodyne_[0] = 0;
  up_velodyne_[1] = 1;
  up_velodyne_[2] = 0;
  forward_velodyne_[0] = 1;
  forward_velodyne_[1] = 0;
  forward_velodyne_[2] = 0;
  view_point_velodyne_[0] = 0;
  view_point_velodyne_[1] = 0;
  view_point_velodyne_[2] = 0;
  camera_center_velodyne_[0] = 0;
  camera_center_velodyne_[1] = 0;
  camera_center_velodyne_[2] = 100;
}

void GLFusionVisualizer::set_main_car_points() {
  main_car_.resize(4);
  main_car_[0][0] = 2.0;
  main_car_[0][1] = 1.0;
  main_car_[0][2] = 0.0;

  main_car_[1][0] = -2.0;
  main_car_[1][1] = 1.0;
  main_car_[1][2] = 0.0;

  main_car_[2][0] = -2.0;
  main_car_[2][1] = -1.0;
  main_car_[2][2] = 0.0;

  main_car_[3][0] = 2.0;
  main_car_[3][1] = -1.0;
  main_car_[3][2] = 0.0;
}

void GLFusionVisualizer::update_camera_system(FrameContent *content) {
  Eigen::Matrix4d pose_v2w = Eigen::Matrix4d::Identity();
  Eigen::Vector4d camera_center_w(camera_center_velodyne_[0],
                                  camera_center_velodyne_[1],
                                  camera_center_velodyne_[2], 0);
  camera_center_w = pose_v2w * camera_center_w;
  camera_center_world_[0] = camera_center_w[0];
  camera_center_world_[1] = camera_center_w[1];
  camera_center_world_[2] = camera_center_w[2];

  Eigen::Vector4d view_point_w(view_point_velodyne_[0], view_point_velodyne_[1],
                               view_point_velodyne_[2], 0);
  view_point_w = pose_v2w * view_point_w;
  view_point_world_[0] = view_point_w[0];
  view_point_world_[1] = view_point_w[1];
  view_point_world_[2] = view_point_w[2];

  Eigen::Vector4d up_w(up_velodyne_[0], up_velodyne_[1], up_velodyne_[2], 0);

  up_w = pose_v2w * up_w;
  up_world_[0] = up_w[0];
  up_world_[1] = up_w[1];
  up_world_[2] = up_w[2];

  Eigen::Vector4d fd_w(forward_velodyne_[0], forward_velodyne_[1],
                       forward_velodyne_[2], 0);
  fd_w = pose_v2w * fd_w;
  forward_world_[0] = fd_w[0];
  forward_world_[1] = fd_w[1];
  forward_world_[2] = fd_w[2];

  main_car_world_.resize(4);
  for (size_t i = 0; i < 4; ++i) {
    Eigen::Vector4d main_car_w(main_car_[i][0], main_car_[i][1],
                               main_car_[i][2], 0.0);

    main_car_w = pose_v2w * main_car_w;
    main_car_world_[i][0] = main_car_w[0];
    main_car_world_[i][1] = main_car_w[1];
    main_car_world_[i][2] = main_car_w[2];
  }
}

}  // namespace lowcostvisualizer
}  // namespace perception
}  // namespace apollo
