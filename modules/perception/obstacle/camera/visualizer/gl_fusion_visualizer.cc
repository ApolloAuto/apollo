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

#include "modules/common/log.h"
#include "modules/perception/obstacle/camera/visualizer/gl_fusion_visualizer.h"

namespace apollo {
namespace perception {
namespace lowcostvisualizer {

GLFusionVisualizer::GLFusionVisualizer() : _name("GLFusionVisualizer") {}

bool GLFusionVisualizer::init() {
  _opengl_vs = std::shared_ptr<GLFWFusionViewer>(new GLFWFusionViewer());

  if (_opengl_vs == nullptr) {
    AINFO << "Failed to create opengl viewer";
    return false;
  }

  if (_opengl_vs->initialize() == false) {
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
  _opengl_vs->set_camera_para(_camera_center_velodyne, _view_point_velodyne,
                              _up_velodyne);
  _opengl_vs->set_forward_dir(_forward_world);
  _opengl_vs->set_main_car(_main_car_world);
  _opengl_vs->set_frame_content(content);
  //    _opengl_vs->set_motion_content(_motion_buffer);
  _opengl_vs->spin_once();

  AINFO << "GLFusionVisualizer spin_once";
}

void GLFusionVisualizer::set_background_color(float r, float g, float b,
                                              float a) {
  _opengl_vs->set_background_color(Eigen::Vector3d(r, g, b));
}

void GLFusionVisualizer::set_camera_position() {
  _up_velodyne[0] = 0;
  _up_velodyne[1] = 1;
  _up_velodyne[2] = 0;
  _forward_velodyne[0] = 1;
  _forward_velodyne[1] = 0;
  _forward_velodyne[2] = 0;
  _view_point_velodyne[0] = 0;
  _view_point_velodyne[1] = 0;
  _view_point_velodyne[2] = 0;
  _camera_center_velodyne[0] = 0;
  _camera_center_velodyne[1] = 0;
  _camera_center_velodyne[2] = 100;
}

void GLFusionVisualizer::set_main_car_points() {
  _main_car.resize(4);
  _main_car[0][0] = 2.0;
  _main_car[0][1] = 1.0;
  _main_car[0][2] = 0.0;

  _main_car[1][0] = -2.0;
  _main_car[1][1] = 1.0;
  _main_car[1][2] = 0.0;

  _main_car[2][0] = -2.0;
  _main_car[2][1] = -1.0;
  _main_car[2][2] = 0.0;

  _main_car[3][0] = 2.0;
  _main_car[3][1] = -1.0;
  _main_car[3][2] = 0.0;
}

void GLFusionVisualizer::update_camera_system(FrameContent *content) {
  Eigen::Matrix4d pose_v2w = Eigen::Matrix4d::Identity();
  Eigen::Vector4d camera_center_w(_camera_center_velodyne[0],
                                  _camera_center_velodyne[1],
                                  _camera_center_velodyne[2], 0);
  camera_center_w = pose_v2w * camera_center_w;
  _camera_center_world[0] = camera_center_w[0];
  _camera_center_world[1] = camera_center_w[1];
  _camera_center_world[2] = camera_center_w[2];

  Eigen::Vector4d view_point_w(_view_point_velodyne[0], _view_point_velodyne[1],
                               _view_point_velodyne[2], 0);
  view_point_w = pose_v2w * view_point_w;
  _view_point_world[0] = view_point_w[0];
  _view_point_world[1] = view_point_w[1];
  _view_point_world[2] = view_point_w[2];

  Eigen::Vector4d up_w(_up_velodyne[0], _up_velodyne[1], _up_velodyne[2], 0);

  up_w = pose_v2w * up_w;
  _up_world[0] = up_w[0];
  _up_world[1] = up_w[1];
  _up_world[2] = up_w[2];

  Eigen::Vector4d fd_w(_forward_velodyne[0], _forward_velodyne[1],
                       _forward_velodyne[2], 0);
  fd_w = pose_v2w * fd_w;
  _forward_world[0] = fd_w[0];
  _forward_world[1] = fd_w[1];
  _forward_world[2] = fd_w[2];

  _main_car_world.resize(4);
  for (size_t i = 0; i < 4; ++i) {
    Eigen::Vector4d main_car_w(_main_car[i][0], _main_car[i][1],
                               _main_car[i][2], 0.0);

    main_car_w = pose_v2w * main_car_w;
    _main_car_world[i][0] = main_car_w[0];
    _main_car_world[i][1] = main_car_w[1];
    _main_car_world[i][2] = main_car_w[2];
  }
}

REGISTER_VISUALIZER(GLFusionVisualizer);

}  // namespace lowcostvisualizer
}  // namespace perception
}  // namespace apollo
