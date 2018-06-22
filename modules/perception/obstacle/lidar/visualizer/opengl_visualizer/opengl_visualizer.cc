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

#include "modules/perception/obstacle/lidar/visualizer/opengl_visualizer/opengl_visualizer.h"

#include "modules/common/log.h"
#include "modules/perception/common/geometry_util.h"

namespace apollo {
namespace perception {

OpenglVisualizer::OpenglVisualizer() : name_("OpenglVisualizer") {}

bool OpenglVisualizer::Init() {
  opengl_vs_ = boost::shared_ptr<GLFWViewer>(new GLFWViewer());

  if (opengl_vs_ == nullptr) {
    AERROR << "Failed to create opengl viewer";
    return false;
  }

  if (!opengl_vs_->Initialize()) {
    AERROR << "Failed to initialize opengl viewer";
    return false;
  }

  SetSize(2000, 1400);
  SetBackgroundColor(0.05, 0.05, 0.05, 0.0f);
  SetVelodyneHeight(2.5);
  SetMainCarPoints();
  SetCameraPosition();
  AINFO << "Initialize OpenglVisualizer successfully";
  return true;
}

void OpenglVisualizer::Render(const FrameContent &content) {
  opengl_vs_->SetCameraPara(
      Eigen::Vector3d(camera_center_world_.x, camera_center_world_.y,
                      camera_center_world_.z),
      Eigen::Vector3d(view_point_world_.x, view_point_world_.y,
                      view_point_world_.z),
      Eigen::Vector3d(up_world_.x, up_world_.y, up_world_.z));
  opengl_vs_->SetForwardDir(
      Eigen::Vector3d(forward_world_.x, forward_world_.y, forward_world_.z));
  opengl_vs_->SetFrameContent(content);
  opengl_vs_->SpinOnce();
}

void OpenglVisualizer::SetSize(int w, int h) { opengl_vs_->SetSize(w, h); }

void OpenglVisualizer::SetBackgroundColor(float r, float g, float b, float a) {
  opengl_vs_->SetBackgroundColor(Eigen::Vector3d(r, g, b));
}

void OpenglVisualizer::SetVelodyneHeight(float h) { velodyne_height_ = h; }

void OpenglVisualizer::SetCameraPosition() {
  up_velodyne_.x = 0;
  up_velodyne_.y = 1;
  up_velodyne_.z = 0;
  forward_velodyne_.x = 1;
  forward_velodyne_.y = 0;
  forward_velodyne_.z = 0;
  view_point_velodyne_.x = 0;
  view_point_velodyne_.y = 0;
  view_point_velodyne_.z = 0;
  camera_center_velodyne_.x = 0;
  camera_center_velodyne_.y = 0;
  camera_center_velodyne_.z = 100;
}

void OpenglVisualizer::SetMainCarPoints() {
  main_car_points_velodyne_.resize(9);
  main_car_points_velodyne_.at(0).x = 0;
  main_car_points_velodyne_.at(0).y = 0;
  main_car_points_velodyne_.at(0).z = 0;

  main_car_points_velodyne_.at(1).x = 0;
  main_car_points_velodyne_.at(1).y = 0;
  main_car_points_velodyne_.at(1).z = -velodyne_height_;
  main_car_points_velodyne_.at(2).x = 3;
  main_car_points_velodyne_.at(2).y = 0;
  main_car_points_velodyne_.at(2).z = -velodyne_height_;

  main_car_points_velodyne_.at(3).x = 2.5;
  main_car_points_velodyne_.at(3).y = 1.0;
  main_car_points_velodyne_.at(3).z = -velodyne_height_;
  main_car_points_velodyne_.at(4).x = 2.5;
  main_car_points_velodyne_.at(4).y = -1.0;
  main_car_points_velodyne_.at(4).z = -velodyne_height_;
  main_car_points_velodyne_.at(5).x = -2.5;
  main_car_points_velodyne_.at(5).y = -1.0;
  main_car_points_velodyne_.at(5).z = -velodyne_height_;
  main_car_points_velodyne_.at(6).x = -2.5;
  main_car_points_velodyne_.at(6).y = 1.0;
  main_car_points_velodyne_.at(6).z = -velodyne_height_;

  main_car_points_velodyne_.at(7).x = 0;
  main_car_points_velodyne_.at(7).y = 0;
  main_car_points_velodyne_.at(7).z = 160;
  main_car_points_velodyne_.at(8).x = -40;
  main_car_points_velodyne_.at(8).y = 0;
  main_car_points_velodyne_.at(8).z = 50;
}

void OpenglVisualizer::UpdateCameraSystem(FrameContent *content) {
  Eigen::Matrix4d pose_v2w = content->GetPoseV2w();

  TransformPoint<pcl_util::Point>(camera_center_velodyne_, pose_v2w,
                                  &camera_center_world_);

  TransformPoint<pcl_util::Point>(view_point_velodyne_, pose_v2w,
                                  &view_point_world_);

  TransformPointCloud<pcl_util::Point>(main_car_points_velodyne_, pose_v2w,
                                       &main_car_points_world_);

  Eigen::Vector4d up_w(up_velodyne_.x, up_velodyne_.y, up_velodyne_.z, 0);

  up_w = pose_v2w * up_w;
  up_world_.x = up_w[0];
  up_world_.y = up_w[1];
  up_world_.z = up_w[2];

  Eigen::Vector4d fd_w(forward_velodyne_.x, forward_velodyne_.y,
                       forward_velodyne_.z, 0);
  fd_w = pose_v2w * fd_w;
  forward_world_.x = fd_w[0];
  forward_world_.y = fd_w[1];
  forward_world_.z = fd_w[2];
}

}  // namespace perception
}  // namespace apollo
