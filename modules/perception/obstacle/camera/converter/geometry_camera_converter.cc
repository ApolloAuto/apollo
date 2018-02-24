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

// Convert 2D detections into 3D objects

#include "modules/perception/obstacle/camera/converter/geometry_camera_converter.h"

namespace apollo {
namespace perception {

bool GeometryCameraConverter::Init() {
  // ConfigManager *config_manager = ConfigManager::instance();

  // if (!load_camera_intrinsics(FLAGS_onsemi_obstacle_intrinsics)) {
  //   XLOG(ERROR) << "camera intrinsics not found: "
  //               << FLAGS_onsemi_obstacle_intrinsics;
  //   return false;
  // }

  return true;
}

bool GeometryCameraConverter::Convert(std::vector<VisualObjectPtr> *objects) {
  if (!objects) return false;

  for (auto &obj : *objects) {
    // TODO(cheni-kuei) Physical Size sanity check based on type
    float deg_alpha = obj->alpha * 180.0f / M_PI;

    Eigen::Vector2f upper_left(obj->upper_left.x(), obj->upper_left.y());
    Eigen::Vector2f lower_right(obj->lower_right.x(), obj->lower_right.y());
    float distance_w = 0.0;
    float distance_h = 0.0;
    Eigen::Vector2f mass_center_pixel = Eigen::Vector2f::Zero();
    ConvertSingle(obj->height, obj->width, obj->length, deg_alpha, upper_left,
                  lower_right, &distance_w, &distance_h, &mass_center_pixel);

    // TODO(cheni-kuei) Choose distance_h or distance_w, considering
    // truncation, type, longer side, or strategy
    obj->distance = distance_h;  // distance_h, distance_w
    Eigen::Vector3f camera_ray = camera_model_.unproject(mass_center_pixel);

    // Angles
    float beta = std::atan2(camera_ray.x(), camera_ray.z());
    // double beta = std::atan2(camera_ray.z(), camera_ray.x());
    float theta = obj->alpha + beta;
    if (theta > M_PI) {
      theta -= 2 * M_PI;
    } else if (theta < -M_PI) {
      theta += 2 * M_PI;
    }
    obj->theta = theta;

    // Center (3D Mass Center of 3D BBox)
    float scale = obj->distance / sqrt(camera_ray.x() * camera_ray.x() +
                                       camera_ray.y() * camera_ray.y() +
                                       camera_ray.z() * camera_ray.z());
    obj->center = Eigen::Vector3f(
        camera_ray.x() * scale, camera_ray.y() * scale, camera_ray.z() * scale);

    if (debug_) {
      //  obj->pts8.clear();
      //  for (size_t i = 0; i < corners_.size(); ++i) {
      //      Eigen::Vector2d point_2d =
      //      camera_model_.project(corners_[i] + obj->center);
      //      obj->pts8.push_back(static_cast<float>(point_2d.x()));
      //      obj->pts8.push_back(static_cast<float>(point_2d.y()));
      //  }
      //
      //  // 3D center projection
      //  Eigen::Vector2d point_2d = camera_model_.project(obj->center);
      //  obj->pts8.push_back(static_cast<float>(point_2d.x()));
      //  obj->pts8.push_back(static_cast<float>(point_2d.y()));
    }
  }

  return true;
}

void GeometryCameraConverter::SetDebug(bool flag) { debug_ = flag; }

std::string GeometryCameraConverter::Name() const {
  return "GeometryCameraConverter";
}

bool GeometryCameraConverter::LoadCameraIntrinsics(
    const std::string &file_path) {
  YAML::Node node = YAML::LoadFile(file_path);

  Eigen::Matrix3f intrinsic_k;
  for (int i = 0; i < 3; ++i) {
    for (int j = 0; j < 3; ++j) {
      int index = i * 3 + j;
      intrinsic_k(i, j) = node["K"][index].as<float>();
    }
  }

  float height = node["height"].as<float>();
  float width = node["width"].as<float>();
  camera_model_.set(intrinsic_k, width, height);

  return true;
}

bool GeometryCameraConverter::ConvertSingle(
    const float &h, const float &w, const float &l, const float &alpha_deg,
    const Eigen::Vector2f &upper_left, const Eigen::Vector2f &lower_right,
    float *distance_w, float *distance_h, Eigen::Vector2f *mass_center_pixel) {
  // Target Goals: Projection target
  int pixel_width = static_cast<int>(lower_right.x() - upper_left.x());
  int pixel_height = static_cast<int>(lower_right.y() - upper_left.y());

  // Target Goals: Box center pixel
  Eigen::Matrix<float, 2, 1> box_center_pixel;
  box_center_pixel.x() = (lower_right.x() + upper_left.x()) / 2.0f;
  box_center_pixel.y() = (lower_right.y() + upper_left.y()) / 2.0f;

  // Generate alpha rotated 3D template here. Corners in Camera space:
  // Bottom: FL, FR, RR, RL => Top: FL, FR, RR, RL
  float deg_alpha = alpha_deg;
  float h_half = h / 2.0f;
  float w_half = w / 2.0f;
  float l_half = l / 2.0f;

  std::vector<Eigen::Vector3f> corners;
  corners.resize(8);
  corners[0] = Eigen::Vector3f(l_half, h_half, w_half);
  corners[1] = Eigen::Vector3f(l_half, h_half, -w_half);
  corners[2] = Eigen::Vector3f(-l_half, h_half, -w_half);
  corners[3] = Eigen::Vector3f(-l_half, h_half, w_half);
  corners[4] = Eigen::Vector3f(l_half, -h_half, w_half);
  corners[5] = Eigen::Vector3f(l_half, -h_half, -w_half);
  corners[6] = Eigen::Vector3f(-l_half, -h_half, -w_half);
  corners[7] = Eigen::Vector3f(-l_half, -h_half, w_half);
  Rotate(deg_alpha, &corners);
  corners_ = corners;

  // Try to get an initial Mass center pixel and vector
  Eigen::Matrix<float, 3, 1> middle_v(0.0f, 0.0f, 20.0f);
  Eigen::Matrix<float, 2, 1> center_pixel = camera_model_.project(middle_v);

  float max_pixel_x = std::numeric_limits<float>::min();
  float min_pixel_x = std::numeric_limits<float>::max();
  float max_pixel_y = std::numeric_limits<float>::min();
  float min_pixel_y = std::numeric_limits<float>::max();
  for (size_t i = 0; i < corners.size(); ++i) {
    Eigen::Vector2f point_2d = camera_model_.project(corners[i] + middle_v);
    min_pixel_x = std::min(min_pixel_x, point_2d.x());
    max_pixel_x = std::max(max_pixel_x, point_2d.x());
    min_pixel_y = std::min(min_pixel_y, point_2d.y());
    max_pixel_y = std::max(max_pixel_y, point_2d.y());
  }
  float relative_x =
      (center_pixel.x() - min_pixel_x) / (max_pixel_x - min_pixel_x);
  float relative_y =
      (center_pixel.y() - min_pixel_y) / (max_pixel_y - min_pixel_y);

  mass_center_pixel->x() =
      (lower_right.x() - upper_left.x()) * relative_x + upper_left.x();
  mass_center_pixel->y() =
      (lower_right.y() - upper_left.y()) * relative_y + upper_left.y();
  Eigen::Matrix<float, 3, 1> mass_center_v =
      camera_model_.unproject(*mass_center_pixel);
  mass_center_v = MakeUnit(mass_center_v);

  // Binary search
  *distance_w = SearchDistance(pixel_width, true, mass_center_v);
  *distance_h = SearchDistance(pixel_height, false, mass_center_v);

  for (size_t i = 0; i < 2; ++i) {
    // Mass center search
    SearchCenterDirection(box_center_pixel, *distance_h, &mass_center_v,
                          mass_center_pixel);

    // Binary search
    *distance_w = SearchDistance(pixel_width, true, mass_center_v);
    *distance_h = SearchDistance(pixel_height, false, mass_center_v);
  }

  return true;
}

void GeometryCameraConverter::Rotate(
    const float &alpha_deg, std::vector<Eigen::Vector3f> *corners) const {
  Eigen::AngleAxisf yaw(alpha_deg / 180.0f * M_PI, Eigen::Vector3f::UnitY());
  Eigen::AngleAxisf pitch(0.0, Eigen::Vector3f::UnitX());
  Eigen::AngleAxisf roll(0.0, Eigen::Vector3f::UnitZ());
  Eigen::Matrix3f rotation = yaw.toRotationMatrix() * pitch.toRotationMatrix() *
                             roll.toRotationMatrix();

  Eigen::Matrix4f transform;
  transform.setIdentity();
  transform.block(0, 0, 3, 3) = rotation;

  for (auto &corner : *corners) {
    Eigen::Vector4f temp(corner.x(), corner.y(), corner.z(), 1.0f);
    temp = transform * temp;
    corner = Eigen::Vector3f(temp.x(), temp.y(), temp.z());
  }
}

float GeometryCameraConverter::SearchDistance(
    const int &pixel_length, const bool &use_width,
    const Eigen::Matrix<float, 3, 1> &mass_center_v) const {
  float close_d = 0.1f;
  float far_d = 200.0f;
  float curr_d = 0.0f;
  int depth = 0;
  while (close_d <= far_d && depth < kMaxDistanceSearchDepth_) {
    curr_d = (far_d + close_d) / 2.0f;
    Eigen::Vector3f curr_p = mass_center_v * curr_d;

    float min_p = std::numeric_limits<float>::max();
    float max_p = 0.0f;
    for (size_t i = 0; i < corners_.size(); ++i) {
      Eigen::Vector2f point_2d = camera_model_.project(corners_[i] + curr_p);

      float curr_pixel = 0.0f;
      if (use_width) {
        curr_pixel = point_2d.x();
      } else {
        curr_pixel = point_2d.y();
      }

      min_p = std::min(min_p, curr_pixel);
      max_p = std::max(max_p, curr_pixel);
    }

    int curr_pixel_length = static_cast<int>(max_p - min_p);
    if (curr_pixel_length == pixel_length) {
      break;
    } else if (pixel_length < curr_pixel_length) {
      close_d = curr_d + 0.01f;
    } else {  // pixel_length > curr_pixel_length
      far_d = curr_d - 0.01f;
    }

    // Early break for 0.01m accuracy
    float next_d = (far_d + close_d) / 2.0f;
    if (std::abs(next_d - curr_d) < 0.01f) {
      break;
    }

    ++depth;
  }

  return curr_d;
}

void GeometryCameraConverter::SearchCenterDirection(
    const Eigen::Matrix<float, 2, 1> &box_center_pixel, const float &curr_d,
    Eigen::Matrix<float, 3, 1> *mass_center_v,
    Eigen::Matrix<float, 2, 1> *mass_center_pixel) const {
  int depth = 0;
  while (depth < kMaxCenterDirectionSearchDepth_) {
    Eigen::Matrix<float, 3, 1> new_center_v = *mass_center_v * curr_d;

    float max_pixel_x = std::numeric_limits<float>::min();
    float min_pixel_x = std::numeric_limits<float>::max();
    float max_pixel_y = std::numeric_limits<float>::min();
    float min_pixel_y = std::numeric_limits<float>::max();
    for (size_t i = 0; i < corners_.size(); ++i) {
      Eigen::Vector2f point_2d =
          camera_model_.project(corners_[i] + new_center_v);
      min_pixel_x = std::min(min_pixel_x, point_2d.x());
      max_pixel_x = std::max(max_pixel_x, point_2d.x());
      min_pixel_y = std::min(min_pixel_y, point_2d.y());
      max_pixel_y = std::max(max_pixel_y, point_2d.y());
    }

    Eigen::Matrix<float, 2, 1> current_box_center_pixel;
    current_box_center_pixel.x() = (max_pixel_x + min_pixel_x) / 2.0;
    current_box_center_pixel.y() = (max_pixel_y + min_pixel_y) / 2.0;

    // Update mass center
    *mass_center_pixel += box_center_pixel - current_box_center_pixel;
    *mass_center_v = camera_model_.unproject(*mass_center_pixel);
    *mass_center_v = MakeUnit(*mass_center_v);

    if (std::abs(mass_center_pixel->x() - box_center_pixel.x()) < 1.0 &&
        std::abs(mass_center_pixel->y() - box_center_pixel.y()) < 1.0) {
      break;
    }

    ++depth;
  }

  return;
}

Eigen::Matrix<float, 3, 1> GeometryCameraConverter::MakeUnit(
    const Eigen::Matrix<float, 3, 1> &v) const {
  Eigen::Matrix<float, 3, 1> unit_v = v;
  float to_unit_scale =
      std::sqrt(unit_v.x() * unit_v.x() + unit_v.y() * unit_v.y() +
                unit_v.z() * unit_v.z());
  unit_v /= to_unit_scale;
  return unit_v;
}

}  // namespace perception
}  // namespace apollo
