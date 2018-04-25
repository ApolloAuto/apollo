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
  ConfigManager *config_manager = ConfigManager::instance();

  const ModelConfig *model_config = config_manager->GetModelConfig(Name());
  if (model_config == nullptr) {
    AERROR << "Model config: " << Name() << " not found";
    return false;
  }

  std::string intrinsic_file_path = "";
  if (!model_config->GetValue("camera_intrinsic_file", &intrinsic_file_path)) {
    AERROR << "Failed to get camera intrinsics file path: " << Name();
    return false;
  }

  if (!LoadCameraIntrinsics(intrinsic_file_path)) {
    AERROR << "Failed to get camera intrinsics: " << intrinsic_file_path;
    return false;
  }

  return true;
}

bool GeometryCameraConverter::Convert(
    std::vector<std::shared_ptr<VisualObject>> *objects) {
  if (!objects) return false;

  for (auto &obj : *objects) {
    Eigen::Vector2f trunc_center_pixel = Eigen::Vector2f::Zero();
    CheckTruncation(obj, &trunc_center_pixel);
    CheckSizeSanity(obj);

    float deg_alpha = obj->alpha * 180.0f / M_PI;
    Eigen::Vector2f upper_left(obj->upper_left.x(), obj->upper_left.y());
    Eigen::Vector2f lower_right(obj->lower_right.x(), obj->lower_right.y());

    float distance = 0.0;
    Eigen::Vector2f mass_center_pixel = Eigen::Vector2f::Zero();
    if (obj->trunc_height < 0.25f) {
      // No truncation on 2D height
      ConvertSingle(obj->height, obj->width, obj->length, deg_alpha, upper_left,
                    lower_right, false, &distance, &mass_center_pixel);
    } else if (obj->trunc_width < 0.25f && obj->trunc_height > 0.25f) {
      // 2D height truncation and no width truncation
      ConvertSingle(obj->height, obj->width, obj->length, deg_alpha, upper_left,
                    lower_right, true, &distance, &mass_center_pixel);
    } else {
      // truncation on both sides
      // Give fix values for detected box with both side and bottom truncation
      distance = 10.0f;
      // Estimation of center pixel due to unknown truncation ratio
      mass_center_pixel = trunc_center_pixel;
    }

    obj->distance = distance;
    Eigen::Vector3f camera_ray = camera_model_.unproject(mass_center_pixel);
    DecideAngle(camera_ray, obj);

    // Center (3D Mass Center of 3D BBox)
    float scale = obj->distance / sqrt(camera_ray.x() * camera_ray.x() +
                                       camera_ray.y() * camera_ray.y() +
                                       camera_ray.z() * camera_ray.z());
    obj->center = camera_ray * scale;

    // Set 8 corner pixels
    SetBoxProjection(obj);
  }

  return true;
}

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

  Eigen::Matrix<float, 5, 1> intrinsic_d;
  for (int i = 0; i < 5; i++) {
    intrinsic_d(i, 0) = node["D"][i].as<float>();
  }

  float height = node["height"].as<float>();
  float width = node["width"].as<float>();
  camera_model_.set(intrinsic_k, width, height);
  camera_model_.set_distort_params(intrinsic_d);

  return true;
}

bool GeometryCameraConverter::ConvertSingle(
    const float &h, const float &w, const float &l, const float &alpha_deg,
    const Eigen::Vector2f &upper_left, const Eigen::Vector2f &lower_right,
    bool use_width, float *distance, Eigen::Vector2f *mass_center_pixel) {
  // Target Goals: Projection target
  int pixel_width = static_cast<int>(lower_right.x() - upper_left.x());
  int pixel_height = static_cast<int>(lower_right.y() - upper_left.y());
  int pixel_length = pixel_height;
  if (use_width) pixel_length = pixel_width;

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
  pixel_corners_.clear();
  pixel_corners_.resize(8);

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

  // Distance search
  *distance = SearchDistance(pixel_length, use_width, mass_center_v,
                             0.1f, 150.0f);
  for (size_t i = 0; i < 1; ++i) {
    // Mass center search
    SearchCenterDirection(box_center_pixel, *distance, &mass_center_v,
                          mass_center_pixel);
    // Distance search
    *distance = SearchDistance(pixel_length, use_width, mass_center_v,
                               0.9f * (*distance), 1.1f * (*distance));
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
    const Eigen::Matrix<float, 3, 1> &mass_center_v,
    float close_d, float far_d) {
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
      close_d = curr_d + 0.1f;
    } else {  // pixel_length > curr_pixel_length
      far_d = curr_d - 0.1f;
    }

    // Early break for 0.1m accuracy
    float next_d = (far_d + close_d) / 2.0f;
    if (std::abs(next_d - curr_d) < 0.1f) {
      break;
    }

    ++depth;
  }

  // Only copy the last projection out
  Eigen::Vector3f curr_p = mass_center_v * curr_d;
  for (size_t i = 0; i < corners_.size(); ++i) {
    Eigen::Vector2f point_2d = camera_model_.project(corners_[i] + curr_p);
    pixel_corners_[i] = point_2d;
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

void GeometryCameraConverter::CheckSizeSanity(
    std::shared_ptr<VisualObject> obj) const {
  if (obj->type == ObjectType::VEHICLE) {
    obj->length = std::max(obj->length, 3.6f);
    obj->width = std::max(obj->width, 1.6f);
    obj->height = std::max(obj->height, 1.5f);
  } else if (obj->type == ObjectType::PEDESTRIAN) {
    obj->length = std::max(obj->length, 0.5f);
    obj->width = std::max(obj->width, 0.5f);
    obj->height = std::max(obj->height, 1.7f);
  } else if (obj->type == ObjectType::BICYCLE) {
    obj->length = std::max(obj->length, 1.8f);
    obj->width = std::max(obj->width, 1.2f);
    obj->height = std::max(obj->height, 1.5f);
  } else {
    obj->length = std::max(obj->length, 0.5f);
    obj->width = std::max(obj->width, 0.5f);
    obj->height = std::max(obj->height, 1.5f);
  }
}

void GeometryCameraConverter::CheckTruncation(
    std::shared_ptr<VisualObject> obj,
    Eigen::Matrix<float, 2, 1> *trunc_center_pixel) const {
  auto width = camera_model_.get_width();
  auto height = camera_model_.get_height();

  // Ad-hoc 2D box truncation binary determination
  if (obj->upper_left.x() < 30.0f || width - 30.0f < obj->lower_right.x()) {
    obj->trunc_width = 0.5f;

    if (obj->upper_left.x() < 30.0f) {
      trunc_center_pixel->x() = obj->upper_left.x();
    } else {
      trunc_center_pixel->x() = obj->lower_right.x();
    }
  }

  if (obj->upper_left.y() < 30.0f || height - 30.0f < obj->lower_right.y()) {
    obj->trunc_height = 0.5f;
    trunc_center_pixel->x() =
        (obj->upper_left.x() + obj->lower_right.x()) / 2.0f;
  }

  trunc_center_pixel->y() = (obj->upper_left.y() + obj->lower_right.y()) / 2.0f;
}

float GeometryCameraConverter::DecideDistance(
    const float &distance_h, const float &distance_w,
    std::shared_ptr<VisualObject> obj) const {
  float distance = distance_h;
  return distance;
}

void GeometryCameraConverter::DecideAngle(
    const Eigen::Vector3f &camera_ray,
    std::shared_ptr<VisualObject> obj) const {
  float beta = std::atan2(camera_ray.x(), camera_ray.z());

  // Orientation is not reliable in these cases (DL model specific issue)
  if (obj->distance > 50.0f || obj->trunc_width > 0.25f) {
    obj->theta = -1.0f * M_PI_2;
    obj->alpha = obj->theta - beta;
    if (obj->alpha > M_PI) {
      obj->alpha -= 2 * M_PI;
    } else if (obj->alpha < -M_PI) {
      obj->alpha += 2 * M_PI;
    }
  } else {  // Normal cases
    float theta = obj->alpha + beta;
    if (theta > M_PI) {
      theta -= 2 * M_PI;
    } else if (theta < -M_PI) {
      theta += 2 * M_PI;
    }
    obj->theta = theta;
  }
}

void GeometryCameraConverter::SetBoxProjection(
    std::shared_ptr<VisualObject> obj) const {
  obj->pts8.resize(16);
  if (obj->trunc_width < 0.25f && obj->trunc_height < 0.25f) {  // No truncation
    for (int i = 0; i < 8; i++) {
      obj->pts8[i * 2] = pixel_corners_[i].x();
      obj->pts8[i * 2 + 1] = pixel_corners_[i].y();
    }
  }
}

}  // namespace perception
}  // namespace apollo
