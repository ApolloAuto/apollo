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
#include "modules/perception/multi_sensor_fusion/fusion/data_association/hm_data_association/track_object_similarity.h"

#include <algorithm>
#include <limits>

#include "modules/perception/multi_sensor_fusion/base/fusion_log.h"
#include "modules/perception/multi_sensor_fusion/common/camera_util.h"

namespace {
constexpr float kFloatEpsilon = std::numeric_limits<float>::epsilon();
}  // namespace

namespace apollo {
namespace perception {
namespace fusion {

// @brief: calculate the location similarity between cloud and camera box
// @return the location similarity which belongs to [0, 1].
// @key idea:
// 1. calculate the mean x y pixel distance
// 2. normalize mean pixel distance on the size of box and std of x/y
// 3. generate location similarity from Chi-Squared distribution
// @NOTE: original method name is compute_pts_box_dist_score
double ComputePtsBoxLocationSimilarity(const ProjectionCachePtr& cache,
                                       const ProjectionCacheObject* object,
                                       const base::BBox2DF& camera_bbox) {
  static const double min_p = 1e-6;
  static const double max_p = 1 - 1e-6;
  double x_std_dev = 0.4;
  double y_std_dev = 0.5;
  size_t check_augmented_iou_minimum_pts_num = 20;
  float augmented_buffer = 25.0f;
  if (object->Empty()) {
    ADEBUG << "cache object is empty!";
    return min_p;
  }
  Eigen::Vector2d mean_pixel_dist(0.0, 0.0);
  // calculate mean x y pixel distance
  const size_t start_ind = object->GetStartInd();
  const size_t end_ind = object->GetEndInd();
  if (end_ind - start_ind >= check_augmented_iou_minimum_pts_num) {
    base::BBox2DF velo_bbox = object->GetBox();
    float augmented_iou =
        CalculateAugmentedIOUBBox(velo_bbox, camera_bbox, augmented_buffer);
    if (augmented_iou < kFloatEpsilon) {
      ADEBUG << "augmented iou is empty!";
      return min_p;
    }
  }
  for (size_t i = start_ind; i < end_ind; ++i) {
    auto* velo_pt2d = cache->GetPoint2d(i);
    if (velo_pt2d == nullptr) {
      AERROR << "query pt from projection cache failed!";
      continue;
    }
    if (velo_pt2d->x() >= camera_bbox.xmin &&
        velo_pt2d->x() < camera_bbox.xmax &&
        velo_pt2d->y() >= camera_bbox.ymin &&
        velo_pt2d->y() < camera_bbox.ymax) {
      continue;
    }
    Eigen::Vector2d diff;
    diff.x() = std::max(0.0, camera_bbox.xmin - velo_pt2d->x());
    diff.x() = std::max(diff.x(), velo_pt2d->x() - camera_bbox.xmax);
    diff.y() = std::max(0.0, camera_bbox.ymin - velo_pt2d->y());
    diff.y() = std::max(diff.y(), velo_pt2d->y() - camera_bbox.ymax);
    mean_pixel_dist += diff;
  }
  mean_pixel_dist /= static_cast<double>(object->Size());
  ADEBUG << "mean_pixel_dist is: " << mean_pixel_dist;
  // normalize according to box size
  Eigen::Vector2d box_size = Eigen::Vector2d(
      camera_bbox.xmax - camera_bbox.xmin, camera_bbox.ymax - camera_bbox.ymin);
  mean_pixel_dist.array() /= box_size.array();
  // assuming the normalized distance obeys gauss distribution
  double square_norm_mean_pixel_dist =
      mean_pixel_dist.x() * mean_pixel_dist.x() / x_std_dev / x_std_dev +
      mean_pixel_dist.y() * mean_pixel_dist.y() / y_std_dev / y_std_dev;
  // use chi-square distribution. Cauchy may be more reasonable.
  double location_similarity =
      1 - ChiSquaredCdf2TableFun(square_norm_mean_pixel_dist);
  // for numerical stability
  location_similarity = std::max(min_p, std::min(max_p, location_similarity));
  return location_similarity;
}
// @brief: calculate the shape similarity between cloud and camera box
// @return the shape similarity which belongs to [0, 1].
// @key idea:
// 1. calculate box size diff between velo box and camera box
// 2. normalize box size diff according to the std of x/y
// 3. generate shape similarity from Chi-Squared distribution
// @NOTE: original method name is compute_pts_box_shape_score
double ComputePtsBoxShapeSimilarity(const ProjectionCachePtr& cache,
                                    const ProjectionCacheObject* object,
                                    const base::BBox2DF& camera_bbox) {
  static const double min_p = 1e-3;
  static const double max_p = 1 - 1e-3;
  double x_std_dev = 0.3;
  double y_std_dev = 0.4;
  if (object->Empty()) {
    return min_p;
  }
  // compute 2d bbox size of camera & velo
  Eigen::Vector2d camera_box_size = Eigen::Vector2d(
      camera_bbox.xmax - camera_bbox.xmin, camera_bbox.ymax - camera_bbox.ymin);
  // handled one point case
  base::BBox2DF velo_projection_bbox = object->GetBox();
  Eigen::Vector2d velo_box_size = camera_box_size / 10;
  velo_box_size.x() =
      std::max(static_cast<float>(velo_box_size.x()),
               velo_projection_bbox.xmax - velo_projection_bbox.xmin);
  velo_box_size.y() =
      std::max(static_cast<float>(velo_box_size.y()),
               velo_projection_bbox.ymax - velo_projection_bbox.ymin);
  // compute normalized box size diff
  Eigen::Vector2d mean_box_size = (camera_box_size + velo_box_size) / 2;
  Eigen::Vector2d box_size_diff =
      (velo_box_size - camera_box_size).array() / mean_box_size.array();
  // assuming the normalized distance obeys gauss distribution
  double square_norm_box_size_diff =
      box_size_diff.x() * box_size_diff.x() / x_std_dev / x_std_dev +
      box_size_diff.y() * box_size_diff.y() / y_std_dev / y_std_dev;
  ADEBUG << "camera_box_size@(" << camera_box_size(0) << ","
         << camera_box_size(1) << "); "
         << "velo_box_size@(" << velo_box_size.x() << "," << velo_box_size.y()
         << "); "
         << "box_size_diff@(" << box_size_diff.x() << "," << box_size_diff.y()
         << "); "
         << "square_norm_box_size_diff@" << square_norm_box_size_diff;
  // use chi-square distribution. Cauchy may be more reasonable.
  double shape_similarity =
      1 - ChiSquaredCdf2TableFun(square_norm_box_size_diff);
  // for numerical stability
  shape_similarity = std::max(min_p, std::min(max_p, shape_similarity));
  return shape_similarity;
}
// @brief: calculate the similarity between cloud and camera box
// @return the similarity which belongs to [0, 1].
// @key idea:
// 1. compute location similarity and shape similarity
// 2. fuse the two similarity above
// @NOTE: original method name is compute_pts_box_score
double ComputePtsBoxSimilarity(const ProjectionCachePtr& cache,
                               const ProjectionCacheObject* object,
                               const base::BBox2DF& camera_bbox) {
  double location_similarity =
      ComputePtsBoxLocationSimilarity(cache, object, camera_bbox);
  double shape_similarity =
      ComputePtsBoxShapeSimilarity(cache, object, camera_bbox);
  double fused_similarity =
      FuseTwoProbabilities(location_similarity, shape_similarity);
  ADEBUG << "fused_similarity@" << fused_similarity << ", location_similarity@"
         << location_similarity << ", shape_similarity@" << shape_similarity;
  return fused_similarity;
}
// @brief: calculate the x/y/h similarity between radar and camera
// @return the similarity which belongs to [0, 1].
// @key idea:
// 1. compute the difference on x/y/h
// 2. compute similarity according to the WelshVarLoss/ChiSquareProb
// 3. scale the similarity above
double ComputeRadarCameraXSimilarity(const double velo_ct_x,
                                     const double camera_ct_x,
                                     const double size_x,
                                     const XSimilarityParams& params) {
  double x_diff = std::abs(velo_ct_x - camera_ct_x) / size_x;
  double x_similarity = WelshVarLossFun(x_diff, params.welsh_loss_thresh_,
                                        params.welsh_loss_scale_);
  x_similarity = ScalePositiveProbability(
      x_similarity, params.scale_positive_max_p_, params.scale_positive_th_p_);
  return x_similarity;
}
double ComputeRadarCameraYSimilarity(const double velo_ct_y,
                                     const double camera_ct_y,
                                     const double size_y,
                                     const YSimilarityParams& params) {
  // double y_diff =
  //     std::abs(velo_ct_y - camera_ct_y + size_y * params.smooth_factor_) /
  //     size_y;

  double y_diff = std::max(std::abs(velo_ct_y - camera_ct_y) -
                               size_y * params.smooth_factor_,
                           0.0) /
                  size_y;

  double normalized_y_diff =
      y_diff * y_diff / params.diff_std_dev_ / params.diff_std_dev_;
  double y_similarity = 1 - ChiSquaredCdf1TableFun(normalized_y_diff);
  y_similarity = BoundedScalePositiveProbability(
      y_similarity, params.bounded_scale_positive_max_p_,
      params.bounded_scale_positive_min_p_);
  return y_similarity;
}
double ComputeRadarCameraHSimilarity(
    const SensorObjectConstPtr& radar, const SensorObjectConstPtr& camera,
    const double size_y,
    const EigenVector<Eigen::Vector2d>& radar_box2d_vertices,
    const HSimilarityParams& params) {
  const double camera_height = camera->GetBaseObject()->size(2);
  double height_similarity = params.initial_similarity_;
  if (camera_height > kFloatEpsilon) {
    double min_height_diff = std::numeric_limits<double>::max();
    for (size_t i = 0; i < 4; ++i) {
      double img_car_height = std::abs(radar_box2d_vertices[i + 4].y() -
                                       radar_box2d_vertices[i].y());
      min_height_diff =
          std::min(std::abs(img_car_height - size_y), min_height_diff);
    }
    min_height_diff /= size_y;
    double normalized_min_height_diff = min_height_diff * min_height_diff /
                                        params.diff_std_dev_ /
                                        params.diff_std_dev_;
    height_similarity = 1 - ChiSquaredCdf1TableFun(normalized_min_height_diff);
    height_similarity = ScalePositiveProbability(height_similarity,
                                                 params.scale_positive_max_p_,
                                                 params.scale_positive_th_p_);
  }
  return height_similarity;
}
double ComputeRadarCameraWSimilarity(
    const SensorObjectConstPtr& radar, const double width, const double size_x,
    const EigenVector<Eigen::Vector2d>& radar_box2d_vertices,
    const WSimilarityParams& params) {
  std::vector<double> radar_box2d_xs = {
      radar_box2d_vertices[0].x(), radar_box2d_vertices[1].x(),
      radar_box2d_vertices[2].x(), radar_box2d_vertices[3].x()};
  for (double& x : radar_box2d_xs) {
    x = Bound(x, width, 0.0);
  }
  auto min_max_xs =
      std::minmax_element(radar_box2d_xs.begin(), radar_box2d_xs.end());
  double radar_box2d_width = *min_max_xs.second - *min_max_xs.first;
  double width_diff = std::abs(radar_box2d_width - size_x) / size_x;
  double normalized_width_diff =
      width_diff * width_diff / params.diff_std_dev_ / params.diff_std_dev_;
  double width_similarity = 1 - ChiSquaredCdf1TableFun(normalized_width_diff);
  width_similarity = BoundedScalePositiveProbability(
      width_similarity, params.bounded_scale_positive_max_p_,
      params.bounded_scale_positive_min_p_);
  return width_similarity;
}
double ComputeRadarCameraLocSimilarity(const Eigen::Vector3d& radar_ct,
                                       const SensorObjectConstPtr& camera,
                                       const Eigen::Matrix4d& world2camera_pose,
                                       const LocSimilarityParams& params) {
  Eigen::Vector3d camera_ct = camera->GetBaseObject()->center;
  Eigen::Vector3d camera_ct_c =
      (world2camera_pose * camera_ct.homogeneous()).head(3);
  double ct_diff = (radar_ct - camera_ct).norm();
  ct_diff = ct_diff / camera_ct_c.z();
  double ct_similarity = WelshVarLossFun(ct_diff, params.welsh_loss_thresh_,
                                         params.welsh_loss_scale_);
  ct_similarity = ScalePositiveProbability(
      ct_similarity, params.scale_positive_max_p_, params.scale_positive_th_p_);
  return ct_similarity;
}

double ComputeRadarCameraVelocitySimilarity(
    const SensorObjectConstPtr& radar, const SensorObjectConstPtr& camera) {
  Eigen::Vector3f radar_velocity = radar->GetBaseObject()->velocity;
  Eigen::Vector3f camera_velocity = camera->GetBaseObject()->velocity;

  float scalar_radar_velocity = radar_velocity.norm();
  float scalar_camera_velocity = camera_velocity.norm();
  if (std::max(scalar_radar_velocity, scalar_camera_velocity) > 2) {
    float diff_velocity = (radar_velocity - camera_velocity).norm() / 2;
    float diff_velocity_ratio =
        diff_velocity / std::max(scalar_camera_velocity, scalar_radar_velocity);
    const float velocity_std = 0.15f;
    const float max_velocity_p = 0.9f;
    const float th_velocity_p = 0.5f;
    float velocity_score = static_cast<float>(
        1 - ChiSquaredCdf1TableFun(diff_velocity_ratio * diff_velocity_ratio /
                                   velocity_std / velocity_std));
    velocity_score = static_cast<float>(ScalePositiveProbability(
        velocity_score, max_velocity_p, th_velocity_p));
    return velocity_score;
  } else {
    return 0.5;
  }
}

}  // namespace fusion
}  // namespace perception
}  // namespace apollo
