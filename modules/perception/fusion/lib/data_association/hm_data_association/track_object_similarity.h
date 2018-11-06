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

#include <vector>

#include "modules/perception/fusion/base/sensor_object.h"
#include "modules/perception/fusion/lib/data_association/hm_data_association/chi_squared_cdf_1_0.0500_0.999900.h"
#include "modules/perception/fusion/lib/data_association/hm_data_association/chi_squared_cdf_2_0.0500_0.999900.h"
#include "modules/perception/fusion/lib/data_association/hm_data_association/probabilities.h"
#include "modules/perception/fusion/lib/data_association/hm_data_association/projection_cache.h"

namespace apollo {
namespace perception {
namespace fusion {

struct XSimilarityParams {
  float welsh_loss_thresh_ = 0.5f;
  float welsh_loss_scale_ = 0.3f;
  float scale_positive_max_p_ = 0.9f;
  float scale_positive_th_p_ = 0.5f;
};  // struct XSimilarityParams
struct YSimilarityParams {
  float smooth_factor_ = 0.3f;
  float diff_std_dev_ = 0.2f;
  float bounded_scale_positive_max_p_ = 0.6f;
  float bounded_scale_positive_min_p_ = 0.5f;
};  // struct YSimilarityParams
struct HSimilarityParams {
  float initial_similarity_ = 0.5f;
  float diff_std_dev_ = 0.1f;
  float scale_positive_max_p_ = 0.9f;
  float scale_positive_th_p_ = 0.5f;
};  // struct HSimilarityParams
struct WSimilarityParams {
  float diff_std_dev_ = 0.1f;
  float bounded_scale_positive_max_p_ = 0.7f;
  float bounded_scale_positive_min_p_ = 0.5f;
};  // struct WSimilarityParams
struct LocSimilarityParams {
  float welsh_loss_thresh_ = 0.05f;
  float welsh_loss_scale_ = 0.05f;
  float scale_positive_max_p_ = 0.7f;
  float scale_positive_th_p_ = 0.5f;
};  // struct LocSimilarityParams

// @brief: calculate the location similarity btween cloud and camera box
// @return the localtion similarity which belongs to [0, 1]. When cloud's
// location close to the camera box's, the similarity would close to 1.
// Otherwise, it would close to 0.
// @NOTE: original method name is compute_pts_box_dist_score
double ComputePtsBoxLocationSimilarity(const ProjectionCachePtr& cache,
                                       const ProjectionCacheObject* object,
                                       const base::BBox2DF& camera_bbox);
// @brief: calculate the shape similarity between cloud and camera box
// @return the shape similarity which belongs to [0, 1]. When cloud's shape
// close to the camera box's, the simialrity would close to 1. Otherwise,
// it would close to 0.
// @NOTE: original method name is compute_pts_box_shape_score
double ComputePtsBoxShapeSimilarity(const ProjectionCachePtr& cache,
                                    const ProjectionCacheObject* object,
                                    const base::BBox2DF& camera_bbox);
// @brief: calculate the similarity between cloud and camera box
// @return the similarity which belongs to [0, 1].
// @key idea:
// 1. compute location similarity and shape similarity
// 2. fuse the two similarity calculated above
// @NOTE: original method name is compute_pts_box_score
double ComputePtsBoxSimilarity(const ProjectionCachePtr& cache,
                               const ProjectionCacheObject* object,
                               const base::BBox2DF& camera_bbox);
// @brief: calculate the x/y/h/w/3d similarity between radar and camera
// @return the similarity which belongs to [0, 1].
// @key idea:
// 1. compute the difference on x/y/h/w/3d
// 2. compute similarity according to the WelshVarLoss/ChiSquareProb
// 3. scale the similarity above
double ComputeRadarCameraXSimilarity(const double velo_ct_x,
                                     const double camera_ct_x,
                                     const double size_x,
                                     const XSimilarityParams& params);
double ComputeRadarCameraYSimilarity(const double velo_ct_y,
                                     const double camera_ct_y,
                                     const double size_y,
                                     const YSimilarityParams& params);
double ComputeRadarCameraHSimilarity(
    const SensorObjectConstPtr& radar, const SensorObjectConstPtr& camera,
    const double size_y,
    const std::vector<Eigen::Vector2d>& radar_box2d_vertices,
    const HSimilarityParams& params);
double ComputeRadarCameraWSimilarity(
    const SensorObjectConstPtr& radar, const double width, const double size_x,
    const std::vector<Eigen::Vector2d>& radar_box2d_vertices,
    const WSimilarityParams& params);
double ComputeRadarCameraLocSimilarity(const Eigen::Vector3d& radar_ct,
                                       const SensorObjectConstPtr& camera,
                                       const Eigen::Matrix4d& world2camera_pose,
                                       const LocSimilarityParams& params);
double ComputeRadarCameraVelocitySimilarity(const SensorObjectConstPtr& radar,
                                            const SensorObjectConstPtr& camera);

}  // namespace fusion
}  // namespace perception
}  // namespace apollo
