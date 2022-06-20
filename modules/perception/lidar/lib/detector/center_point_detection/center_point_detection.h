/******************************************************************************
 * Copyright 2022 The Apollo Authors. All Rights Reserved.
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

#include <deque>
#include <memory>
#include <string>
#include <vector>

#include "paddle/include/paddle_inference_api.h"
#include "pcl/point_cloud.h"
#include "pcl/point_types.h"

#include "modules/perception/base/object.h"
#include "modules/perception/base/point_cloud.h"
#include "modules/perception/lidar/common/lidar_frame.h"
#include "modules/perception/lidar/lib/interface/base_lidar_detector.h"

namespace apollo {
namespace perception {
namespace lidar {

class CenterPointDetection : public BaseLidarDetector {
 public:
  CenterPointDetection();
  virtual ~CenterPointDetection() = default;

  bool Init(const LidarDetectorInitOptions &options =
                LidarDetectorInitOptions()) override;

  bool Detect(const LidarDetectorOptions &options, LidarFrame *frame) override;

  std::string Name() const override { return "CenterPointDetection"; }

 private:
  void CloudToArray(const base::PointFCloudPtr &pc_ptr, float *out_points_array,
                    float normalizing_factor);

  void FuseCloud(const base::PointFCloudPtr &out_cloud_ptr,
                 const std::deque<base::PointDCloudPtr> &fuse_clouds);

  bool Preprocess(const float *in_points_array, const int in_num_points,
                  std::vector<int> *voxels_shape,
                  std::vector<float> *voxels_data,
                  std::vector<int> *num_points_shape,
                  std::vector<int> *num_points_data,
                  std::vector<int> *coords_shape,
                  std::vector<int> *coords_data);

  std::vector<int> GenerateIndices(int start_index, int size, bool shuffle);

  bool insert_batch_idx_to_coords(const int *coords_ptr, const int voxel_num,
                                  const int batch_idx,
                                  int *coords_w_batch_idx_ptr);

  void DoInference(const float *in_points_array, const int in_num_points,
                   std::vector<float> *out_detections,
                   std::vector<int64_t> *out_labels,
                   std::vector<float> *out_scores);

  void run(paddle_infer::Predictor *predictor,
           const std::vector<int> &voxels_shape,
           const std::vector<float> &voxels_data,
           const std::vector<int> &coords_shape,
           const std::vector<int> &coords_data,
           const std::vector<int> &num_points_shape,
           const std::vector<int> &num_points_data,
           std::vector<float> *box3d_lidar, std::vector<int64_t> *label_preds,
           std::vector<float> *scores);

  bool hard_voxelize(const float point_cloud_range_x_min,
                     const float point_cloud_range_y_min,
                     const float point_cloud_range_z_min,
                     const float voxel_size_x, const float voxel_size_y,
                     const float voxel_size_z, const int grid_size_x,
                     const int grid_size_y, const int grid_size_z,
                     const int max_num_points_in_voxel, const int max_voxels,
                     const float *points, const int num_point_dim,
                     const int num_points, float *voxels, int *coords,
                     int *num_points_per_voxel, int *voxel_num);

  // std::vector<int> GenerateIndices(int start_index, int size, bool
  // shuffle);

  void GetObjects(std::vector<std::shared_ptr<base::Object>> *objects,
                  const Eigen::Affine3d &pose, std::vector<float> *detections,
                  std::vector<int64_t> *labels);

  void filter_score(const std::vector<float> *box3d_lidar,
                    const std::vector<int64_t> *label_preds,
                    const std::vector<float> *scores,
                    const float score_threshold,
                    std::vector<float> *box3d_lidar_final,
                    std::vector<int64_t> *label_preds_final);

  std::shared_ptr<paddle_infer::Predictor> create_predictor(
      const std::string &model_path, const std::string &params_path,
      const bool use_trt, const paddle::AnalysisConfig::Precision &precision);

  base::ObjectSubType GetObjectSubType(int label);

  // reference pointer of lidar frame
  LidarFrame *lidar_frame_ref_ = nullptr;
  std::shared_ptr<base::AttributePointCloud<base::PointF>> original_cloud_;
  std::shared_ptr<base::AttributePointCloud<base::PointD>>
      original_world_cloud_;

  std::deque<base::PointDCloudPtr> prev_world_clouds_;

  base::PointFCloudPtr cur_cloud_ptr_;

  // point cloud range
  float x_min_range_;
  float x_max_range_;
  float y_min_range_;
  float y_max_range_;
  float z_min_range_;
  float z_max_range_;

  // voxel params
  float x_voxel_size_;
  float y_voxel_size_;
  float z_voxel_size_;

  int max_num_points_in_voxel_;
  int num_point_feature_;
  int max_voxels_;

  // time statistics
  double downsample_time_ = 0.0;
  double fuse_time_ = 0.0;

  double shuffle_time_ = 0.0;
  double cloud_to_array_time_ = 0.0;
  double inference_time_ = 0.0;
  double collect_time_ = 0.0;

  // bounding_box 有9个参数
  const int num_output_box_feature_ = 9;

  int x_grid_size_;
  int y_grid_size_;
  int z_grid_size_;

  // tensor input
  std::vector<int> *voxels_shape_;
  std::vector<float> *voxels_data_;
  std::vector<int> *coords_shape_;
  std::vector<int> *coords_data_;
  std::vector<int> *num_points_shape_;
  std::vector<int> *num_points_data_;

  // tensor output
  std::vector<float> box3d_lidar_;
  std::vector<int64_t> label_preds_;
  std::vector<float> scores_;

  bool use_trt_ = false;
  
  std::shared_ptr<paddle_infer::Predictor> predictor_;

};  // class CenterPointDetection

}  // namespace lidar
}  // namespace perception
}  // namespace apollo
