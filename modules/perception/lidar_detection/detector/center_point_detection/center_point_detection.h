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
#include <map>
#include <memory>
#include <string>
#include <vector>

#include "paddle/include/paddle_inference_api.h"
#include "pcl/point_cloud.h"
#include "pcl/point_types.h"

#include "modules/perception/common/base/object.h"
#include "modules/perception/common/base/point_cloud.h"
#include "modules/perception/common/inference/inference.h"
#include "modules/perception/common/lidar/common/lidar_frame.h"
#include "modules/perception/lidar_detection/interface/base_lidar_detector.h"
#include "modules/perception/common/interface/base_down_sample.h"
#include "modules/perception/lidar_detection/detector/center_point_detection/proto/model_param.pb.h"

namespace apollo {
namespace perception {
namespace lidar {

class CenterPointDetection : public BaseLidarDetector {
 public:
  /**
   * @brief Construct a new Center Point Detection object
   * 
   */
  CenterPointDetection();

  /**
   * @brief Destroy the Center Point Detection object
   * 
   */
  virtual ~CenterPointDetection() = default;

  /**
   * @brief Init of Center Point Detection module
   * 
   * @param options lidar detection init options
   * @return true 
   * @return false 
   */
  bool Init(const LidarDetectorInitOptions &options =
                LidarDetectorInitOptions()) override;

  /**
   * @brief Detect foreground object using centerpoint
   * 
   * @param options lidar dectection options
   * @param frame lidar frame
   * @return true 
   * @return false 
   */
  bool Detect(const LidarDetectorOptions &options, LidarFrame *frame) override;

  /**
   * @brief Name of Center Point Detection module
   * 
   * @return std::string 
   */
  std::string Name() const { return "CenterPointDetection"; }

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

  void GetBoxCorner(int num_objects,
                    const std::vector<float> &detections,
                    std::vector<float> &box_corner,
                    std::vector<float> &box_rectangular);

  void GetBoxIndices(int num_objects,
                     const std::vector<float> &detections,
                     const std::vector<float> &box_corner,
                     const std::vector<float> &box_rectangular,
                     std::vector<std::shared_ptr<base::Object>> *objects);

  void GetObjects(const Eigen::Affine3d &pose,
                  const std::vector<float> &detections,
                  const std::vector<int64_t> &labels,
                  const std::vector<float> &scores,
                  std::vector<std::shared_ptr<base::Object>> *objects);

  void FilterScore(
      const std::shared_ptr<apollo::perception::base::Blob<float>> &box3d,
      const std::shared_ptr<apollo::perception::base::Blob<float>> &label,
      const std::shared_ptr<apollo::perception::base::Blob<float>> &scores,
      float score_threshold, std::vector<float> *box3d_filtered,
      std::vector<int64_t> *label_preds_filtered,
      std::vector<float> *scores_filtered);

  void FilterDiffScore(
      const std::shared_ptr<apollo::perception::base::Blob<float>> &box3d,
      const std::shared_ptr<apollo::perception::base::Blob<float>> &label,
      const std::shared_ptr<apollo::perception::base::Blob<float>> &scores,
      std::vector<float> *box3d_filtered,
      std::vector<int64_t> *label_preds_filtered,
      std::vector<float> *scores_filtered);

  base::ObjectSubType GetObjectSubType(int label);

  float GetObjectScoreThreshold(int label);

  void FilterObjectsbyPoints(
    std::vector<std::shared_ptr<base::Object>> *objects);

  void FilterForegroundPoints(
    std::vector<std::shared_ptr<base::Object>> *objects);

  void SetPointsInROI(
    std::vector<std::shared_ptr<base::Object>> *objects);

  void FilterObjectsbyClassNMS(
    std::vector<std::shared_ptr<base::Object>> *objects);

  void FilterObjectsbySemanticType(
    std::vector<std::shared_ptr<base::Object>> *objects);

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

  // time statistics
  double downsample_time_ = 0.0;
  double fuse_time_ = 0.0;
  double shuffle_time_ = 0.0;
  double cloud_to_array_time_ = 0.0;
  double inference_time_ = 0.0;
  double collect_time_ = 0.0;
  double postprocess_time_ = 0.0;
  double nms_time_ = 0.0;

  // diff class nms params
  bool diff_class_nms_ = false;
  float diff_class_iou_ = 0.0;
  // true: use nms strategy, false: use nms by object score
  bool nms_strategy_ = false;
  std::map<base::ObjectType, std::vector<base::ObjectType>> nms_strategy_table_;

  // centerpoint param
  centerpoint::ModelParam model_param_;

  // centerpoint type score threshold
  float cone_score_threshold_ = 0.40;
  float ped_score_threshold_ = 0.40;
  float cyc_score_threshold_ = 0.40;
  float small_mot_score_threshold_ = 0.40;
  float big_mot_score_threshold_ = 0.40;

  std::shared_ptr<inference::Inference> inference_;

  std::vector<std::string> input_blob_names_;

  std::vector<std::string> output_blob_names_;

  std::shared_ptr<BaseDownSample> down_sample_;
};  // class CenterPointDetection

}  // namespace lidar
}  // namespace perception
}  // namespace apollo
