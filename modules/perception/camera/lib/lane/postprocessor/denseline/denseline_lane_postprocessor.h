/******************************************************************************
 * Copyright 2018 The Apollo Authors. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the License);
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an AS IS BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *****************************************************************************/
#pragma once

#include <string>
#include <vector>

#include "modules/perception/base/point.h"
#include "modules/perception/camera/common/camera_frame.h"
#include "modules/perception/camera/lib/interface/base_calibration_service.h"
#include "modules/perception/camera/lib/interface/base_lane_postprocessor.h"
#include "modules/perception/camera/lib/lane/common/common_functions.h"
#include "modules/perception/camera/lib/lane/postprocessor/denseline/denseline_postprocessor.pb.h"
#include "modules/perception/lib/registerer/registerer.h"

namespace apollo {
namespace perception {
namespace camera {

enum class LaneType {
  UNKNOWN_LANE = -1,
  EGO_LANE = 0,
  ADJACENT_LEFT_LANE = 1,
  ADJACENT_RIGHT_LANE = 2
};

class DenselineLanePostprocessor : public BaseLanePostprocessor {
 public:
  DenselineLanePostprocessor() : BaseLanePostprocessor() {}

  virtual ~DenselineLanePostprocessor() {}

  bool Init(const LanePostprocessorInitOptions& options =
                LanePostprocessorInitOptions()) override;

  // @brief: detect lane from image.
  // @param [in]: options
  // @param [in/out]: frame
  // detected lanes should be filled, required,
  // 3D information of lane can be filled, optional.
  bool Process2D(const LanePostprocessorOptions& options,
                 CameraFrame* frame) override;
  // convert image point to the camera coordinate
  // & fit the line using polynomial
  bool Process3D(const LanePostprocessorOptions& options,
                 CameraFrame* frame) override;

  std::string Name() const override;

  std::vector<std::vector<LanePointInfo>> GetLanelinePointSet();
  std::vector<LanePointInfo> GetAllInferLinePointSet();

  void GetLaneCCs(std::vector<unsigned char>* lane_map, int* lane_map_width,
                  int* lane_map_height,
                  std::vector<ConnectedComponent>* connected_components,
                  std::vector<ConnectedComponent>* select_connected_components);

 private:
  void ConvertImagePoint2Camera(CameraFrame* frame);
  // @brief: locate lane line points
  bool LocateLanelinePointSet(const CameraFrame* frame);
  // @brief: calculate the map using network output(score map)
  void CalLaneMap(const float* output_data, int width, int height,
                  std::vector<unsigned char>* lane_map);
  // @brief: select lane center ccs
  bool SelectLanecenterCCs(const std::vector<ConnectedComponent>& lane_ccs,
                           std::vector<ConnectedComponent>* select_lane_ccs);
  // @brief: classify lane ccs position type in image
  bool ClassifyLaneCCsPosTypeInImage(
      const std::vector<ConnectedComponent>& select_lane_ccs,
      std::vector<LaneType>* ccs_pos_type);
  // @brief: compare CC's size
  static bool CompareCCSize(const ConnectedComponent& cc1,
                            const ConnectedComponent& cc2) {
    std::vector<base::Point2DI> cc1_pixels = cc1.GetPixels();
    std::vector<base::Point2DI> cc2_pixels = cc2.GetPixels();
    return cc1_pixels.size() > cc2_pixels.size();
  }
  // @brief: infer point set from lane center
  void InferPointSetFromLaneCenter(
      const std::vector<ConnectedComponent>& lane_ccs,
      const std::vector<LaneType>& ccs_pos_type,
      std::vector<std::vector<LanePointInfo>>* lane_map_group_point_set);
  // @brief: infer point from one cc
  void InferPointSetFromOneCC(
      const ConnectedComponent& lane_cc, int left_index, int right_index,
      std::vector<std::vector<LanePointInfo>>* lane_map_group_point_set);
  // @brief: find the point with the maximum score
  bool MaxScorePoint(const float* score_pointer, const float* x_pointer,
                     const int* x_count_pointer, int y_pos,
                     LanePointInfo* point_info);
  // @brief: convert the point to the original image
  void Convert2OriginalCoord(
      const std::vector<std::vector<LanePointInfo>>& lane_map_group_point_set,
      std::vector<std::vector<LanePointInfo>>* image_group_point_set);
  // @brief: classify laneline pos type in image
  void ClassifyLanelinePosTypeInImage(
      const std::vector<std::vector<LanePointInfo>>& image_group_point_set,
      std::vector<base::LaneLinePositionType>* laneline_type,
      std::vector<bool>* line_flag);
  // @brief: locate neighbor lane lines
  bool LocateNeighborLaneLine(const std::vector<float>& latitude_intersection,
                              int line_index, bool left_flag,
                              int* locate_index);
  // @brief: add image lane line
  void AddImageLaneline(const std::vector<LanePointInfo>& image_point_set,
                        const base::LaneLineType type,
                        const base::LaneLinePositionType pos_type,
                        int line_index,
                        std::vector<base::LaneLine>* lane_marks);
  // @brief: fit camera lane line using polynomial
  void PolyFitCameraLaneline(CameraFrame* frame);

 private:
  int input_image_width_ = 1920;
  int input_image_height_ = 1080;
  int input_offset_x_ = 0;
  int input_offset_y_ = 440;
  int input_crop_width_ = 1920;
  int input_crop_height_ = 640;

  int omit_bottom_line_num_ = 3;
  float laneline_map_score_thresh_ = 0.4f;
  float laneline_point_score_thresh_ = 0.6f;
  int laneline_point_min_num_thresh_ = 2;
  float cc_valid_pixels_ratio_ = 2.0F;
  float laneline_reject_dist_thresh_ = 50.0f;

  int lane_map_width_ = 192;
  int lane_map_height_ = 64;
  int lane_map_dim_ = lane_map_width_ * lane_map_height_;
  int net_model_channel_num_ = 7;
  float lane_max_value_ = 1e6f;
  float lane_map_width_inverse_ = 1.0f / static_cast<float>(lane_map_width_);
  float lane_map_height_inverse_ = 1.0f / static_cast<float>(lane_map_height_);

  lane::LanePostprocessorParam lane_postprocessor_param_;

 private:
  std::vector<std::vector<LanePointInfo>> image_group_point_set_;
  std::vector<std::vector<base::Point3DF>> camera_group_point_set_;

  std::vector<LanePointInfo> image_laneline_point_set_;
  //  lane map of detected pixels
  std::vector<unsigned char> lane_map_;
  //  offset and confidence information
  //  0: dist-left 1: dist-right 2: score
  std::vector<float> lane_output_;
  std::vector<ConnectedComponent> lane_ccs_;
  std::vector<ConnectedComponent> select_lane_ccs_;

  base::Blob<float> lane_pos_blob_;
  base::Blob<int> lane_hist_blob_;
};

}  // namespace camera
}  // namespace perception
}  // namespace apollo
