/******************************************************************************
 * Copyright 2020 The Apollo Authors. All Rights Reserved.
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

/**
 * @file
 * @brief Define the birdview img feature renderer class
 */

#pragma once

#include <string>
#include <utility>
#include <vector>

#include "cyber/common/macros.h"
#include "modules/common/configs/vehicle_config_helper.h"
#include "modules/planning/proto/learning_data.pb.h"
#include "modules/planning/proto/planning_semantic_map_config.pb.h"
#include "opencv2/opencv.hpp"

namespace apollo {
namespace planning {

class BirdviewImgFeatureRenderer {
 public:
  /**
   * @brief Destructor
   */
  virtual ~BirdviewImgFeatureRenderer() = default;

  /**
   * @brief initialize renderer
   * @param config configuration
   */
  bool Init(const PlanningSemanticMapConfig& config);

  /**
   * @brief generate multi-channel img as input feature for certain model
   * @param learning_data_frame a proto message containing info for renderering
   * @param img_feature a pointer to opencv img to render on
   */
  bool RenderMultiChannelEnv(const LearningDataFrame& learning_data_frame,
                             cv::Mat* img_feature);

  /**
   * @brief generate bgr img as input feature for certain model
   * @param learning_data_frame a proto message  containing info for renderering
   * @param img_feature a pointer to opencv img to render on
   */
  bool RenderBGREnv(const LearningDataFrame& learning_data_frame,
                    cv::Mat* img_feature);

  /**
   * @brief generate a two channel img, one for cur box, one for cur point
   * @param learning_data_frame a proto message containing info for renderering
   * @param img_feature a pointer to opencv img to render on
   */
  bool RenderCurrentEgoStatus(const LearningDataFrame& learning_data_frame,
                              cv::Mat* img_feature);

  /**
   * @brief generate a single channel img
   * @param learning_data_frame a proto message containing info for renderering
   * @param img_feature a pointer to opencv img to render on
   */
  bool RenderCurrentEgoPoint(const LearningDataFrame& learning_data_frame,
                             cv::Mat* img_feature);

  /**
   * @brief generate a single channel img
   * @param learning_data_frame a proto message containing info for renderering
   * @param img_feature a pointer to opencv img to render on
   */
  bool RenderCurrentEgoBox(const LearningDataFrame& learning_data_frame,
                           cv::Mat* img_feature);

 private:
  /**
   * @brief load a rgb road map from current map
   * @param map_file complete map file name
   */
  bool LoadRoadMap(const std::string& map_file);

  /**
   * @brief crop a rgb speed limit map from current map
   * @param map_file complete map file name
   */
  bool LoadSpeedlimitMap(const std::string& map_file);

  /**
   * @brief crop local roadmap around ego position from base img
   * @param ego_current_x current ego vehicle x coordinates
   * @param ego_current_y current ego vehicle y coordinates
   * @param ego_current_heading current ego vehicle heading
   * @param img_feature a pointer to opencv img to render on
   */
  bool RenderLocalRoadMap(const double ego_current_x,
                          const double ego_current_y,
                          const double ego_current_heading,
                          cv::Mat* img_feature);

  /**
   * @brief crop local speedlimit around ego position from base img
   * @param ego_current_x current ego vehicle x coordinates
   * @param ego_current_y current ego vehicle y coordinates
   * @param ego_current_heading current ego vehicle heading
   * @param img_feature a pointer to opencv img to render on
   */
  bool RenderLocalSpeedlimitMap(const double ego_current_x,
                                const double ego_current_y,
                                const double ego_current_heading,
                                cv::Mat* img_feature);

  /**
   * @brief generate a single channel img, current position is highlighted
   * @param img_feature a pointer to opencv img to render on
   * @param gray_scale color value if img_feature is 1 channel
   * @param bgr_color color value if img_feature is 3 channel bgr
   */
  bool RenderEgoCurrentPoint(cv::Mat* img_feature,
                             const cv::Scalar& gray_scale = cv::Scalar(255),
                             const cv::Scalar& bgr_color = cv::Scalar(255, 255,
                                                                      255));

  /**
   * @brief generate a single channel img, current vehicle box is highlighted
   * @param img_feature a pointer to opencv img to render on
   * @param gray_scale color value if img_feature is 1 channel
   * @param bgr_color color value if img_feature is 3 channel bgr
   */
  bool RenderEgoCurrentBox(cv::Mat* img_feature,
                           const cv::Scalar& gray_scale = cv::Scalar(255),
                           const cv::Scalar& bgr_color = cv::Scalar(255, 255,
                                                                    255));

  /**
   * @brief generate a single channel img, past point is highlighted
   * @param learning_data_frame a proto message containing info for renderering
   * @param current_time_sec current ego time in second
   * @param ego_current_x current ego vehicle x coordinates
   * @param ego_current_y current ego vehicle y coordinates
   * @param ego_current_heading current ego vehicle heading
   * @param img_feature a pointer to opencv img to render on
   * @param gray_scale color value if img_feature is 1 channel
   * @param bgr_color color value if img_feature is 3 channel bgr
   */
  bool RenderEgoPastPoint(
      const LearningDataFrame& learning_data_frame,
      const double current_time_sec, const double ego_current_x,
      const double ego_current_y, const double ego_current_heading,
      cv::Mat* img_feature, const cv::Scalar& gray_scale = cv::Scalar(255),
      const cv::Scalar& bgr_color = cv::Scalar(255, 255, 255));

  /**
   * @brief generate a single channel img, past obstacle box is highlighted.
   obstacles are assumed to be in ego vehicle coordiantes where ego car faces
   toward EAST, so rotation to NORTH is done
   * @param learning_data_frame a proto message containing info for renderering
   * @param current_time_sec current ego time in second
   * @param img_feature a pointer to opencv img to render on
   * @param gray_scale color value if img_feature is 1 channel
   * @param bgr_color color value if img_feature is 3 channel bgr
   */
  bool RenderObsPastBox(const LearningDataFrame& learning_data_frame,
                        const double current_time_sec, cv::Mat* img_feature,
                        const cv::Scalar& gray_scale = cv::Scalar(255),
                        const cv::Scalar& bgr_color = cv::Scalar(0, 255, 0));

  /**
   * @brief generate a single channel img, predicted obstacle box is
   highlighted.   obstacles are assumed to be in ego vehicle coordiantes where
   ego car faces toward EAST, so rotation to NORTH is done
   * @param learning_data_frame a proto message containing info for renderering
   * @param current_time_sec current ego time in second
   * @param img_feature a pointer to opencv img to render on
   * @param gray_scale color value if img_feature is 1 channel
   * @param bgr_color color value if img_feature is 3 channel bgr
   */
  bool RenderObsFutureBox(const LearningDataFrame& learning_data_frame,
                          const double current_time_sec, cv::Mat* img_feature,
                          const cv::Scalar& gray_scale = cv::Scalar(255),
                          const cv::Scalar& bgr_color = cv::Scalar(0, 0, 255));

  /**
   * @brief generate a single channel img, trafficlight related lanes are
   * highlighted
   * @param learning_data_frame a proto message containing info for renderering
   * @param ego_current_x current ego vehicle x coordinates
   * @param ego_current_y current ego vehicle y coordinates
   * @param ego_current_heading current ego vehicle heading
   * @param img_feature a pointer to opencv img to render on
   * @param gray_scale color value if img_feature is 1 channel
   * @param bgr_color color value if img_feature is 3 channel bgr
   */
  bool RenderTrafficLight(
      const LearningDataFrame& learning_data_frame, const double ego_current_x,
      const double ego_current_y, const double ego_current_heading,
      cv::Mat* img_feature, const cv::Scalar& gray_scale = cv::Scalar(255),
      const cv::Scalar& bgr_color = cv::Scalar(255, 255, 255));

  /**
   * @brief generate a single channel img, close routing lanes are highlighted
   * @param learning_data_frame a proto message containing info for renderering
   * @param ego_current_x current ego vehicle x coordinates
   * @param ego_current_y current ego vehicle y coordinates
   * @param ego_current_heading current ego vehicle heading
   * @param img_feature a pointer to opencv img to render on
   * @param gray_scale color value if img_feature is 1 channel
   * @param bgr_color color value if img_feature is 3 channel bgr
   */
  bool RenderRouting(const LearningDataFrame& learning_data_frame,
                     const double ego_current_x, const double ego_current_y,
                     const double ego_current_heading, cv::Mat* img_feature,
                     const cv::Scalar& gray_scale = cv::Scalar(255),
                     const cv::Scalar& bgr_color = cv::Scalar(255, 255, 255));

  /**
   * @brief crop a img by ego around ego position from base img
   * @param ego_x ego point x coordinates
   * @param ego_y ego point y coordinates
   * @param ego_heading ego point heading
   * @param base_map the large map to crop on
   * @param img_feature a pointer to opencv img to render on
   */
  bool CropByPose(const double ego_x, const double ego_y,
                  const double ego_heading, const cv::Mat& base_map,
                  cv::Mat* img_feature);

  /**
   * @brief transform a relative x,y double coordinates in "y axis point up"
   * axis to img "y axis point down" integer coordinates
   * @param local_point_x relative x coordinates
   * @param local_point_y relative y coordinates
   * @param center_point_idx_x relative x coordinates
   * @param center_point_idx_y relative y coordinates
   * @return local_point indexes on the image in cv::Point2i
   */
  cv::Point2i GetPointImgIdx(const double local_point_x,
                             const double local_point_y,
                             const int center_point_idx_x,
                             const int center_point_idx_y);

  /**
   * @brief translate a point wrt to a center and rotate around it
   * @param point_x world x coordinates
   * @param point_y world y coordinates
   * @param center_x center world x coordinates
   * @param center_y center world y coordinates
   * @param theta rotation angle wrt to center
   * @return affined local_point indexes on the image
   */
  cv::Point2i GetAffinedPointImgIdx(const double point_x, const double point_y,
                                    const double center_x,
                                    const double center_y, const double theta);

  /**
   * @brief translate a box wrt to a center and rotate around it
   * @param box_center_x box center world x coordinates
   * @param box_center_y box center world y coordinates
   * @param box_theta rotation angle for box
   * @param box_corner_points east oriented box corner coordinates relative to
   * box center
   * @param center_x center world x coordinates
   * @param center_y center world y coordinates
   * @param theta rotation angle wrt to center
   * @return affined local_box indexes on the image
   */
  std::vector<cv::Point2i> GetAffinedBoxImgIdx(
      const double box_center_x, const double box_center_y,
      const double box_theta,
      const std::vector<std::pair<double, double>>& box_corner_points,
      const double center_x, const double center_y, const double theta);

  PlanningSemanticMapConfig config_;
  common::VehicleConfig ego_vehicle_config_;
  cv::Mat base_roadmap_img_;
  cv::Mat base_speedlimit_img_;
  double map_bottom_left_point_x_ = 0.0;
  double map_bottom_left_point_y_ = 0.0;
  cv::Mat ego_cur_point_img_;
  cv::Mat ego_cur_box_img_;
  cv::Mat stacked_ego_cur_status_img_;

  DECLARE_SINGLETON(BirdviewImgFeatureRenderer)
};
}  // namespace planning
}  // namespace apollo
