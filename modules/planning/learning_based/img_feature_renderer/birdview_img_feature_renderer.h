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

#include "modules/planning/proto/learning_data.pb.h"
#include "modules/planning/proto/planning_semantic_map_config.pb.h"
#include "opencv2/opencv.hpp"

namespace apollo {
namespace planning {

class BirdviewImgFeatureRenderer {
 public:
  /**
   * @brief Constructor
   */
  BirdviewImgFeatureRenderer();

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
  bool RenderLocalRoadMap(double ego_current_x, double ego_current_y,
                          double ego_current_heading, cv::Mat* img_feature);

  /**
   * @brief crop local speedlimit around ego position from base img
   * @param ego_current_x current ego vehicle x coordinates
   * @param ego_current_y current ego vehicle y coordinates
   * @param ego_current_heading current ego vehicle heading
   * @param img_feature a pointer to opencv img to render on
   */
  bool RenderLocalSpeedlimitMap(double ego_current_x, double ego_current_y,
                                double ego_current_heading,
                                cv::Mat* img_feature);

  /**
   * @brief generate a single channel img, current position is highlighted
   * @param img_feature a pointer to opencv img to render on
   * @param color single or brga color value
   */
  bool RenderEgoCurrentPoint(cv::Mat* img_feature,
                             const cv::Scalar& gray_scale = cv::Scalar(255),
                             const cv::Scalar& bgr_color = cv::Scalar(255, 255,
                                                                      255));

  /**
   * @brief generate a single channel img, current vehicle box is highlighted
   * @param img_feature a pointer to opencv img to render on
   * @param color single or brga color value
   */
  bool RenderEgoCurrentBox(cv::Mat* img_feature,
                           const cv::Scalar& color = cv::Scalar(255));

  /**
   * @brief generate a single channel img, past point is highlighted
   * @param learning_data_frame a proto message containing info for renderering
   * @param img_feature a pointer to opencv img to render on
   * @param color single or brga color value
   */
  bool RenderEgoPastPoint(const LearningDataFrame& learning_data_frame,
                          cv::Mat* img_feature,
                          const cv::Scalar& color = cv::Scalar(255));

  /**
   * @brief generate a single channel img, past obstacle box is highlighted
   * @param learning_data_frame a proto message containing info for renderering
   * @param img_feature a pointer to opencv img to render on
   * @param color single or brga color value
   */
  bool RenderObsPastBox(const LearningDataFrame& learning_data_frame,
                        cv::Mat* img_feature,
                        const cv::Scalar& color = cv::Scalar(255));

  /**
   * @brief generate a single channel img, predicted obstacle box is highlighted
   * @param learning_data_frame a proto message containing info for renderering
   * @param img_feature a pointer to opencv img to render on
   * @param color single or brga color value
   */
  bool RenderObsFutureBox(const LearningDataFrame& learning_data_frame,
                          cv::Mat* img_feature,
                          const cv::Scalar& color = cv::Scalar(255));

  /**
   * @brief generate a single channel img, trafficlight related lanes are
   * highlighted
   * @param learning_data_frame a proto message containing info for renderering
   * @param img_feature a pointer to opencv img to render on
   * @param color single or brga color value
   */
  bool RenderTrafficLight(const LearningDataFrame& learning_data_frame,
                          cv::Mat* img_feature,
                          const cv::Scalar& color = cv::Scalar(255));

  /**
   * @brief generate a single channel img, close routing lanes are highlighted
   * @param learning_data_frame a proto message containing info for renderering
   * @param img_feature a pointer to opencv img to render on
   * @param color single or brga color value
   */
  bool RenderRouting(const LearningDataFrame& learning_data_frame,
                     cv::Mat* img_feature,
                     const cv::Scalar& color = cv::Scalar(255));

  /**
   * @brief crop a img by ego around ego position from base img
   * @param ego_x ego point x coordinates
   * @param ego_y ego point y coordinates
   * @param ego_heading ego point heading
   * @param base_map the large map to crop on
   * @param img_feature a pointer to opencv img to render on
   */
  bool CropByPose(double ego_x, double ego_y, double ego_heading,
                  const cv::Mat& base_map, cv::Mat* img_feature);

  PlanningSemanticMapConfig config_;
  cv::Mat base_roadmap_img_;
  cv::Mat base_speedlimit_img_;
  cv::Mat ego_cur_point_img_;
  cv::Mat ego_cur_box_img_;
  cv::Mat stacked_ego_cur_status_img_;
};
}  // namespace planning
}  // namespace apollo
