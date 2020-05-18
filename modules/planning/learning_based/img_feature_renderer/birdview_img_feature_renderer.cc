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

#include "modules/planning/learning_based/img_feature_renderer/birdview_img_feature_renderer.h"

#include <string>
#include <vector>

#include "cyber/common/log.h"

namespace apollo {
namespace planning {

BirdviewImgFeatureRenderer::BirdviewImgFeatureRenderer() {}

bool BirdviewImgFeatureRenderer::Init(const PlanningSemanticMapConfig& config) {
  config_ = config;

  // TODO(Jinyun): take map name from upstream and move to conf
  std::string ROADMAP_IMG_PATH =
      "/apollo/modules/planning/data/"
      "sunnyvale_with_two_offices.png";
  std::string SPEEDLIMITMAP_IMG_PATH =
      "/apollo/modules/planning/data/"
      "sunnyvale_with_two_offices_speedlimit.png";
  bool roadmap_img_status = LoadRoadMap(ROADMAP_IMG_PATH);
  bool speedlimit_img_status = LoadSpeedlimitMap(SPEEDLIMITMAP_IMG_PATH);

  if (!roadmap_img_status || !speedlimit_img_status) {
    AERROR << "Base map image read failed";
    return false;
  }

  bool render_ego_point_status = RenderEgoCurrentPoint(&ego_cur_point_img_);
  bool render_ego_box_status = RenderEgoCurrentBox(&ego_cur_box_img_);

  if (!render_ego_point_status || !render_ego_box_status) {
    AERROR << "Ego point or box img rendering failed";
    return false;
  }

  std::vector<cv::Mat> merge_imgs = {ego_cur_point_img_, ego_cur_box_img_};
  cv::merge(merge_imgs, stacked_ego_cur_status_img_);

  return true;
}

bool BirdviewImgFeatureRenderer::RenderMultiChannelEnv(
    const LearningDataFrame& learning_data_frame, cv::Mat* img_feature) {
  cv::Mat ego_past =
      cv::Mat(config_.height(), config_.width(), CV_8UC1, cv::Scalar(0));
  cv::Mat obs_past =
      cv::Mat(config_.height(), config_.width(), CV_8UC1, cv::Scalar(0));
  cv::Mat obs_future =
      cv::Mat(config_.height(), config_.width(), CV_8UC1, cv::Scalar(0));
  cv::Mat road_map =
      cv::Mat(config_.height(), config_.width(), CV_8UC3, cv::Scalar(0, 0, 0));
  cv::Mat routing =
      cv::Mat(config_.height(), config_.width(), CV_8UC1, cv::Scalar(0));
  cv::Mat speed_limit =
      cv::Mat(config_.height(), config_.width(), CV_8UC3, cv::Scalar(0, 0, 0));
  cv::Mat traffic_light =
      cv::Mat(config_.height(), config_.width(), CV_8UC1, cv::Scalar(0));

  int ego_trajectory_point_history_size =
      learning_data_frame.adc_trajectory_point_size();
  if (ego_trajectory_point_history_size < 1) {
    AERROR << "Ego past history is empty";
    return false;
  }

  const auto& current_traj_point = learning_data_frame.adc_trajectory_point(
      ego_trajectory_point_history_size - 1);
  const auto& current_path_point =
      current_traj_point.trajectory_point().path_point();
  const double current_x = current_path_point.x();
  const double current_y = current_path_point.y();
  const double current_heading = current_path_point.theta();

  if (!RenderEgoPastPoint(learning_data_frame, &ego_past)) {
    AERROR << "RenderEgoPastPoint failed";
    return false;
  }
  if (!RenderObsPastBox(learning_data_frame, &obs_past)) {
    AERROR << "RenderObsPastBox failed";
    return false;
  }
  if (!RenderObsFutureBox(learning_data_frame, &obs_future)) {
    AERROR << "RenderObsFutureBox failed";
    return false;
  }
  if (!RenderLocalRoadMap(current_x, current_y, current_heading, &road_map)) {
    AERROR << "RenderLocalRoadMap failed";
    return false;
  }
  if (!RenderRouting(learning_data_frame, &routing)) {
    AERROR << "RenderRouting failed";
    return false;
  }
  if (!RenderLocalSpeedlimitMap(current_x, current_y, current_heading,
                                &speed_limit)) {
    AERROR << "RenderLocalSpeedlimitMap failed";
    return false;
  }
  if (!RenderTrafficLight(learning_data_frame, &traffic_light)) {
    AERROR << "RenderTrafficLight failed";
    return false;
  }

  std::vector<cv::Mat> merge_imgs = {ego_cur_box_img_, ego_past,     obs_past,
                                     obs_future,       road_map,     routing,
                                     speed_limit,      traffic_light};
  cv::merge(merge_imgs, *img_feature);
  return true;
}

// TODO(Jinyun): implement for debugging purpose
bool BirdviewImgFeatureRenderer::RenderBGREnv(
    const LearningDataFrame& learning_data_frame, cv::Mat* img_feature) {
  return false;
}

bool BirdviewImgFeatureRenderer::RenderCurrentEgoStatus(
    const LearningDataFrame& learning_data_frame, cv::Mat* img_feature) {
  stacked_ego_cur_status_img_.copyTo(*img_feature);
  return true;
}

bool BirdviewImgFeatureRenderer::LoadRoadMap(const std::string& map_file) {
  base_roadmap_img_ = cv::imread(map_file);
  return !base_roadmap_img_.empty();
}

bool BirdviewImgFeatureRenderer::LoadSpeedlimitMap(
    const std::string& map_file) {
  base_speedlimit_img_ = cv::imread(map_file);
  return !base_speedlimit_img_.empty();
}

bool BirdviewImgFeatureRenderer::RenderLocalRoadMap(double ego_current_x,
                                                    double ego_current_y,
                                                    double ego_current_heading,
                                                    cv::Mat* img_feature) {
  return false;
}

bool BirdviewImgFeatureRenderer::RenderLocalSpeedlimitMap(
    double ego_current_x, double ego_current_y, double ego_current_heading,
    cv::Mat* img_feature) {
  return false;
}

bool BirdviewImgFeatureRenderer::RenderEgoCurrentPoint(
    cv::Mat* img_feature, const cv::Scalar& gray_scale,
    const cv::Scalar& bgr_color) {
  cv::Scalar color;
  if ((*img_feature).channels() == 3) {
    color = bgr_color;
  } else {
    color = gray_scale;
  }
  return false;
}

bool BirdviewImgFeatureRenderer::RenderEgoCurrentBox(cv::Mat* img_feature,
                                                     const cv::Scalar& color) {
  return false;
}

bool BirdviewImgFeatureRenderer::RenderEgoPastPoint(
    const LearningDataFrame& learning_data_frame, cv::Mat* img_feature,
    const cv::Scalar& color) {
  return false;
}

bool BirdviewImgFeatureRenderer::RenderObsPastBox(
    const LearningDataFrame& learning_data_frame, cv::Mat* img_feature,
    const cv::Scalar& color) {
  return false;
}

bool BirdviewImgFeatureRenderer::RenderObsFutureBox(
    const LearningDataFrame& learning_data_frame, cv::Mat* img_feature,
    const cv::Scalar& color) {
  return false;
}

bool BirdviewImgFeatureRenderer::RenderTrafficLight(
    const LearningDataFrame& learning_data_frame, cv::Mat* img_feature,
    const cv::Scalar& color) {
  return false;
}

bool BirdviewImgFeatureRenderer::RenderRouting(
    const LearningDataFrame& learning_data_frame, cv::Mat* img_feature,
    const cv::Scalar& color) {
  return false;
}

bool BirdviewImgFeatureRenderer::CropByPose(double ego_x, double ego_y,
                                            double ego_heading,
                                            const cv::Mat& base_map,
                                            cv::Mat* img_feature) {
  return false;
}

}  // namespace planning
}  // namespace apollo
