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

#include "modules/planning/planning_base/learning_based/img_feature_renderer/birdview_img_feature_renderer.h"

#include <string>
#include <utility>
#include <vector>

#include "modules/common_msgs/perception_msgs/traffic_light_detection.pb.h"

#include "cyber/common/log.h"
#include "modules/common/configs/config_gflags.h"
#include "modules/map/hdmap/hdmap_util.h"

namespace apollo {
namespace planning {

// TODO(Jinyun): take map name from upstream and move to conf
static const char ROADMAP_IMG_PATH[] =
    "/apollo/modules/planning/planning_base/learning_based/data/semantic_map/"
    "sunnyvale_with_two_offices.png";
static const char SPEEDLIMITMAP_IMG_PATH[] =
    "/apollo/modules/planning/planning_base/learning_based/data/semantic_map/"
    "sunnyvale_with_two_offices_speedlimit.png";

BirdviewImgFeatureRenderer::BirdviewImgFeatureRenderer() {}

bool BirdviewImgFeatureRenderer::Init(const PlanningSemanticMapConfig& config) {
  config_ = config;
  ego_vehicle_config_ = common::VehicleConfigHelper::GetConfig();

  // a redundant call to HDMapUtil::BaseMap() to save time for renderering when
  // the basemap is not yet initialized in HDMapUtil
  apollo::hdmap::HDMapUtil::BaseMap();

  const std::string map_name =
      FLAGS_map_dir.substr(FLAGS_map_dir.find_last_of("/") + 1);
  if (map_name != "sunnyvale_with_two_offices" && map_name != "sunnyvale") {
    AERROR << "Map other than sunnyvale_with_two_offices are not supported";
  }
  // TODO(Jinyun): add sunnyvale map or draw basemap online
  if (map_name == "sunnyvale") {
    AWARN << "use sunnyvale_with_two_offices for sunnyvale for now";
  }

  // TODO(Jinyun): move to a more managable place
  map_bottom_left_point_x_ = 585875.3316302994;
  map_bottom_left_point_y_ = 4139916.6342316796;
  bool roadmap_img_status = LoadRoadMap(ROADMAP_IMG_PATH);
  bool speedlimit_img_status = LoadSpeedlimitMap(SPEEDLIMITMAP_IMG_PATH);

  if (!roadmap_img_status || !speedlimit_img_status) {
    AERROR << "Base map image read failed";
    return false;
  }

  ego_cur_point_img_ =
      cv::Mat(config_.height(), config_.width(), CV_8UC1, cv::Scalar(0));
  ego_cur_box_img_ =
      cv::Mat(config_.height(), config_.width(), CV_8UC1, cv::Scalar(0));

  bool render_ego_point_status = RenderEgoCurrentPoint(&ego_cur_point_img_);
  bool render_ego_box_status = RenderEgoCurrentBox(&ego_cur_box_img_);

  if (!render_ego_point_status || !render_ego_box_status) {
    AERROR << "Ego point or box img rendering failed";
    return false;
  }

  if (base_roadmap_img_.size[0] != base_speedlimit_img_.size[0] ||
      base_roadmap_img_.size[1] != base_speedlimit_img_.size[1]) {
    AERROR << "base map sizes doesn't match";
    return false;
  }

  std::vector<cv::Mat> merge_imgs = {ego_cur_point_img_, ego_cur_box_img_};
  cv::merge(merge_imgs, stacked_ego_cur_status_img_);

  return true;
}

bool BirdviewImgFeatureRenderer::RenderMultiChannelEnv(
    const LearningDataFrame& learning_data_frame, cv::Mat* img_feature) {
  int ego_trajectory_point_history_size =
      learning_data_frame.adc_trajectory_point_size();
  if (ego_trajectory_point_history_size < 1) {
    AERROR << "Ego past history is empty";
    return false;
  }

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

  const auto& current_traj_point = learning_data_frame.adc_trajectory_point(
      ego_trajectory_point_history_size - 1);
  const double current_time_sec = current_traj_point.timestamp_sec();
  const auto& current_path_point =
      current_traj_point.trajectory_point().path_point();
  const double current_x = current_path_point.x();
  const double current_y = current_path_point.y();
  const double current_heading = current_path_point.theta();

  if (!RenderEgoPastPoint(learning_data_frame, current_time_sec, current_x,
                          current_y, current_heading, &ego_past)) {
    AERROR << "RenderEgoPastPoint failed";
    return false;
  }
  if (!RenderObsPastBox(learning_data_frame, current_time_sec, &obs_past)) {
    AERROR << "RenderObsPastBox failed";
    return false;
  }
  if (!RenderObsFutureBox(learning_data_frame, current_time_sec, &obs_future)) {
    AERROR << "RenderObsFutureBox failed";
    return false;
  }
  if (!RenderLocalRoadMap(current_x, current_y, current_heading, &road_map)) {
    AERROR << "RenderLocalRoadMap failed";
    return false;
  }
  if (!RenderRouting(learning_data_frame, current_x, current_y, current_heading,
                     &routing)) {
    AERROR << "RenderRouting failed";
    return false;
  }
  if (!RenderLocalSpeedlimitMap(current_x, current_y, current_heading,
                                &speed_limit)) {
    AERROR << "RenderLocalSpeedlimitMap failed";
    return false;
  }
  if (!RenderTrafficLight(learning_data_frame, current_x, current_y,
                          current_heading, &traffic_light)) {
    AERROR << "RenderTrafficLight failed";
    return false;
  }

  std::vector<cv::Mat> merge_imgs = {ego_cur_box_img_, ego_past,     obs_past,
                                     obs_future,       road_map,     routing,
                                     speed_limit,      traffic_light};
  cv::merge(merge_imgs, *img_feature);
  return true;
}

bool BirdviewImgFeatureRenderer::RenderBGREnv(
    const LearningDataFrame& learning_data_frame, cv::Mat* img_feature) {
  int ego_trajectory_point_history_size =
      learning_data_frame.adc_trajectory_point_size();
  if (ego_trajectory_point_history_size < 1) {
    AERROR << "Ego past history is empty";
    return false;
  }

  cv::Mat bgr_canvas =
      cv::Mat(config_.height(), config_.width(), CV_8UC3, cv::Scalar(0, 0, 0));

  const auto& current_traj_point = learning_data_frame.adc_trajectory_point(
      ego_trajectory_point_history_size - 1);
  const double current_time_sec = current_traj_point.timestamp_sec();
  const auto& current_path_point =
      current_traj_point.trajectory_point().path_point();
  const double current_x = current_path_point.x();
  const double current_y = current_path_point.y();
  const double current_heading = current_path_point.theta();

  if (!RenderLocalRoadMap(current_x, current_y, current_heading, &bgr_canvas)) {
    AERROR << "RenderLocalRoadMap failed";
    return false;
  }
  if (!RenderRouting(learning_data_frame, current_x, current_y, current_heading,
                     &bgr_canvas)) {
    AERROR << "RenderRouting failed";
    return false;
  }
  if (!RenderTrafficLight(learning_data_frame, current_x, current_y,
                          current_heading, &bgr_canvas)) {
    AERROR << "RenderTrafficLight failed";
    return false;
  }
  if (!RenderObsPastBox(learning_data_frame, current_time_sec, &bgr_canvas)) {
    AERROR << "RenderObsPastBox failed";
    return false;
  }
  if (!RenderObsFutureBox(learning_data_frame, current_time_sec, &bgr_canvas)) {
    AERROR << "RenderObsFutureBox failed";
    return false;
  }
  if (!RenderEgoCurrentBox(&bgr_canvas)) {
    AERROR << "RenderEgoCurrentBox failed";
    return false;
  }
  if (!RenderEgoPastPoint(learning_data_frame, current_time_sec, current_x,
                          current_y, current_heading, &bgr_canvas)) {
    AERROR << "RenderEgoPastPoint failed";
    return false;
  }

  bgr_canvas.copyTo(*img_feature);

  return true;
}

bool BirdviewImgFeatureRenderer::RenderCurrentEgoStatus(
    const LearningDataFrame& learning_data_frame, cv::Mat* img_feature) {
  stacked_ego_cur_status_img_.copyTo(*img_feature);
  return true;
}

bool BirdviewImgFeatureRenderer::RenderCurrentEgoPoint(
    const LearningDataFrame& learning_data_frame, cv::Mat* img_feature) {
  ego_cur_point_img_.copyTo(*img_feature);
  return true;
}

bool BirdviewImgFeatureRenderer::RenderCurrentEgoBox(
    const LearningDataFrame& learning_data_frame, cv::Mat* img_feature) {
  ego_cur_box_img_.copyTo(*img_feature);
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

bool BirdviewImgFeatureRenderer::RenderLocalRoadMap(
    const double ego_current_x, const double ego_current_y,
    const double ego_current_heading, cv::Mat* img_feature) {
  return CropByPose(ego_current_x, ego_current_y, ego_current_heading,
                    base_roadmap_img_, img_feature);
}

bool BirdviewImgFeatureRenderer::RenderLocalSpeedlimitMap(
    const double ego_current_x, const double ego_current_y,
    const double ego_current_heading, cv::Mat* img_feature) {
  return CropByPose(ego_current_x, ego_current_y, ego_current_heading,
                    base_speedlimit_img_, img_feature);
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
  (*img_feature).at<uchar>(config_.ego_idx_y(), config_.ego_idx_x()) = color[0];
  return true;
}

bool BirdviewImgFeatureRenderer::RenderEgoCurrentBox(
    cv::Mat* img_feature, const cv::Scalar& gray_scale,
    const cv::Scalar& bgr_color) {
  cv::Scalar color;
  if ((*img_feature).channels() == 3) {
    color = bgr_color;
  } else {
    color = gray_scale;
  }
  const auto& param = ego_vehicle_config_.vehicle_param();
  std::vector<cv::Point2i> box_corner_points =
      GetAffinedBoxImgIdx(0.0, 0.0, M_PI_2,
                          {
                              std::make_pair(param.front_edge_to_center(),
                                             param.left_edge_to_center()),
                              std::make_pair(param.front_edge_to_center(),
                                             -param.right_edge_to_center()),
                              std::make_pair(-param.back_edge_to_center(),
                                             -param.right_edge_to_center()),
                              std::make_pair(-param.back_edge_to_center(),
                                             param.left_edge_to_center()),
                          },
                          0.0, 0.0, 0.0);
  cv::fillPoly(
      *img_feature,
      std::vector<std::vector<cv::Point>>({std::move(box_corner_points)}),
      color);
  return true;
}

bool BirdviewImgFeatureRenderer::RenderEgoPastPoint(
    const LearningDataFrame& learning_data_frame, const double current_time_sec,
    const double ego_current_x, const double ego_current_y,
    const double ego_current_heading, cv::Mat* img_feature,
    const cv::Scalar& gray_scale, const cv::Scalar& bgr_color) {
  cv::Scalar color;
  if ((*img_feature).channels() == 3) {
    color = bgr_color;
  } else {
    color = gray_scale;
  }
  const auto& ego_past = learning_data_frame.adc_trajectory_point();
  cv::Scalar gradual_change_color;
  for (const auto& ego_past_point : ego_past) {
    const double relative_time =
        current_time_sec - ego_past_point.timestamp_sec();
    if (relative_time > config_.max_ego_past_horizon()) {
      continue;
    }
    gradual_change_color =
        color * (1 - relative_time / config_.max_ego_past_horizon());
    cv::circle(*img_feature,
               GetAffinedPointImgIdx(
                   ego_past_point.trajectory_point().path_point().x(),
                   ego_past_point.trajectory_point().path_point().y(),
                   ego_current_x, ego_current_y, M_PI_2 - ego_current_heading),
               2, gradual_change_color, -1);
  }
  return true;
}

bool BirdviewImgFeatureRenderer::RenderObsPastBox(
    const LearningDataFrame& learning_data_frame, const double current_time_sec,
    cv::Mat* img_feature, const cv::Scalar& gray_scale,
    const cv::Scalar& bgr_color) {
  cv::Scalar color;
  if ((*img_feature).channels() == 3) {
    color = bgr_color;
  } else {
    color = gray_scale;
  }

  for (const auto& obstacle : learning_data_frame.obstacle()) {
    if (obstacle.obstacle_trajectory().evaluated_trajectory_point_size() == 0) {
      continue;
    }
    const double obstacle_box_length = obstacle.length();
    const double obstacle_box_width = obstacle.width();
    const auto& past_traj =
        obstacle.obstacle_trajectory().evaluated_trajectory_point();
    cv::Scalar gradual_change_color;
    for (const auto& traj_point : past_traj) {
      const double relative_time =
          current_time_sec - traj_point.timestamp_sec();
      if (relative_time > config_.max_obs_past_horizon()) {
        continue;
      }
      gradual_change_color =
          color * (1 - relative_time / config_.max_obs_past_horizon());
      const auto& path_point = traj_point.trajectory_point().path_point();
      std::vector<cv::Point2i> box_corner_points = GetAffinedBoxImgIdx(
          path_point.x(), path_point.y(), M_PI_2 + path_point.theta(),
          {
              std::make_pair(obstacle_box_length / 2, obstacle_box_width / 2),
              std::make_pair(obstacle_box_length / 2, -obstacle_box_width / 2),
              std::make_pair(-obstacle_box_length / 2, -obstacle_box_width / 2),
              std::make_pair(-obstacle_box_length / 2, obstacle_box_width / 2),
          },
          0.0, 0.0, M_PI_2);
      cv::fillPoly(
          *img_feature,
          std::vector<std::vector<cv::Point>>({std::move(box_corner_points)}),
          color);
    }
  }

  return true;
}

bool BirdviewImgFeatureRenderer::RenderObsFutureBox(
    const LearningDataFrame& learning_data_frame, const double current_time_sec,
    cv::Mat* img_feature, const cv::Scalar& gray_scale,
    const cv::Scalar& bgr_color) {
  cv::Scalar color;
  if ((*img_feature).channels() == 3) {
    color = bgr_color;
  } else {
    color = gray_scale;
  }

  for (const auto& obstacle : learning_data_frame.obstacle()) {
    const double obstacle_box_length = obstacle.length();
    const double obstacle_box_width = obstacle.width();

    if (obstacle.obstacle_prediction().is_static()) {
      const auto& past_traj_points_size =
          obstacle.obstacle_trajectory().evaluated_trajectory_point_size();
      if (past_traj_points_size == 0) {
        AERROR << "obstacle[" << obstacle.id()
               << "] is static without tracking history points";
        return false;
      }
      const auto& last_past_traj_point =
          obstacle.obstacle_trajectory().evaluated_trajectory_point(
              past_traj_points_size - 1);
      const auto& path_point =
          last_past_traj_point.trajectory_point().path_point();
      std::vector<cv::Point2i> box_corner_points = GetAffinedBoxImgIdx(
          path_point.x(), path_point.y(), M_PI_2 + path_point.theta(),
          {
              std::make_pair(obstacle_box_length / 2, obstacle_box_width / 2),
              std::make_pair(obstacle_box_length / 2, -obstacle_box_width / 2),
              std::make_pair(-obstacle_box_length / 2, -obstacle_box_width / 2),
              std::make_pair(-obstacle_box_length / 2, obstacle_box_width / 2),
          },
          0.0, 0.0, M_PI_2);
      cv::fillPoly(
          *img_feature,
          std::vector<std::vector<cv::Point>>({std::move(box_corner_points)}),
          color);
      continue;
    }

    const auto& predicted_traj_size =
        obstacle.obstacle_prediction().trajectory_size();
    if (predicted_traj_size == 0) {
      AWARN << "obstacle[" << obstacle.id() << "] has no prediction trajectory";
      continue;
    }

    int max_prob = 0;
    int max_prob_idx = 0;
    for (int i = 0; i < predicted_traj_size; ++i) {
      const auto& trajectory = obstacle.obstacle_prediction().trajectory(i);
      if (trajectory.probability() > max_prob) {
        max_prob = trajectory.probability();
        max_prob_idx = i;
      }
    }

    const auto& predicted_traj = obstacle.obstacle_prediction()
                                     .trajectory(max_prob_idx)
                                     .trajectory_point();
    cv::Scalar gradual_change_color;
    for (const auto& traj_point : predicted_traj) {
      const double relative_time =
          traj_point.timestamp_sec() - current_time_sec;
      if (relative_time > config_.max_obs_future_horizon()) {
        break;
      }
      gradual_change_color =
          color * relative_time / config_.max_obs_past_horizon();
      const auto& path_point = traj_point.trajectory_point().path_point();
      std::vector<cv::Point2i> box_corner_points = GetAffinedBoxImgIdx(
          path_point.x(), path_point.y(), M_PI_2 + path_point.theta(),
          {
              std::make_pair(obstacle_box_length / 2, obstacle_box_width / 2),
              std::make_pair(obstacle_box_length / 2, -obstacle_box_width / 2),
              std::make_pair(-obstacle_box_length / 2, -obstacle_box_width / 2),
              std::make_pair(-obstacle_box_length / 2, obstacle_box_width / 2),
          },
          0.0, 0.0, M_PI_2);
      cv::fillPoly(
          *img_feature,
          std::vector<std::vector<cv::Point>>({std::move(box_corner_points)}),
          color);
    }
  }
  return true;
}

bool BirdviewImgFeatureRenderer::RenderTrafficLight(
    const LearningDataFrame& learning_data_frame, const double ego_current_x,
    const double ego_current_y, const double ego_current_heading,
    cv::Mat* img_feature, const cv::Scalar& gray_scale,
    const cv::Scalar& bgr_color) {
  cv::Scalar color;
  if ((*img_feature).channels() == 3) {
    color = bgr_color;
  } else {
    color = gray_scale;
  }
  for (const auto& traffic_light_status :
       learning_data_frame.traffic_light_detection().traffic_light()) {
    const auto& traffic_light =
        apollo::hdmap::HDMapUtil::BaseMap().GetSignalById(
            hdmap::MakeMapId(traffic_light_status.id()));
    if (traffic_light == nullptr) {
      AERROR << "traffic_light [" << traffic_light_status.id() << "] not found";
      return false;
    }
    if (traffic_light_status.color() == perception::TrafficLight::RED) {
      color = cv::Scalar(255);
    } else if (traffic_light_status.color() ==
               perception::TrafficLight::YELLOW) {
      color = cv::Scalar(192);
    } else {
      color = cv::Scalar(128);
    }
    for (const auto& overlap_id : traffic_light->signal().overlap_id()) {
      const auto& overlap =
          apollo::hdmap::HDMapUtil::BaseMap().GetOverlapById(overlap_id);
      if (overlap == nullptr) {
        AERROR << "overlap [" << overlap_id.id() << "] not found";
        return false;
      }
      for (const auto& overlap_object : overlap->overlap().object()) {
        if (!overlap_object.has_lane_overlap_info()) {
          continue;
        }
        const auto& lane = apollo::hdmap::HDMapUtil::BaseMap().GetLaneById(
            overlap_object.id());
        if (lane == nullptr) {
          AERROR << "lane [" << overlap_object.id().id() << "] not found";
          return false;
        }
        for (const auto& segment : lane->lane().central_curve().segment()) {
          const int segment_point_size = segment.line_segment().point_size();
          for (int i = 0; i < segment_point_size - 1; ++i) {
            const auto& p0 = GetAffinedPointImgIdx(
                segment.line_segment().point(i).x(),
                segment.line_segment().point(i).y(), ego_current_x,
                ego_current_y, M_PI_2 - ego_current_heading);
            const auto& p1 = GetAffinedPointImgIdx(
                segment.line_segment().point(i + 1).x(),
                segment.line_segment().point(i + 1).y(), ego_current_x,
                ego_current_y, M_PI_2 - ego_current_heading);
            cv::line(*img_feature, p0, p1, color, 4);
          }
        }
      }
    }
  }

  return true;
}

bool BirdviewImgFeatureRenderer::RenderRouting(
    const LearningDataFrame& learning_data_frame, const double ego_current_x,
    const double ego_current_y, const double ego_current_heading,
    cv::Mat* img_feature, const cv::Scalar& gray_scale,
    const cv::Scalar& bgr_color) {
  cv::Scalar color;
  if ((*img_feature).channels() == 3) {
    color = bgr_color;
  } else {
    color = gray_scale;
  }

  const int routing_lanes_size =
      learning_data_frame.routing().local_routing_lane_id_size();
  if (routing_lanes_size == 0) {
    AERROR << "routing is empty";
    return false;
  }

  cv::Scalar color_gradient = color / routing_lanes_size;
  for (int i = 0; i < routing_lanes_size; ++i) {
    const cv::Scalar routing_color = color - color_gradient * i;
    const auto& lane =
        apollo::hdmap::HDMapUtil::BaseMap().GetLaneById(hdmap::MakeMapId(
            learning_data_frame.routing().local_routing_lane_id(i)));
    if (lane == nullptr) {
      AERROR << "lane ["
             << learning_data_frame.routing().local_routing_lane_id(i)
             << "] not found";
      return false;
    }
    for (const auto& segment : lane->lane().central_curve().segment()) {
      const int segment_point_size = segment.line_segment().point_size();
      for (int i = 0; i < segment_point_size - 1; ++i) {
        const auto& p0 = GetAffinedPointImgIdx(
            segment.line_segment().point(i).x(),
            segment.line_segment().point(i).y(), ego_current_x, ego_current_y,
            M_PI_2 - ego_current_heading);
        const auto& p1 = GetAffinedPointImgIdx(
            segment.line_segment().point(i + 1).x(),
            segment.line_segment().point(i + 1).y(), ego_current_x,
            ego_current_y, M_PI_2 - ego_current_heading);
        cv::line(*img_feature, p0, p1, routing_color, 12);
      }
    }
  }
  return true;
}

bool BirdviewImgFeatureRenderer::CropByPose(const double ego_x,
                                            const double ego_y,
                                            const double ego_heading,
                                            const cv::Mat& base_map,
                                            cv::Mat* img_feature) {
  // use dimension of base_roadmap_img as it's assumed that all base maps have
  // the same size
  cv::Point2i ego_img_idx = GetPointImgIdx(ego_x - map_bottom_left_point_x_,
                                           ego_y - map_bottom_left_point_y_, 0,
                                           base_roadmap_img_.size[0]);
  if (ego_img_idx.x < 0 || ego_img_idx.x + 1 > base_roadmap_img_.size[1] ||
      ego_img_idx.y < 0 || ego_img_idx.y + 1 > base_roadmap_img_.size[0]) {
    AERROR << "ego vehicle position out of bound of base map";
    return false;
  }

  const int rough_radius = static_cast<int>(sqrt(
      config_.height() * config_.height() + config_.width() * config_.width()));
  if (ego_img_idx.x - rough_radius < 0 || ego_img_idx.y - rough_radius < 0) {
    AERROR << "cropping out of bound of base map";
    return false;
  }

  cv::Rect rough_rect(ego_img_idx.x - rough_radius,
                      ego_img_idx.y - rough_radius, 2 * rough_radius,
                      2 * rough_radius);
  cv::Mat rotation_matrix =
      cv::getRotationMatrix2D(cv::Point2i(rough_radius, rough_radius),
                              90.0 - ego_heading * 180.0 / M_PI, 1.0);
  cv::Mat rotated_mat;
  cv::warpAffine(base_map(rough_rect), rotated_mat, rotation_matrix,
                 base_map(rough_rect).size());

  cv::Rect fine_rect(rough_radius - config_.ego_idx_x(),
                     rough_radius - config_.ego_idx_y(), config_.height(),
                     config_.width());

  rotated_mat(fine_rect).copyTo(*img_feature);
  return true;
}

cv::Point2i BirdviewImgFeatureRenderer::GetPointImgIdx(
    const double local_point_x, const double local_point_y,
    const int center_point_idx_x, const int center_point_idx_y) {
  return cv::Point2i(
      center_point_idx_x +
          static_cast<int>(local_point_x / config_.resolution()),
      center_point_idx_y -
          static_cast<int>(local_point_y / config_.resolution()));
}

cv::Point2i BirdviewImgFeatureRenderer::GetAffinedPointImgIdx(
    const double point_x, const double point_y, const double center_x,
    const double center_y, const double theta) {
  const double local_x = point_x - center_x;
  const double local_y = point_y - center_y;

  const double rotated_local_x = local_x * cos(theta) - local_y * sin(theta);
  const double rotated_local_y = local_x * sin(theta) + local_y * cos(theta);

  return GetPointImgIdx(rotated_local_x, rotated_local_y, config_.ego_idx_x(),
                        config_.ego_idx_y());
}

std::vector<cv::Point2i> BirdviewImgFeatureRenderer::GetAffinedBoxImgIdx(
    const double box_center_x, const double box_center_y,
    const double box_theta,
    const std::vector<std::pair<double, double>>& box_corner_points,
    const double center_x, const double center_y, const double theta) {
  const double local_box_center_x = box_center_x - center_x;
  const double local_box_center_y = box_center_y - center_y;

  const double affined_local_box_center_x =
      local_box_center_x * cos(theta) - local_box_center_y * sin(theta);
  const double affined_local_box_center_y =
      local_box_center_x * sin(theta) + local_box_center_y * cos(theta);

  std::vector<cv::Point2i> affined_box;

  for (const auto& point : box_corner_points) {
    const double affined_corner_x = point.first * cos(box_theta) -
                                    point.second * sin(box_theta) +
                                    affined_local_box_center_x;
    const double affined_corner_y = point.first * sin(box_theta) +
                                    point.second * cos(box_theta) +
                                    affined_local_box_center_y;
    affined_box.push_back(GetPointImgIdx(affined_corner_x, affined_corner_y,
                                         config_.ego_idx_x(),
                                         config_.ego_idx_y()));
  }

  return affined_box;
}

}  // namespace planning
}  // namespace apollo
