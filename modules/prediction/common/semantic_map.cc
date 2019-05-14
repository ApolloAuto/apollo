/******************************************************************************
 * Copyright 2019 The Apollo Authors. All Rights Reserved.
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

#include "modules/prediction/common/semantic_map.h"

#include <string>
#include <vector>

#include "cyber/common/file.h"
#include "cyber/common/log.h"
#include "modules/common/configs/config_gflags.h"
#include "modules/common/util/string_util.h"
#include "modules/prediction/container/container_manager.h"
#include "modules/prediction/container/pose/pose_container.h"

namespace apollo {
namespace prediction {

SemanticMap::SemanticMap() {}

void SemanticMap::Init() {
  const std::string file_path =
      apollo::common::util::StrCat(FLAGS_map_dir, "/base_map.png");
  if (cyber::common::PathExists(file_path)) {
    base_img_ = cv::imread(file_path, CV_LOAD_IMAGE_COLOR);
  }
  curr_img_ = cv::Mat(2000, 2000, CV_8UC3, cv::Scalar(0, 0, 0));
}

void SemanticMap::RunCurrFrame(
    const std::unordered_map<int, ObstacleHistory>& obstacle_id_history_map) {
  // TODO(Hongyi): moving all these magic numbers to conf
  const Feature& ego_feature = obstacle_id_history_map.at(-1).feature(0);
  curr_timestamp_ = ego_feature.timestamp();
  curr_base_x_ = ego_feature.position().x() - 100.0;
  curr_base_y_ = ego_feature.position().y() - 100.0;
  cv::Rect rect(
      static_cast<int>((curr_base_x_ - 585950.0) / 0.1),
      static_cast<int>(18000 - (curr_base_y_ - 4140000.0) / 0.1) - 2000, 2000,
      2000);
  base_img_(rect).copyTo(curr_img_);

  // Draw all obstacles_history
  for (const auto obstacle_id_history_pair : obstacle_id_history_map) {
    DrawHistory(obstacle_id_history_pair.second, cv::Scalar(0, 255, 255),
                &curr_img_);
  }

  // Crop ego_vehicle for demo
  cv::Mat output_img =
      CropByHistory(obstacle_id_history_map.at(-1), cv::Scalar(0, 0, 255));
  cv::namedWindow("Demo window", cv::WINDOW_NORMAL);
  cv::imshow("Demo window", output_img);
  cv::waitKey();
}

void SemanticMap::DrawRect(const Feature& feature, const cv::Scalar& color,
                           cv::Mat* img) {
  double obs_l = feature.length();
  double obs_w = feature.width();
  double obs_x = feature.position().x();
  double obs_y = feature.position().y();
  double theta = feature.theta();
  std::vector<cv::Point> polygon;
  // point 1 (head-right point)
  polygon.push_back(
      GetTransPoint(obs_x + (cos(theta) * obs_l - sin(theta) * obs_w) / 2,
                    obs_y + (sin(theta) * obs_l + cos(theta) * obs_w) / 2));
  // point 2 (head-left point)
  polygon.push_back(
      GetTransPoint(obs_x + (cos(theta) * -obs_l - sin(theta) * obs_w) / 2,
                    obs_y + (sin(theta) * -obs_l + cos(theta) * obs_w) / 2));
  // point 3 (back-left point)
  polygon.push_back(
      GetTransPoint(obs_x + (cos(theta) * -obs_l - sin(theta) * -obs_w) / 2,
                    obs_y + (sin(theta) * -obs_l + cos(theta) * -obs_w) / 2));
  // point 4 (back-right point)
  polygon.push_back(
      GetTransPoint(obs_x + (cos(theta) * obs_l - sin(theta) * -obs_w) / 2,
                    obs_y + (sin(theta) * obs_l + cos(theta) * -obs_w) / 2));
  cv::fillPoly(*img, std::vector<std::vector<cv::Point>>({polygon}), color);
}

void SemanticMap::DrawPoly(const Feature& feature, const cv::Scalar& color,
                           cv::Mat* img) {
  std::vector<cv::Point> polygon;
  for (auto& polygon_point : feature.polygon_point()) {
    polygon.push_back(GetTransPoint(polygon_point.x(), polygon_point.y()));
  }
  cv::fillPoly(*img, std::vector<std::vector<cv::Point>>({polygon}), color);
}

void SemanticMap::DrawHistory(const ObstacleHistory& history,
                              const cv::Scalar& color, cv::Mat* img) {
  for (int i = history.feature_size() - 1; i >= 0; --i) {
    const Feature& feature = history.feature(i);
    double time_decay = 1.0 - curr_timestamp_ + feature.timestamp();
    cv::Scalar decay_color = color * time_decay;
    if (feature.id() == -1) {
      DrawRect(feature, decay_color, img);
    } else {
      DrawPoly(feature, decay_color, img);
    }
  }
}

cv::Mat SemanticMap::CropArea(const cv::Mat& input_img,
                              const cv::Point2i& center_point,
                              const double heading) {
  cv::Mat rotation_mat =
      cv::getRotationMatrix2D(center_point, 90.0 - heading * 180.0 / M_PI, 1.0);
  cv::Mat rotated_mat;
  cv::warpAffine(input_img, rotated_mat, rotation_mat, input_img.size());
  cv::Rect rect(center_point.x - 200, center_point.y - 300, 400, 400);
  cv::Mat output_img;
  cv::resize(rotated_mat(rect), output_img, cv::Size(224, 224));
  return output_img;
}

cv::Mat SemanticMap::CropByHistory(const ObstacleHistory& history,
                                   const cv::Scalar& color) {
  cv::Mat feature_map = curr_img_.clone();
  DrawHistory(history, color, &feature_map);
  const Feature& curr_feature = history.feature(0);
  cv::Point2i center_point =
      GetTransPoint(curr_feature.position().x(), curr_feature.position().y());
  return CropArea(feature_map, center_point, curr_feature.theta());
}

}  // namespace prediction
}  // namespace apollo
