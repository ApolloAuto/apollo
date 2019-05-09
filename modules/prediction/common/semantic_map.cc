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

void SemanticMap::RunCurrFrame(const FrameEnv& curr_frame_env) {
  // TODO(Hongyi): moving all these magic numbers to conf
  curr_timestamp_ = curr_frame_env.timestamp();
  curr_base_x_ = curr_frame_env.ego_history().feature(0).position().x() - 100.0;
  curr_base_y_ = curr_frame_env.ego_history().feature(0).position().y() - 100.0;
  cv::Rect rect(
      static_cast<int>((curr_base_x_ - 585950.0) / 0.1),
      static_cast<int>(18000 - (curr_base_y_ - 4140000.0) / 0.1) - 2000, 2000,
      2000);
  base_img_(rect).copyTo(curr_img_);

  // Draw ego_vehicle_history
  DrawHistory(curr_frame_env.ego_history(), cv::Scalar(0, 255, 255));

  // Draw obstacles_history
  for (auto& history : curr_frame_env.obstacles_history()) {
    DrawHistory(history);
  }

  // For disaplay
  cv::namedWindow("Display window", cv::WINDOW_NORMAL);
  cv::imshow("Display window", curr_img_);
  cv::waitKey();
}

void SemanticMap::DrawRect(const Feature& feature, const cv::Scalar& color) {
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
  cv::fillPoly(curr_img_, std::vector<std::vector<cv::Point>>({polygon}),
               color);
}

void SemanticMap::DrawPoly(const Feature& feature, const cv::Scalar& color) {
  std::vector<cv::Point> polygon;
  for (auto& polygon_point : feature.polygon_point()) {
    polygon.push_back(GetTransPoint(polygon_point.x(), polygon_point.y()));
  }
  cv::fillPoly(curr_img_, std::vector<std::vector<cv::Point>>({polygon}),
               color);
}

void SemanticMap::DrawHistory(const ObstacleHistory& history,
                              const cv::Scalar& color) {
  for (int i = history.feature_size() - 1; i >= 0; --i) {
    const Feature& feature = history.feature(i);
    double time_decay = 1.0 - curr_timestamp_ + feature.timestamp();
    cv::Scalar decay_color = color * time_decay;
    if (feature.id() == -1) {
      DrawRect(feature, decay_color);
    } else {
      DrawPoly(feature, decay_color);
    }
  }
}

}  // namespace prediction
}  // namespace apollo
