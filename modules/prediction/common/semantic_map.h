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
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or
 *implied. See the License for the specific language governing
 *permissions and limitations under the License.
 *****************************************************************************/

#pragma once

#include <future>
#include <unordered_map>

#include "opencv2/opencv.hpp"

#include "cyber/common/macros.h"
#include "modules/prediction/proto/feature.pb.h"

namespace apollo {
namespace prediction {

class SemanticMap {
 public:
  virtual ~SemanticMap() = default;

  void Init();

  void RunCurrFrame(
      const std::unordered_map<int, ObstacleHistory>& obstacle_id_history_map);

  bool GetMapById(const int obstacle_id, cv::Mat* feature_map);

 private:
  cv::Point2i GetTransPoint(const double x, const double y, const double base_x,
                            const double base_y) {
    return cv::Point2i(static_cast<int>((x - base_x) / 0.1),
                       static_cast<int>(2000 - (y - base_y) / 0.1));
  }

  void DrawBaseMap(const double x, const double y, const double base_x,
                   const double base_y);

  void DrawBaseMapThread();

  void DrawRoads(const common::PointENU& center_point, const double base_x,
                 const double base_y,
                 const cv::Scalar& color = cv::Scalar(64, 64, 64));

  void DrawJunctions(const common::PointENU& center_point, const double base_x,
                     const double base_y,
                     const cv::Scalar& color = cv::Scalar(128, 128, 128));

  void DrawCrosswalks(const common::PointENU& center_point, const double base_x,
                      const double base_y,
                      const cv::Scalar& color = cv::Scalar(192, 192, 192));

  void DrawLanes(const common::PointENU& center_point, const double base_x,
                 const double base_y,
                 const cv::Scalar& color = cv::Scalar(255, 255, 255));

  cv::Scalar HSVtoRGB(double H = 1.0, double S = 1.0, double V = 1.0);

  void DrawRect(const Feature& feature, const cv::Scalar& color,
                const double base_x, const double base_y, cv::Mat* img);

  void DrawPoly(const Feature& feature, const cv::Scalar& color,
                const double base_x, const double base_y, cv::Mat* img);

  void DrawHistory(const ObstacleHistory& history, const cv::Scalar& color,
                   const double base_x, const double base_y, cv::Mat* img);

  cv::Mat CropArea(const cv::Mat& input_img, const cv::Point2i& center_point,
                   const double heading);

  cv::Mat CropByHistory(const ObstacleHistory& history, const cv::Scalar& color,
                        const double base_x, const double base_y);

 private:
  // base_image, base_x, and base_y to be updated by async thread
  cv::Mat base_img_;
  double base_x_ = 0.0;
  double base_y_ = 0.0;

  std::mutex draw_base_map_thread_mutex_;

  // base_image, base_x, and base_y to be used in the current cycle
  cv::Mat curr_img_;
  double curr_base_x_ = 0.0;
  double curr_base_y_ = 0.0;

  std::unordered_map<int, ObstacleHistory> obstacle_id_history_map_;
  Feature ego_feature_;

  std::future<void> task_future_;

  bool started_drawing_ = false;

  DECLARE_SINGLETON(SemanticMap)
};

}  // namespace prediction
}  // namespace apollo
