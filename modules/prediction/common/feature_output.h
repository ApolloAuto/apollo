/******************************************************************************
 * Copyright 2017 The Apollo Authors. All Rights Reserved.
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

#include <mutex>
#include <string>
#include <vector>

#include "modules/prediction/container/obstacles/obstacle.h"
#include "modules/prediction/proto/offline_features.pb.h"
#include "modules/common_msgs/prediction_msgs/prediction_obstacle.pb.h"

namespace apollo {
namespace prediction {

class FeatureOutput {
 public:
  /**
   * @brief Constructor; disabled
   */
  FeatureOutput() = delete;

  /**
   * @brief Close the output stream
   */
  static void Close();

  /**
   * @brief Reset
   */
  static void Clear();

  /**
   * @brief Check if output is ready
   * @return True if output is ready
   */
  static bool Ready();

  /**
   * @brief Insert a feature
   * @param A feature in proto
   */
  static void InsertFeatureProto(const Feature& feature);

  /**
   * @brief Insert a data_for_learning
   * @param A feature in proto
   */
  static void InsertDataForLearning(const Feature& feature,
                                    const std::vector<double>& feature_values,
                                    const std::string& category,
                                    const LaneSequence* lane_sequence_ptr);

  static void InsertDataForLearning(
      const Feature& feature, const std::vector<double>& feature_values,
      const std::vector<std::string>& string_feature_values,
      const std::string& category, const LaneSequence* lane_sequence_ptr);

  /**
   * @brief Insert a prediction result with predicted trajectories
   * @param Obstacle id
   * @param prediction_obstacle
   * @param obstacle_conf
   * @param scenario
   */
  static void InsertPredictionResult(
      const Obstacle* obstacle, const PredictionObstacle& prediction_obstacle,
      const ObstacleConf& obstacle_conf, const Scenario& scenario);

  /**
   * @brief Insert a frame env
   * @param frame env
   */
  static void InsertFrameEnv(const FrameEnv& frame_env);

  /**
   * @brief Insert a data_for_tuning
   * @param A feature in proto
   * @param values for tuning
   * @param category of the data
   * @param lane sequence
   * @param adc trajectory
   */
  static void InsertDataForTuning(
      const Feature& feature, const std::vector<double>& feature_values,
      const std::string& category, const LaneSequence& lane_sequence,
      const std::vector<apollo::common::TrajectoryPoint>& adc_trajectory);

  /**
   * @brief Write features to a file
   */
  static void WriteFeatureProto();

  /**
   * @brief Write DataForLearning features to a file
   */
  static void WriteDataForLearning();

  /**
   * @brief Write PredictionResult to a file
   */
  static void WritePredictionResult();

  /**
   * @brief Write frame env to a file
   */
  static void WriteFrameEnv();

  /**
   * @brief Write DataForTuning features to a file
   */
  static void WriteDataForTuning();

  /**
   * @brief Get feature size
   * @return Feature size
   */
  static int Size();

  /**
   * @brief Get the size of data_for_learning features.
   * @return The size of data_for_learning features.
   */
  static int SizeOfDataForLearning();

  /**
   * @brief Get the size of prediction results.
   * @return The size of prediction results.
   */
  static int SizeOfPredictionResult();

  /**
   * @brief Get the size of frame env.
   * @return The size of frame env.
   */
  static int SizeOfFrameEnv();

  /**
   * @brief Get the size of data for tuning.
   * @return The size of data for tuning.
   */
  static int SizeOfDataForTuning();

 private:
  static Features features_;
  static std::size_t idx_feature_;
  static ListDataForLearning list_data_for_learning_;
  static std::size_t idx_learning_;
  static ListPredictionResult list_prediction_result_;
  static std::size_t idx_prediction_result_;
  static ListFrameEnv list_frame_env_;
  static std::size_t idx_frame_env_;
  static ListDataForTuning list_data_for_tuning_;
  static std::size_t idx_tuning_;
  static std::mutex mutex_feature_;
};

}  // namespace prediction
}  // namespace apollo
