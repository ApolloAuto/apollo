/******************************************************************************
 * Copyright 2018 The Apollo Authors. All Rights Reserved.
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

#include <map>
#include <string>
#include <vector>

#include "cyber/common/macros.h"
#include "modules/perception/multi_sensor_fusion/common/dst_evidence.h"
#include "modules/perception/multi_sensor_fusion/interface/base_existence_fusion.h"

namespace apollo {
namespace perception {
namespace fusion {

struct ToicDstMaps {
  // for (N)TOIC: (not)target of interest in camera judgement
  enum { TOIC = (1 << 0), NTOIC = (1 << 1), TOICUNKNOWN = (TOIC | NTOIC) };
  std::vector<uint64_t> fod_subsets_ = {TOIC, NTOIC, TOICUNKNOWN};
  std::vector<std::string> subset_names_ = {"TOIC", "NTOIC", "TOICUNKNOWN"};
};

struct ExistenceDstMaps {
  enum { EXIST = (1 << 0), NEXIST = (1 << 1), EXISTUNKNOWN = (EXIST | NEXIST) };
  std::vector<uint64_t> fod_subsets_ = {EXIST, NEXIST, EXISTUNKNOWN};
  std::vector<std::string> subset_names_ = {"EXIST", "NEXIST", "EXISTUNKNOWN"};
};

struct DstExistenceFusionOptions {
  std::map<std::string, double> camera_max_valid_dist_ = {
      {"camera_smartereye", 110},   {"camera_front_obstacle", 110},
      {"camera_front_narrow", 150}, {"front_6mm", 110},
      {"camera_rear", 60},          {"camera_front", 110}
  };
  double track_object_max_match_distance_ = 4.0;
};

class DstExistenceFusion : public BaseExistenceFusion {
 public:
  explicit DstExistenceFusion(TrackPtr track);
  ~DstExistenceFusion() = default;

  /**
   * @brief add dst application
   *
   * @param options
   * @return true
   * @return false
   */
  static bool Init(const ExistenceFusionInitOptions &options);

  // @brief: update track state with measurement
  // @param [in]: measurement
  // @param [in]: target_timestamp
  void UpdateWithMeasurement(const SensorObjectPtr measurement,
                             double target_timestamp,
                             double match_dist) override;
  /**
   * @brief update track state without measurement
   *
   * @param sensor_id
   * @param measurement_timestamp
   * @param target_timestamp
   * @param min_match_dist
   */
  void UpdateWithoutMeasurement(const std::string &sensor_id,
                                double measurement_timestamp,
                                double target_timestamp,
                                double min_match_dist) override;

  std::string Name() const;

  /**
   * @brief Get toic score
   *
   * @return double
   */
  double GetToicScore() const { return toic_score_; }

  /**
   * @brief Get existence probability
   *
   * @return double
   */
  double GetExistenceProbability() const;

 private:
  /**
   * @brief Update toic with camera measurement
   *
   * @param camera_obj
   * @param match_dist
   */
  void UpdateToicWithCameraMeasurement(const SensorObjectPtr &camera_obj,
                                       double match_dist);

  /**
   * @brief Update toic without camera measurement
   *
   * @param sensor_id
   * @param measurement_timestamp
   * @param match_dist
   */
  void UpdateToicWithoutCameraMeasurement(const std::string &sensor_id,
                                          double measurement_timestamp,
                                          double match_dist);

  /**
   * @brief compute distance decay
   *
   * @param obj
   * @param sensor_id
   * @param timestamp
   * @return double
   */
  double ComputeDistDecay(base::ObjectConstPtr obj,
                          const std::string &sensor_id, double timestamp);

  /**
   * @brief Compute feature influence
   *
   * @param measurement
   * @return double
   */
  double ComputeFeatureInfluence(const SensorObjectPtr measurement);

  /**
   * @brief Get exist reliability
   *
   * @param measurement
   * @return double
   */
  double GetExistReliability(const SensorObjectPtr measurement);

  /**
   * @brief Get unexist reliability
   *
   * @param sensor_id
   * @return double
   */
  double GetUnexistReliability(const std::string &sensor_id);

  /**
   * @brief Get toic probability
   *
   * @return double
   */
  double GetToicProbability() const;

  /// @brief update existence state
  void UpdateExistenceState();

 private:
  double existence_score_ = 0.0;
  Dst fused_toic_;
  Dst fused_existence_;
  double toic_score_ = 0.0;

 private:
  static const char *name_;
  static const char *toic_name_;
  static ExistenceDstMaps existence_dst_maps_;
  static ToicDstMaps toic_dst_maps_;
  static DstExistenceFusionOptions options_;

  DISALLOW_COPY_AND_ASSIGN(DstExistenceFusion);
};

}  // namespace fusion
}  // namespace perception
}  // namespace apollo
