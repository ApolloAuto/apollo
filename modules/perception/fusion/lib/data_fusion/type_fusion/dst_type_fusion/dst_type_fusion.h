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
#include <unordered_map>
#include <vector>

#include "modules/perception/fusion/common/dst_evidence.h"
#include "modules/perception/fusion/lib/interface/base_type_fusion.h"

namespace apollo {
namespace perception {
namespace fusion {

struct DstMaps {
  // dst hypothesis types
  enum {
    PEDESTRIAN = (1 << 0),
    BICYCLE = (1 << 1),
    VEHICLE = (1 << 2),
    OTHERS_MOVABLE = (1 << 3),
    OTHERS_UNMOVABLE = (1 << 4)
  };
  enum {
    OTHERS = (OTHERS_MOVABLE | OTHERS_UNMOVABLE),
    UNKNOWN = (PEDESTRIAN | BICYCLE | VEHICLE | OTHERS)
  };

  std::vector<uint64_t> fod_subsets_ = {
      PEDESTRIAN,       BICYCLE, VEHICLE, OTHERS_MOVABLE,
      OTHERS_UNMOVABLE, OTHERS,  UNKNOWN};
  std::vector<std::string> subset_names_ = {
      "PEDESTRIAN",       "BICYCLE", "VEHICLE", "OTHERS_MOVABLE",
      "OTHERS_UNMOVABLE", "OTHERS",  "UNKNOWN"};
  std::unordered_map<size_t, uint64_t> typ_to_hyp_map_ = {
      {static_cast<size_t>(base::ObjectType::PEDESTRIAN), PEDESTRIAN},
      {static_cast<size_t>(base::ObjectType::BICYCLE), BICYCLE},
      {static_cast<size_t>(base::ObjectType::VEHICLE), VEHICLE},
      {static_cast<size_t>(base::ObjectType::UNKNOWN_MOVABLE), OTHERS_MOVABLE},
      {static_cast<size_t>(base::ObjectType::UNKNOWN_UNMOVABLE),
       OTHERS_UNMOVABLE},
      {static_cast<size_t>(base::ObjectType::UNKNOWN), OTHERS},
  };
  std::map<uint64_t, size_t> hyp_to_typ_map_ = {
      {PEDESTRIAN, static_cast<size_t>(base::ObjectType::PEDESTRIAN)},
      {BICYCLE, static_cast<size_t>(base::ObjectType::BICYCLE)},
      {VEHICLE, static_cast<size_t>(base::ObjectType::VEHICLE)},
      {OTHERS_MOVABLE, static_cast<size_t>(base::ObjectType::UNKNOWN_MOVABLE)},
      {OTHERS_UNMOVABLE,
       static_cast<size_t>(base::ObjectType::UNKNOWN_UNMOVABLE)},
      {OTHERS, static_cast<size_t>(base::ObjectType::UNKNOWN)},
      {UNKNOWN, static_cast<size_t>(base::ObjectType::UNKNOWN)}};
};

struct DstTypeFusionOptions {
  std::map<std::string, double> camera_max_valid_dist_ = {
      {"camera_smartereye", 110},
      {"camera_front_obstacle", 110},
      {"front_6mm", 110},
      {"camera_front_narrow", 150},
  };
  std::map<std::string, double> sensor_reliability_ = {
      {"velodyne64", 0.5},          {"velodyne_64", 0.5},
      {"velodyne128", 0.5},         {"camera_smartereye", 0.95},
      {"front_6mm", 0.95},          {"camera_front_obstacle", 0.95},
      {"camera_front_narrow", 0.5},
  };
  std::map<std::string, double> sensor_reliability_for_unknown_ = {
      {"velodyne64", 0.5},          {"velodyne_64", 0.5},
      {"velodyne128", 0.5},         {"camera_smartereye", 0.2},
      {"front_6mm", 0.2},           {"camera_front_obstacle", 0.2},
      {"camera_front_narrow", 0.2},
  };
};

class DstTypeFusion : public BaseTypeFusion {
 public:
  explicit DstTypeFusion(TrackPtr track);
  ~DstTypeFusion() {}

  // @brief: init dst application and options_
  static bool Init();

  // @brief: update track state with measurement
  // @param [in]: measurement
  // @param [in]: target_timestamp
  void UpdateWithMeasurement(const SensorObjectPtr measurement,
                             double target_timestamp) override;

  void UpdateWithoutMeasurement(const std::string &sensor_id,
                                double measurement_timestamp,
                                double target_timestamp,
                                double min_match_dist) override;

  std::string Name() const;

 private:
  bool TypToHyp(size_t object_type, uint64_t *hypothesis_type) const;
  bool HypToTyp(uint64_t hypothesis_type, size_t *object_type) const;
  Dst TypeProbsToDst(const std::vector<float> &type_probs);
  double GetReliability(const std::string &sensor_id) const;
  double GetReliabilityForUnKnown(const std::string &sensor_id,
                                  double measurement_timestamp) const;

  // Update state
  void UpdateTypeState();

 private:
  Dst fused_dst_;

 private:
  static std::string name_;
  // static const char name_[];
  static DstMaps dst_maps_;
  static DstTypeFusionOptions options_;
};

}  // namespace fusion
}  // namespace perception
}  // namespace apollo
