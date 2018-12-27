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

/**
 * @file predictor_perception.h
 * @brief The class of PredictorPerception.
 */

#ifndef MODULES_LOCALIZATION_LMD_PREDICTOR_PERCEPTION_PREDICTOR_PERCEPTION_H_
#define MODULES_LOCALIZATION_LMD_PREDICTOR_PERCEPTION_PREDICTOR_PERCEPTION_H_

#include <limits>
#include <vector>

#include "modules/perception/proto/perception_obstacle.pb.h"

#include "modules/localization/lmd/common/tm_list.h"
#include "modules/localization/lmd/predictor/perception/lm_provider.h"
#include "modules/localization/lmd/predictor/perception/lm_sampler.h"
#include "modules/localization/lmd/predictor/perception/pc_map.h"
#include "modules/localization/lmd/predictor/perception/pc_registrator.h"
#include "modules/localization/lmd/predictor/predictor.h"

/**
 * @namespace apollo::localization
 * @brief apollo::localization
 */
namespace apollo {
namespace localization {

/**
 * @class PredictorPerception
 *
 * @brief  Implementation of predictor.
 */
class PredictorPerception : public Predictor {
 public:
  explicit PredictorPerception(double memory_cycle_sec);
  virtual ~PredictorPerception();

  /**
   * @brief Update lane markers from perception.
   * @param timestamp_sec The timestamp of lane markers.
   * @param lane_markers The lane markers.
   * @return True if success; false if not needed.
   */
  bool UpdateLaneMarkers(double timestamp_sec,
                         const apollo::perception::LaneMarkers &lane_markers);

  /**
   * @brief Overrided implementation of the virtual function "Updateable" in the
   * base class "Predictor".
   * @return True if yes; no otherwise.
   */
  bool Updateable() const override;

  /**
   * @brief Overrided implementation of the virtual function "Update" in the
   * base class "Predictor".
   * @return Status::OK() if success; error otherwise.
   */
  apollo::common::Status Update() override;

 private:
  double DepsTimestamp() const;

 private:
  LMSampler lm_sampler_;
  LMProvider lm_provider_;
  PCMap pc_map_;
  PCRegistrator pc_registrator_;

  TimeMarkedList<std::vector<PCSourcePoint>> lane_markers_samples_;
  double deps_timestamp_sec_ = std::numeric_limits<double>::min();
};

}  // namespace localization
}  // namespace apollo

#endif  // MODULES_LOCALIZATION_LMD_PREDICTOR_PERCEPTION_PREDICTOR_PERCEPTION_H_
