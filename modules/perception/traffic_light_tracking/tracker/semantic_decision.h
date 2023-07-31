/******************************************************************************
 * Copyright 2018 The Apollo Authors. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the License);
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an AS IS BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *****************************************************************************/
#pragma once

#include <string>
#include <vector>

#include "modules/perception/traffic_light_tracking/tracker/proto/semantic.pb.h"

#include "cyber/common/macros.h"
#include "modules/perception/common/camera/common/trafficlight_frame.h"
#include "modules/perception/traffic_light_tracking/interface/base_traffic_light_tracker.h"

namespace apollo {
namespace perception {
namespace trafficlight {

struct HystereticWindow {
  int hysteretic_count = 0;
  base::TLColor hysteretic_color = base::TLColor::TL_UNKNOWN_COLOR;
};

struct SemanticTable {
  double time_stamp = 0.0;
  double last_bright_time_stamp = 0.0;
  double last_dark_time_stamp = 0.0;
  bool blink = false;
  std::string semantic;
  std::vector<int> light_ids;
  base::TLColor color;
  HystereticWindow hystertic_window;
};

class SemanticReviser final : public BaseTrafficLightTracker {
 public:
  /**
   * @brief Construct a new semantic reviser object.
   * 
   */
  SemanticReviser();
  /**
   * @brief Destroy the semantic reviser object.
   * 
   */
  ~SemanticReviser() = default;
  /**
   * @brief Initialize semantic reviser parameters.
   * 
   * @param options 
   * @return true 
   * @return false 
   */
  bool Init(const TrafficLightTrackerInitOptions &options =
                TrafficLightTrackerInitOptions()) override;

  /**
   * @brief Track detected traffic_light.
   * 
   * @param frame 
   * @return true 
   * @return false 
   */
  bool Track(camera::TrafficLightFrame *frame) override;
  /**
   * @brief Correction based on semantic information.
   * 
   * @param semantic_table 
   * @param lights 
   * @return base::TLColor 
   */
  base::TLColor ReviseBySemantic(SemanticTable semantic_table,
                                 std::vector<base::TrafficLightPtr> *lights);
  /**
   * @brief Correction based on timeseries information.
   * 
   * @param time_stamp 
   * @param semantic_table 
   * @param lights 
   */
  void ReviseByTimeSeries(double time_stamp, SemanticTable semantic_table,
                          std::vector<base::TrafficLightPtr> *lights);
  /**
   * @brief Update the traffic light history information.
   * 
   * @param cur 
   * @param lights 
   * @param history 
   */
  void UpdateHistoryAndLights(const SemanticTable &cur,
                              std::vector<base::TrafficLightPtr> *lights,
                              std::vector<SemanticTable>::iterator *history);
  /**
   * @brief Revise and tracking traffic light.
   * 
   * @param lights 
   * @param light_ids 
   * @param dst_color 
   */
  void ReviseLights(std::vector<base::TrafficLightPtr> *lights,
                    const std::vector<int> &light_ids, base::TLColor dst_color);

 private:
  SemanticReviserConfig semantic_param_;

  float revise_time_s_;
  float blink_threshold_s_;
  float non_blink_threshold_s_;
  int hysteretic_threshold_;
  std::vector<SemanticTable> history_semantic_;

  DISALLOW_COPY_AND_ASSIGN(SemanticReviser);
};

}  // namespace trafficlight
}  // namespace perception
}  // namespace apollo
