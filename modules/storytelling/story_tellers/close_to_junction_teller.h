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

#pragma once

#include <string>

#include "modules/planning/proto/planning.pb.h"
#include "modules/storytelling/story_tellers/base_teller.h"

namespace apollo {
namespace storytelling {

class CloseToJunctionTeller : public BaseTeller {
 public:
  void Init() override;
  void Update(Stories* stories) override;

 private:
  void GetOverlaps(const apollo::planning::ADCTrajectory& adc_trajectory);

 private:
  std::string junction_id_;
  CloseToJunction::JunctionType junction_type_;
  double junction_distance_;
  std::string clear_area_id_;
  double clear_area_distance_;
  std::string crosswalk_id_;
  double crosswalk_distance_;
  std::string signal_id_;
  double signal_distance_;
  std::string stop_sign_id_;
  double stop_sign_distance_;
  std::string yield_sign_id_;
  double yield_sign_distance_;
};

}  // namespace storytelling
}  // namespace apollo
