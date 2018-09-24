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
#ifndef PERCEPTION_BASE_VEHICLE_STRUCT_H_
#define PERCEPTION_BASE_VEHICLE_STRUCT_H_
namespace apollo {
namespace perception {
namespace base {
struct CarLight {
  float brake_visible = 0;
  float brake_switch_on = 0;
  float left_turn_visible = 0;
  float left_turn_switch_on = 0;
  float right_turn_visible = 0;
  float right_turn_switch_on = 0;

  void Reset() {
    brake_visible = 0;
    brake_switch_on = 0;
    left_turn_visible = 0;
    left_turn_switch_on = 0;
    right_turn_visible = 0;
    right_turn_switch_on = 0;
  }
};
}  // namespace base
}  // namespace perception
}  // namespace apollo

#endif  // PERCEPTION_BASE_VEHICLE_STRUCT_H_
