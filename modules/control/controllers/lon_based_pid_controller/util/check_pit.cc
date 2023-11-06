/******************************************************************************
 * Copyright 2023 The Apollo Authors. All Rights Reserved.
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
#include "modules/control/controllers/lon_based_pid_controller/util/check_pit.h"

namespace apollo {
namespace control {


bool CheckPit::CheckInPit(SimpleLongitudinalDebug* debug,
                          const LonBasedPidControllerConf* conf,
                          double speed,
                          bool replan) {
  static std::pair<double, int> replan_count(0, 0);
  // static double pre_station_error = 0;

  double now = apollo::cyber::Clock::NowInSeconds();

  if (now - replan_count.first > conf->pit_replan_check_time()) {
    replan_count.first = 0;
    replan_count.second = 0;
  }

  if (replan) {
    if (now - replan_count.first > conf->pit_replan_check_time()) {
      replan_count.first = now;
      replan_count.second = 1;
    } else {
      replan_count.first = now;
      replan_count.second++;
    }
  }

  const auto& vehicle_param =
      common::VehicleConfigHelper::GetConfig().vehicle_param();
  if (replan_count.second >= conf->pit_replan_check_count() &&
        abs(speed) < vehicle_param.max_abs_speed_when_stopped()) {
    return true;
  } else {
    return false;
  }
}

}  // namespace control
}  // namespace apollo
