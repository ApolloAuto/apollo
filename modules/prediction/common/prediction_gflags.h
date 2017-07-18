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
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *****************************************************************************/

#ifndef MODULES_PREDICTION_COMMON_PREDICTION_GFLAGS_H_
#define MODULES_PREDICTION_COMMON_PREDICTION_GFLAGS_H_

#include "gflags/gflags.h"

// System gflags
DECLARE_string(prediction_module_name);
DECLARE_string(prediction_conf_file);

DECLARE_double(double_precision);
DECLARE_double(max_acc);
DECLARE_double(min_acc);
DECLARE_double(q_var);
DECLARE_double(r_var);
DECLARE_double(p_var);
DECLARE_double(go_approach_rate);
DECLARE_double(cutin_approach_rate);
DECLARE_int32(still_obstacle_history_length);
DECLARE_bool(enable_kf_tracking);
DECLARE_double(still_obstacle_speed_threshold);
DECLARE_double(still_obstacle_position_std);

#endif  // MODULES_PREDICTION_COMMON_PREDICTION_GFLAGS_H_
