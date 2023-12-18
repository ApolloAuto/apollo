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

#include "gflags/gflags.h"

DECLARE_string(dynamic_model_name);

DECLARE_string(fnn_model_path);

DECLARE_string(backward_fnn_model_path);

DECLARE_string(sim_control_module_name);

DECLARE_string(calibration_conf_file);

DECLARE_string(echo_lincoln_conf_file);

DECLARE_int32(reader_pending_queue_size);

// echosim model
DECLARE_string(echosim_license_file);
DECLARE_string(echosim_parameters_file);
DECLARE_double(echosim_simulation_step);
// DECLARE_double(dynamics_echosim_friction_coefficient);

DECLARE_bool(echo_lincoln_switch_model);

DECLARE_bool(enable_steering_latency);

DECLARE_bool(enable_backward_fnn_model);

DECLARE_bool(enable_sim_at_nonauto_mode);

// cascade model
DECLARE_string(torch_gp_model_file);
DECLARE_bool(use_cuda_in_cascade_model);
DECLARE_string(cascade_model_conf_file);
