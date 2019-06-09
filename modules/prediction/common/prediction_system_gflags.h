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

// System gflags
DECLARE_string(prediction_module_name);
DECLARE_string(prediction_conf_file);
DECLARE_string(prediction_adapter_config_filename);
DECLARE_string(prediction_data_dir);
DECLARE_string(offline_feature_proto_file_name);
DECLARE_string(output_filename);
DECLARE_string(extract_feature_type);

DECLARE_bool(prediction_test_mode);
DECLARE_double(prediction_test_duration);

DECLARE_string(prediction_offline_bags);
DECLARE_int32(prediction_offline_mode);
DECLARE_bool(enable_multi_thread);
DECLARE_int32(max_thread_num);
DECLARE_int32(max_caution_thread_num);

// Bag replay timestamp gap
DECLARE_double(replay_timestamp_gap);
DECLARE_int32(max_num_dump_feature);
DECLARE_int32(max_num_dump_dataforlearn);
