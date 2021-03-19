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

#include "modules/data/tools/smart_recorder/smart_recorder_gflags.h"

DEFINE_string(source_records_dir, "", "The source dir with original records.");
DEFINE_string(restored_output_dir, "", "The output dir after processing.");
DEFINE_string(smart_recorder_config_filename,
              "/apollo/modules/data/tools/smart_recorder/conf/"
              "smart_recorder_config.pb.txt",
              "The config file.");
DEFINE_bool(real_time_trigger, true, "Whether to use realtime trigger.");
