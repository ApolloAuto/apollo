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

#include "modules/drivers/ultanalyse/common/ultanalyse_gflags.h"

DEFINE_string(node_name, "ultanalyse", "The chassis module name in proto");
DEFINE_string(module_name, "ultanalyse", "Module name");
DEFINE_string(adapter_config_filename,
             "modules/ultanalyse/conf/ultanalyse.conf",
             "Path for adapter configuration");
DEFINE_bool(ultanalyse_enabled, false,
            "Enable ultanalyse, safe mode activation enabled");
// set the distance,in which sonar_range is effective
DEFINE_double(min_sonar_range_detectable, 20.0,
            "set the min detectable range of ultrasonic sensors");
DEFINE_double(max_sonar_range_detectable, 250.0,
            "set the max detectable range of ultrasonic sensors");
