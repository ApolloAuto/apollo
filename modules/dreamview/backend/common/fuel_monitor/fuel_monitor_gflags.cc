/******************************************************************************
 * Copyright 2020 The Apollo Authors. All Rights Reserved.
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
#include "modules/dreamview/backend/common/fuel_monitor/fuel_monitor_gflags.h"

DEFINE_string(data_collection_monitor_name, "DataCollectionMonitor",
              "Name of the data collection monitor");

DEFINE_string(preprocess_monitor_name, "PreprocessMonitor",
              "Name of the preprocess monitor");

DEFINE_string(progress_topic, "/apollo/dreamview/progress",
              "Sensor calibration preprocess progress topic name.");
