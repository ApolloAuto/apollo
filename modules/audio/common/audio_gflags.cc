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

#include "modules/audio/common/audio_gflags.h"

DEFINE_int32(cache_signal_time, 3, "The time to cache signal");
DEFINE_string(torch_siren_detection_model,
              "/apollo/modules/audio/data/torch_siren_detection_model.pt",
              "Siren detection model file");

DEFINE_string(audio_records_dir, "", "The dir path to offline cyber records");
DEFINE_string(audio_conf_file,
              "/apollo/modules/audio/conf/audio_conf.pb.txt",
              "Default conf file for audio module");
