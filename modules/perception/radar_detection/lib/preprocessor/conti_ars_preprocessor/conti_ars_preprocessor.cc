/******************************************************************************
 * Copyright 2018 The Apollo Authors. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the License);
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an AS IS BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *****************************************************************************/
#include "modules/perception/radar_detection/lib/preprocessor/conti_ars_preprocessor/conti_ars_preprocessor.h"

#include "modules/perception/radar_detection/proto/preprocessor.pb.h"

#include "cyber/common/file.h"
#include "cyber/common/log.h"
#include "modules/common/util/perf_util.h"
#include "modules/perception/common/util.h"
#include "modules/perception/radar_detection/common/types.h"

namespace apollo {
namespace perception {
namespace radar {

int ContiArsPreprocessor::current_idx_ = 0;
std::unordered_map<int, int> ContiArsPreprocessor::local2global_;

bool ContiArsPreprocessor::Init(const PreprocessorInitOptions& options) {
  std::string config_file =
      GetConfigFile(options.config_path, options.config_file);

  PreprocessorConfig config;
  ACHECK(cyber::common::GetProtoFromFile(config_file, &config));

  delay_time_ = config.delay_time();
  return true;
}

bool ContiArsPreprocessor::Preprocess(
    const drivers::ContiRadar& raw_obstacles,
    const PreprocessorOptions& options,
    drivers::ContiRadar* corrected_obstacles) {
  PERF_FUNCTION();
  SkipObjects(raw_obstacles, corrected_obstacles);
  ExpandIds(corrected_obstacles);
  CorrectTime(corrected_obstacles);
  return true;
}

void ContiArsPreprocessor::SkipObjects(
    const drivers::ContiRadar& raw_obstacles,
    drivers::ContiRadar* corrected_obstacles) {
  corrected_obstacles->mutable_header()->CopyFrom(raw_obstacles.header());
  double timestamp = raw_obstacles.header().timestamp_sec() - 1e-6;
  for (const auto& contiobs : raw_obstacles.contiobs()) {
    double object_timestamp = contiobs.header().timestamp_sec();
    if (object_timestamp > timestamp &&
        object_timestamp < timestamp + CONTI_ARS_INTERVAL) {
      drivers::ContiRadarObs* obs = corrected_obstacles->add_contiobs();
      *obs = contiobs;
    }
  }
  if (raw_obstacles.contiobs_size() > corrected_obstacles->contiobs_size()) {
    AINFO << "skip objects: " << raw_obstacles.contiobs_size() << "-> "
          << corrected_obstacles->contiobs_size();
  }
}

void ContiArsPreprocessor::ExpandIds(drivers::ContiRadar* corrected_obstacles) {
  for (int iobj = 0; iobj < corrected_obstacles->contiobs_size(); ++iobj) {
    const auto& contiobs = corrected_obstacles->contiobs(iobj);
    int id = contiobs.obstacle_id();
    int corrected_id = 0;
    auto ob = local2global_.find(id);
    if (ob != local2global_.end()) {
      if (CONTI_NEW == contiobs.meas_state()) {
        corrected_id = GetNextId();
        ob->second = corrected_id;
      } else {
        corrected_id = ob->second;
      }
    } else {
      corrected_id = GetNextId();
      local2global_.insert({id, corrected_id});
    }
    corrected_obstacles->mutable_contiobs(iobj)->set_obstacle_id(corrected_id);
  }
}

void ContiArsPreprocessor::CorrectTime(
    drivers::ContiRadar* corrected_obstacles) {
  double correct_timestamp =
      corrected_obstacles->header().timestamp_sec() - delay_time_;
  corrected_obstacles->mutable_header()->set_timestamp_sec(correct_timestamp);
}

int ContiArsPreprocessor::GetNextId() {
  ++current_idx_;
  if (MAX_RADAR_IDX == current_idx_) {
    current_idx_ = 1;
  }
  return current_idx_;
}

PERCEPTION_REGISTER_PREPROCESSOR(ContiArsPreprocessor);

}  // namespace radar
}  // namespace perception
}  // namespace apollo
