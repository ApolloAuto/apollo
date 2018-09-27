// Copyright 2018 Baidu Inc. All Rights Reserved.
// @author: Chongchong Li (lichongchong@baidu.com)
// @file: conti_ars_preprocessor.h
// @brief: preprocessor for Continental ARS408-21.

#include "modules/perception/radar/lib/preprocessor/conti_ars_preprocessor/conti_ars_preprocessor.h"
#include "modules/perception/lib/utils/perf.h"

namespace apollo {
namespace perception {
namespace radar {

int ContiArsPreprocessor::current_idx_ = 0;
int ContiArsPreprocessor::local2global_[ORIGIN_CONTI_MAX_ID_NUM] = {0};

bool ContiArsPreprocessor::Init() {
  lib::ConfigManager* config_manager
                 = lib::Singleton<lib::ConfigManager>::get_instance();
  std::string model_name = "ContiArsPreprocessor";
  const lib::ModelConfig* model_config = NULL;
  CHECK(config_manager->GetModelConfig(model_name, &model_config));
  CHECK(model_config->get_value("delay_time", &delay_time_));
  return true;
}

bool ContiArsPreprocessor::Preprocess(
          const ContiRadar& raw_obstacles,
          const PreprocessorOptions& options,
          ContiRadar* corrected_obstacles) {
  PERCEPTION_PERF_FUNCTION();
  SkipObjects(raw_obstacles, corrected_obstacles);
  ExpandIds(corrected_obstacles);
  CorrectTime(corrected_obstacles);
  return true;
}

std::string ContiArsPreprocessor::Name() const {
  return "ContiArsPreprocessor";
}

void ContiArsPreprocessor::SkipObjects(const ContiRadar& raw_obstacles,
                                       ContiRadar* corrected_obstacles) {
  corrected_obstacles->mutable_header()->CopyFrom(raw_obstacles.header());
  double timestamp = raw_obstacles.header().timestamp_sec() - 1e-6;
  for (auto contiobs : raw_obstacles.contiobs()) {
    double object_timestamp = contiobs.header().timestamp_sec();
    if (object_timestamp > timestamp &&
        object_timestamp < timestamp + CONTI_ARS_INTERVAL) {
      ContiRadarObs* obs = corrected_obstacles->add_contiobs();
      *obs = contiobs;
    }
  }
  if (raw_obstacles.contiobs_size() > corrected_obstacles->contiobs_size()) {
    AINFO << "skip objects: "
             << raw_obstacles.contiobs_size()
             << "-> " << corrected_obstacles->contiobs_size();
  }
}

void ContiArsPreprocessor::ExpandIds(ContiRadar* corrected_obstacles) {
  for (int iobj = 0; iobj < corrected_obstacles->contiobs_size(); ++iobj) {
    auto contiobs = corrected_obstacles->contiobs(iobj);
    int id = contiobs.obstacle_id();
    if (CONTI_NEW == contiobs.meas_state()) {
      local2global_[id] = GetNextId();
    } else {
      if (local2global_[id] == 0) {
        local2global_[id] = GetNextId();
      }
    }
    corrected_obstacles->mutable_contiobs(iobj)
                       ->set_obstacle_id(local2global_[id]);
  }
}

void ContiArsPreprocessor::CorrectTime(ContiRadar* corrected_obstacles) {
  double correct_timestamp = corrected_obstacles->header().timestamp_sec()
                           - delay_time_;
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

