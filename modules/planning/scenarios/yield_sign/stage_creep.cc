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

/**
 * @file
 **/
#include "modules/planning/scenarios/yield_sign/stage_creep.h"

#include <string>

#include "modules/common_msgs/perception_msgs/perception_obstacle.pb.h"
#include "cyber/common/log.h"
#include "cyber/time/clock.h"
#include "modules/common/vehicle_state/vehicle_state_provider.h"
#include "modules/map/pnc_map/path.h"
#include "modules/planning/planning_base/common/frame.h"
#include "modules/planning/planning_base/common/planning_context.h"
#include "modules/planning/planning_base/common/speed_profile_generator.h"
#include "modules/planning/planning_base/common/util/util.h"

namespace apollo {
namespace planning {

using apollo::common::TrajectoryPoint;
using apollo::cyber::Clock;
using apollo::hdmap::PathOverlap;

bool YieldSignStageCreep::Init(
    const StagePipeline& config,
    const std::shared_ptr<DependencyInjector>& injector,
    const std::string& config_dir, void* context) {
  CHECK_NOTNULL(context);
  bool ret = Stage::Init(config, injector, config_dir, context);
  if (!ret) {
    AERROR << Name() << "init failed!";
    return false;
  }
  return ret;
}

StageResult YieldSignStageCreep::Process(
    const TrajectoryPoint& planning_init_point, Frame* frame) {
  ADEBUG << "stage: Creep";
  CHECK_NOTNULL(frame);

  auto scenario_context = GetContextAs<YieldSignContext>();
  scenario_config_.CopyFrom(scenario_context->scenario_config);

  if (!pipeline_config_.enabled()) {
    return FinishStage();
  }

  // Run creep decider.
  for (auto& reference_line_info : *frame->mutable_reference_line_info()) {
    if (!reference_line_info.IsDrivable()) {
      AERROR << "The generated path is not drivable";
      break;
    }

    const auto ret = ProcessCreep(frame, &reference_line_info);
    if (!ret.ok()) {
      AERROR << "Failed to run CreepDecider ], Error message: "
             << ret.error_message();
      break;
    }
  }

  StageResult result = ExecuteTaskOnReferenceLine(planning_init_point, frame);
  if (result.HasError()) {
    AERROR << "YieldSignStageCreep planning error";
  }

  if (scenario_context->current_yield_sign_overlap_ids.empty()) {
    return FinishScenario();
  }

  const auto& reference_line_info = frame->reference_line_info().front();
  const std::string yield_sign_overlap_id =
      scenario_context->current_yield_sign_overlap_ids[0];

  // get overlap along reference line
  PathOverlap* current_yield_sign_overlap =
      reference_line_info.GetOverlapOnReferenceLine(
          yield_sign_overlap_id, ReferenceLineInfo::YIELD_SIGN);
  if (!current_yield_sign_overlap) {
    return FinishScenario();
  }

  // set right_of_way_status
  const double yield_sign_start_s = current_yield_sign_overlap->start_s;
  reference_line_info.SetJunctionRightOfWay(yield_sign_start_s, false);

  const double yield_sign_end_s = current_yield_sign_overlap->end_s;
  const double wait_time =
      Clock::NowInSeconds() - scenario_context->creep_start_time;
  const double timeout_sec = scenario_config_.creep_timeout_sec();

  double creep_stop_s =
      GetCreepFinishS(yield_sign_end_s, *frame, reference_line_info);
  const double distance =
      creep_stop_s - reference_line_info.AdcSlBoundary().end_s();
  if (distance <= 0.0) {
    auto& rfl_info = frame->mutable_reference_line_info()->front();
    *(rfl_info.mutable_speed_data()) =
        SpeedProfileGenerator::GenerateFixedDistanceCreepProfile(0.0, 0);
  }

  if (CheckCreepDone(*frame, reference_line_info, yield_sign_end_s, wait_time,
                     timeout_sec)) {
    return FinishStage();
  }

  return result.SetStageStatus(StageStatusType::RUNNING);
}

const CreepStageConfig& YieldSignStageCreep::GetCreepStageConfig() const {
  return GetContextAs<YieldSignContext>()->scenario_config.creep_stage_config();
}

bool YieldSignStageCreep::GetOverlapStopInfo(
    Frame* frame, ReferenceLineInfo* reference_line_info, double* overlap_end_s,
    std::string* overlap_id) const {
  std::string yield_sign_overlap_id;
  if (injector_->planning_context()
          ->planning_status()
          .yield_sign()
          .current_yield_sign_overlap_id_size() > 0) {
    yield_sign_overlap_id = injector_->planning_context()
                                ->planning_status()
                                .yield_sign()
                                .current_yield_sign_overlap_id(0);
  }

  if (!yield_sign_overlap_id.empty()) {
    // get overlap along reference line
    PathOverlap* current_yield_sign_overlap =
        reference_line_info->GetOverlapOnReferenceLine(
            yield_sign_overlap_id, ReferenceLineInfo::YIELD_SIGN);
    if (current_yield_sign_overlap) {
      *overlap_end_s = current_yield_sign_overlap->end_s;
      *overlap_id = yield_sign_overlap_id;
      return true;
    }
  }
  return false;
}

StageResult YieldSignStageCreep::FinishStage() { return FinishScenario(); }

}  // namespace planning
}  // namespace apollo
