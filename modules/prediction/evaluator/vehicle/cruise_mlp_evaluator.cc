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

#include "modules/prediction/evaluator/vehicle/cruise_mlp_evaluator.h"
#include "modules/prediction/common/prediction_gflags.h"

namespace apollo {
namespace prediction {

CruiseMLPEvaluator::CruiseMLPEvaluator() {
  LoadModel(FLAGS_evaluator_vehicle_cruise_mlp_file);
}

void CruiseMLPEvaluator::Evaluate(Obstacle* obstacle_ptr) {
  Clear();
  CHECK_NOTNULL(obstacle_ptr);
  int id = obstacle_ptr->id();

  if (!obstacle_ptr->latest_feature().IsInitialized()) {
    AERROR << "Obstacle [" << id << "] has no latest feature.";
    return;
  }
  Feature* latest_feature_ptr = obstacle_ptr->mutable_latest_feature();
  CHECK_NOTNULL(latest_feature_ptr);

  if (!latest_feature_ptr->has_lane() ||
      !latest_feature_ptr->lane().has_lane_graph()) {
    ADEBUG << "Obstacle [" << id << "] has no lane graph.";
    return;
  }
  LaneGraph* lane_graph_ptr =
      latest_feature_ptr->mutable_lane()->mutable_lane_graph();
  CHECK_NOTNULL(lane_graph_ptr);

  if (lane_graph_ptr->lane_sequence_size() == 0) {
    AERROR << "Obstacle [" << id << "] has no lane sequences.";
    return;
  }
  // For every possible lane sequence, extract needed features.
  for (int i = 0; i < lane_graph_ptr->lane_sequence_size(); ++i) {
    LaneSequence* lane_sequence_ptr = lane_graph_ptr->mutable_lane_sequence(i);
    CHECK_NOTNULL(lane_sequence_ptr);
    std::vector<double> feature_values;
    ExtractFeatureValues(obstacle_ptr, lane_sequence_ptr, &feature_values);
    double finish_time = ComputeFinishTime(feature_values);

    double centripetal_acc_probability =
        ValidationChecker::ProbabilityByCentripetalAcceleration(
            *lane_sequence_ptr, speed);
    probability *= centripetal_acc_probability;
    lane_sequence_ptr->set_probability(probability);
  }

  if (FLAGS_prediction_offline_mode) {
    FeatureOutput::Insert(*latest_feature_ptr);
  }
}

void CruiseMLPEvaluator::LoadModel(const std::string& model_file) {
  // Currently, it's using FnnVehicleModel
  // TODO(all) implement it using the generic "network" class.
  model_ptr_.reset(new FnnVehicleModel());
  CHECK(model_ptr_ != nullptr);
  CHECK(common::util::GetProtoFromFile(model_file, model_ptr_.get()))
      << "Unable to load model file: " << model_file << ".";

  AINFO << "Succeeded in loading the model file: " << model_file << ".";
}

double CruiseMLPEvaluator::ComputeFinishTime(
    const std::vector<double>& feature_values) {
  // TODO(all) implement
  return 6.0;
}

}  // namespace prediction
}  // namespace apollo
