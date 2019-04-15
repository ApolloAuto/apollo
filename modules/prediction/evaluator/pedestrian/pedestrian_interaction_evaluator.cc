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

#include "modules/prediction/evaluator/pedestrian/pedestrian_interaction_evaluator.h"

#include <vector>

#include "modules/prediction/common/feature_output.h"
#include "modules/prediction/common/prediction_gflags.h"
#include "modules/prediction/common/prediction_system_gflags.h"

namespace apollo {
namespace prediction {

void PedestrianInteractionEvaluator::Evaluate(Obstacle* obstacle_ptr) {
  // Sanity checks.
  CHECK_NOTNULL(obstacle_ptr);
  int id = obstacle_ptr->id();
  if (!obstacle_ptr->latest_feature().IsInitialized()) {
    AERROR << "Obstacle [" << id << "] has no latest feature.";
    return;
  }
  Feature* latest_feature_ptr = obstacle_ptr->mutable_latest_feature();
  CHECK_NOTNULL(latest_feature_ptr);

  // Extract features, and:
  //  - if in offline mode, save it locally for training.
  //  - if in online mode, pass it through trained model to evaluate.
  std::vector<double> feature_values;
  ExtractFeatures(obstacle_ptr, &feature_values);
  if (FLAGS_prediction_offline_mode == 2) {
    FeatureOutput::InsertDataForLearning(*latest_feature_ptr, feature_values,
                                         "pedestrian", nullptr);
    ADEBUG << "Save extracted features for learning locally.";
    return;
  }
  // TODO(jiacheng): once the model is trained, implement this online part.
}

bool PedestrianInteractionEvaluator::ExtractFeatures(
    const Obstacle* obstacle_ptr, std::vector<double>* feature_values) {
  // Sanity checks.
  CHECK_NOTNULL(obstacle_ptr);
  // int id = obstacle_ptr->id();

  // Extract obstacle related features.
  // TODO(all): implement this.
  return true;
}

}  // namespace prediction
}  // namespace apollo
