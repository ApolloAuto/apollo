/******************************************************************************
 * Copyright 2017 The Apollo Authors. All Rights Reserved.
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

#include "modules/perception/common/sequence_type_fuser/sequence_type_fuser.h"

#include "modules/common/log.h"
#include "modules/common/util/file.h"
#include "modules/perception/lib/config_manager/config_manager.h"

namespace apollo {
namespace perception {

using apollo::common::util::GetAbsolutePath;

bool SequenceTypeFuser::Init() {
  const ModelConfig* model_config =
      ConfigManager::instance()->GetModelConfig(name());
  if (model_config == nullptr) {
    AERROR << "Failed to found model config: " << name();
    return false;
  }

  if (!model_config->GetValue("temporal_window", &temporal_window_)) {
    AERROR << "Failed to find temporal_window in config. ";
    return false;
  }

  // get the transition matrix
  std::string transition_property_file_path;
  if (!model_config->GetValue("transition_property_file_path",
                              &transition_property_file_path)) {
    AERROR << "Failed to find transition_property_file_path in config. ";
    return false;
  }
  const std::string& work_root = ConfigManager::instance()->WorkRoot();
  transition_property_file_path =
      GetAbsolutePath(work_root, transition_property_file_path);

  if (!fuser_util::LoadSingleMatrixFile(transition_property_file_path,
                                        &transition_matrix_)) {
    return false;
  }
  transition_matrix_ += Matrixd::Ones() * 1e-6;
  for (std::size_t i = 0; i < VALID_OBJECT_TYPE; ++i) {
    fuser_util::NormalizeRow(&transition_matrix_);
  }
  AINFO << "transition matrix";
  AINFO << std::endl << transition_matrix_;
  for (std::size_t i = 0; i < VALID_OBJECT_TYPE; ++i) {
    for (std::size_t j = 0; j < VALID_OBJECT_TYPE; ++j) {
      transition_matrix_(i, j) = log(transition_matrix_(i, j));
    }
  }
  AINFO << std::endl << transition_matrix_;

  // get classifier property
  std::string classifiers_property_file_path;
  if (!model_config->GetValue("classifiers_property_file_path",
                              &classifiers_property_file_path)) {
    AERROR << "Failed to find classifiers_property_file_path in config. ";
    return false;
  }
  classifiers_property_file_path =
      GetAbsolutePath(work_root, classifiers_property_file_path);

  if (!fuser_util::LoadMultipleMatricesFile(classifiers_property_file_path,
                                            &smooth_matrices_)) {
    return false;
  }
  for (auto& pair : smooth_matrices_) {
    fuser_util::NormalizeRow(&pair.second);
    pair.second.transposeInPlace();
    AINFO << "Source: " << pair.first;
    AINFO << std::endl << pair.second;
  }

  confidence_smooth_matrix_ = Matrixd::Identity();
  auto iter = smooth_matrices_.find("Confidence");
  if (iter != smooth_matrices_.end()) {
    confidence_smooth_matrix_ = iter->second;
    smooth_matrices_.erase(iter);
  }
  AINFO << "Confidence: ";
  AINFO << std::endl << confidence_smooth_matrix_;

  return true;
}

bool SequenceTypeFuser::FuseType(
    const TypeFuserOptions& options,
    std::vector<std::shared_ptr<Object>>* objects) {
  if (objects == nullptr) {
    return false;
  }
  if (options.timestamp > 0.0) {
    sequence_.AddTrackedFrameObjects(*objects, options.timestamp);
    std::map<int64_t, std::shared_ptr<Object>> tracked_objects;
    for (auto& object : *objects) {
      if (object->is_background) {
        object->type_probs.assign(static_cast<int>(ObjectType::MAX_OBJECT_TYPE),
                                  0);
        object->type = ObjectType::UNKNOWN_UNMOVABLE;
        continue;
      }
      const int& track_id = object->track_id;
      sequence_.GetTrackInTemporalWindow(track_id, &tracked_objects,
                                         temporal_window_);
      if (tracked_objects.size() == 0) {
        AERROR << "Find zero-length track, so skip.";
        continue;
      }
      if (object != tracked_objects.rbegin()->second) {
        AERROR << "There must exist some timestamp in disorder, so skip.";
        continue;
      }
      if (!FuseWithCCRF(&tracked_objects)) {
        AERROR << "Failed to fuse types, so break.";
        break;
      }
    }
  }
  return true;
}

bool SequenceTypeFuser::FuseWithCCRF(
    std::map<int64_t, std::shared_ptr<Object>>* tracked_objects) {
  if (tracked_objects == nullptr || tracked_objects->size() == 0) {
    return false;
  }

  /// rectify object type with smooth matrices
  fused_oneshot_probs_.resize(tracked_objects->size());
  std::size_t i = 0;
  for (auto& pair : *tracked_objects) {
    std::shared_ptr<Object>& object = pair.second;
    if (!RectifyObjectType(object, &fused_oneshot_probs_[i++])) {
      AERROR << "Failed to fuse one shot probs in sequence.";
      return false;
    }
  }

  /// use Viterbi algorithm to infer the state
  std::size_t length = tracked_objects->size();
  fused_sequence_probs_.resize(length);
  state_back_trace_.resize(length);
  fused_sequence_probs_[0] = fused_oneshot_probs_[0];
  /// add prior knowledge to suppress the sudden-appeared object types.
  fused_sequence_probs_[0] += transition_matrix_.row(0).transpose();
  for (std::size_t i = 1; i < length; ++i) {
    for (std::size_t right = 0; right < VALID_OBJECT_TYPE; ++right) {
      double max_prob = -DBL_MAX;
      std::size_t id = 0;
      for (std::size_t left = 0; left < VALID_OBJECT_TYPE; ++left) {
        const double prob = fused_sequence_probs_[i - 1](left) +
                            transition_matrix_(left, right) * s_alpha_ +
                            fused_oneshot_probs_[i](right);
        if (prob > max_prob) {
          max_prob = prob;
          id = left;
        }
      }
      fused_sequence_probs_[i](right) = max_prob;
      state_back_trace_[i](right) = id;
    }
  }
  std::shared_ptr<Object> object = tracked_objects->rbegin()->second;
  RecoverFromLogProb(&fused_sequence_probs_.back(), &object->type_probs,
                     &object->type);

  return true;
}

bool SequenceTypeFuser::RectifyObjectType(const std::shared_ptr<Object>& object,
                                          Vectord* log_prob) {
  if (object == nullptr || log_prob == nullptr) {
    return false;
  }

  log_prob->setZero();

  Vectord single_prob;
  fuser_util::FromStdVector(object->type_probs, &single_prob);
  auto iter = smooth_matrices_.find("CNNSegClassifier");
  if (iter == smooth_matrices_.end()) {
    AERROR << "Failed to find CNNSegmentation classifier property.";
    return false;
  }
  static const Vectord epsilon = Vectord::Ones() * 1e-6;
  single_prob = iter->second * single_prob + epsilon;
  fuser_util::Normalize(&single_prob);

  double conf = object->score;
  single_prob = conf * single_prob +
                (1.0 - conf) * confidence_smooth_matrix_ * single_prob;
  fuser_util::ToLog(&single_prob);
  *log_prob += single_prob;
  return true;
}

bool SequenceTypeFuser::RecoverFromLogProb(Vectord* prob,
                                           std::vector<float>* dst,
                                           ObjectType* type) {
  fuser_util::ToExp(prob);
  fuser_util::Normalize(prob);
  fuser_util::FromEigenVector(*prob, dst);
  *type = static_cast<ObjectType>(
      std::distance(dst->begin(), std::max_element(dst->begin(), dst->end())));
  return true;
}

}  // namespace perception
}  // namespace apollo
