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
#include "modules/perception/lidar/lib/classifier/fused_classifier/ccrf_type_fusion.h"

#include "cyber/common/file.h"
#include "cyber/common/log.h"
#include "modules/perception/base/object_types.h"
#include "modules/perception/base/point_cloud.h"
#include "modules/perception/lib/config_manager/config_manager.h"
#include "modules/perception/proto/ccrf_type_fusion_config.pb.h"

namespace apollo {
namespace perception {
namespace lidar {

using ObjectPtr = std::shared_ptr<apollo::perception::base::Object>;
using apollo::cyber::common::GetAbsolutePath;
using apollo::perception::base::ObjectType;

bool CCRFOneShotTypeFusion::Init(const TypeFusionInitOption& option) {
  auto config_manager = lib::ConfigManager::Instance();
  const lib::ModelConfig* model_config = nullptr;
  ACHECK(config_manager->GetModelConfig(Name(), &model_config));
  const std::string work_root = config_manager->work_root();
  std::string config_file;
  std::string root_path;
  ACHECK(model_config->get_value("root_path", &root_path));
  config_file = GetAbsolutePath(work_root, root_path);
  config_file = GetAbsolutePath(config_file, "ccrf_type_fusion.conf");
  CcrfTypeFusionConfig config;
  ACHECK(cyber::common::GetProtoFromFile(config_file, &config));
  std::string classifiers_property_file_path =
      GetAbsolutePath(work_root, config.classifiers_property_file_path());
  ACHECK(util::LoadMultipleMatricesFile(classifiers_property_file_path,
                                        &smooth_matrices_));

  for (auto& pair : smooth_matrices_) {
    util::NormalizeRow(&pair.second);
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

bool CCRFOneShotTypeFusion::TypeFusion(const TypeFusionOption& option,
                                       ObjectPtr object) {
  if (object == nullptr) {
    return false;
  }
  Vectord log_prob;
  if (!FuseOneShotTypeProbs(object, &log_prob)) {
    return false;
  }
  util::ToExp(&log_prob);
  util::Normalize(&log_prob);
  util::FromEigenToVector(log_prob, &(object->type_probs));
  object->type = static_cast<ObjectType>(std::distance(
      object->type_probs.begin(),
      std::max_element(object->type_probs.begin(), object->type_probs.end())));
  return true;
}

bool CCRFOneShotTypeFusion::FuseOneShotTypeProbs(const ObjectPtr& object,
                                                 Vectord* log_prob) {
  if (object == nullptr) {
    return false;
  }
  if (log_prob == nullptr) {
    return false;
  }
  const auto& vecs = object->lidar_supplement.raw_probs;
  const auto& names = object->lidar_supplement.raw_classification_methods;
  if (vecs.empty()) {
    return false;
  }

  log_prob->setZero();

  Vectord single_prob;
  static const Vectord epsilon = Vectord::Ones() * 1e-6;
  float conf = object->confidence;
  for (size_t i = 0; i < vecs.size(); ++i) {
    auto& vec = vecs[i];
    util::FromStdToVector(vec, &single_prob);
    auto iter = smooth_matrices_.find(names[i]);
    if (vecs.size() == 1 || iter == smooth_matrices_.end()) {
      single_prob = single_prob + epsilon;
    } else {
      single_prob = iter->second * single_prob + epsilon;
    }
    util::Normalize(&single_prob);
    // p(c|x) = p(c|x,o)p(o|x)+ p(c|x,~o)p(~o|x)
    single_prob = conf * single_prob +
                  (1.0 - conf) * confidence_smooth_matrix_ * single_prob;
    util::ToLog(&single_prob);
    *log_prob += single_prob;
  }

  return true;
}

bool CCRFSequenceTypeFusion::Init(const TypeFusionInitOption& option) {
  ACHECK(one_shot_fuser_.Init(option));
  auto config_manager = lib::ConfigManager::Instance();
  const lib::ModelConfig* model_config = nullptr;
  ACHECK(config_manager->GetModelConfig(Name(), &model_config));
  const std::string work_root = config_manager->work_root();
  std::string config_file;
  std::string root_path;
  ACHECK(model_config->get_value("root_path", &root_path));
  config_file = GetAbsolutePath(work_root, root_path);
  config_file = GetAbsolutePath(config_file, "ccrf_type_fusion.conf");
  CcrfTypeFusionConfig config;
  ACHECK(cyber::common::GetProtoFromFile(config_file, &config));
  std::string transition_property_file_path =
      GetAbsolutePath(work_root, config.transition_property_file_path());
  s_alpha_ = config.transition_matrix_alpha();
  ACHECK(util::LoadSingleMatrixFile(transition_property_file_path,
                                    &transition_matrix_));
  transition_matrix_ += Matrixd::Ones() * 1e-6;
  util::NormalizeRow(&transition_matrix_);
  AINFO << "transition matrix";
  AINFO << std::endl << transition_matrix_;
  for (std::size_t i = 0; i < VALID_OBJECT_TYPE; ++i) {
    for (std::size_t j = 0; j < VALID_OBJECT_TYPE; ++j) {
      transition_matrix_(i, j) = log(transition_matrix_(i, j));
    }
  }
  AINFO << std::endl << transition_matrix_;
  return true;
}

bool CCRFSequenceTypeFusion::TypeFusion(const TypeFusionOption& option,
                                        TrackedObjects* tracked_objects) {
  if (tracked_objects == nullptr) {
    return false;
  }
  if (tracked_objects->empty()) {
    return false;
  }
  return FuseWithConditionalProbabilityInference(tracked_objects);
}

bool CCRFSequenceTypeFusion::FuseWithConditionalProbabilityInference(
    TrackedObjects* tracked_objects) {
  // AINFO << "Enter fuse with conditional probability inference";
  fused_oneshot_probs_.resize(tracked_objects->size());

  std::size_t i = 0;
  for (auto& pair : *tracked_objects) {
    ObjectPtr& object = pair.second;
    if (!one_shot_fuser_.FuseOneShotTypeProbs(object,
                                              &fused_oneshot_probs_[i++])) {
      AERROR << "Failed to fuse one short probs in sequence.";
      return false;
    }
  }

  // Use viterbi algorithm to infer the state
  std::size_t length = tracked_objects->size();
  fused_sequence_probs_.resize(length);
  state_back_trace_.resize(length);

  fused_sequence_probs_[0] = fused_oneshot_probs_[0];
  // Add priori knowledge to suppress the sudden-appeared object types.
  fused_sequence_probs_[0] += transition_matrix_.row(0).transpose();

  for (std::size_t i = 1; i < length; ++i) {
    for (std::size_t right = 0; right < VALID_OBJECT_TYPE; ++right) {
      double prob = 0.0;
      double max_prob = -DBL_MAX;
      std::size_t id = 0;
      for (std::size_t left = 0; left < VALID_OBJECT_TYPE; ++left) {
        prob = fused_sequence_probs_[i - 1](left) +
               transition_matrix_(left, right) * s_alpha_ +
               fused_oneshot_probs_[i](right);
        if (prob > max_prob) {
          max_prob = prob;
          id = left;
        }
      }
      fused_sequence_probs_[i](right) = max_prob;
      state_back_trace_[i](right) = static_cast<int>(id);
    }
  }
  ObjectPtr object = tracked_objects->rbegin()->second;
  RecoverFromLogProbability(&fused_sequence_probs_.back(), &object->type_probs,
                            &object->type);
  return true;
}

bool CCRFSequenceTypeFusion::RecoverFromLogProbability(Vectord* prob,
                                                       std::vector<float>* dst,
                                                       ObjectType* type) {
  util::ToExpStable(prob);
  util::Normalize(prob);
  util::FromEigenToVector(*prob, dst);
  *type = static_cast<ObjectType>(
      std::distance(dst->begin(), std::max_element(dst->begin(), dst->end())));
  return true;
}

PERCEPTION_REGISTER_ONESHOTTYPEFUSION(CCRFOneShotTypeFusion);
PERCEPTION_REGISTER_SEQUENCETYPEFUSION(CCRFSequenceTypeFusion);

}  // namespace lidar
}  // namespace perception
}  // namespace apollo
