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
#include "modules/perception/obstacle/lidar/type_fuser/sequence_type_fuser/sequence_type_fuser.h"
#include "modules/common/log.h"
#include "modules/perception/lib/base/file_util.h"
#include "modules/perception/lib/config_manager/config_manager.h"

namespace apollo {
namespace perception {

bool SequenceTypeFuser::Init() {
  using namespace sequence_type_fuser;

  const ModelConfig* model_config = NULL;
  if (!ConfigManager::instance()->GetModelConfig(name(), &model_config)) {
    AERROR << "Failed to found model config: " << name();
    return false;
  }

  if (!model_config->GetValue("temporal_window", &_temporal_window)) {
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
  const std::string &work_root = ConfigManager::instance()->work_root();
  transition_property_file_path = FileUtil::GetAbsolutePath(work_root, 
          transition_property_file_path);

  if (!LoadSingleMatrixFile(transition_property_file_path, 
            &_transition_matrix)) {
    return false;
  } 
  _transition_matrix += Matrixd::Ones() * 1e-6;
  for (std::size_t i = 0; i < VALID_OBJECT_TYPE; ++i) {
    NormalizeRow(&_transition_matrix);
  }
  AINFO << "transition matrix";
  AINFO << std::endl << _transition_matrix;
  for (std::size_t i = 0; i < VALID_OBJECT_TYPE; ++i) {
    for (std::size_t j = 0; j < VALID_OBJECT_TYPE; ++j) {
      _transition_matrix(i, j) = log(_transition_matrix(i, j));
    }
  }
  AINFO << std::endl << _transition_matrix;

  // get classifier property
  std::string classifiers_property_file_path;
  if (!model_config->GetValue("classifiers_property_file_path", 
            &classifiers_property_file_path)) {
    AERROR << "Failed to find classifiers_property_file_path in config. ";
    return false;
  }
  classifiers_property_file_path = FileUtil::GetAbsolutePath(work_root, 
          classifiers_property_file_path);

  if (!LoadMultipleMatricesFile(classifiers_property_file_path, &_smooth_matrices)) {
    return false;
  }
  for (auto& pair : _smooth_matrices) {
    NormalizeRow(&pair.second);
    pair.second.transposeInPlace();
    AINFO << "Source: " << pair.first;
    AINFO << std::endl << pair.second;
  }

  _confidence_smooth_matrix = Matrixd::Identity();
  auto iter = _smooth_matrices.find("Confidence");
  if (iter != _smooth_matrices.end()) {
    _confidence_smooth_matrix = iter->second; 
    _smooth_matrices.erase(iter);
  }
  AINFO << "Confidence: ";
  AINFO << std::endl << _confidence_smooth_matrix;

  _ccrf_debug = true;
  return true;
}

bool SequenceTypeFuser::FuseType(
            const TypeFuserOptions& options,
            std::vector<ObjectPtr>* objects) {
  if (objects == nullptr) {
    return false;
  }
  if (options.timestamp > 0.0) {
    AINFO << "Combined classifier, temporal fusion";
    // sequence fusion
    _sequence.AddTrackedFrameObjects(*objects, options.timestamp);
    ObjectSequence::TrackedObjects tracked_objects;
    for (auto& object : *objects) {
      if (object->is_background) {
        object->type_probs.assign(MAX_OBJECT_TYPE, 0);
        object->type = UNKNOWN_UNMOVABLE;
        continue;
      }
      const int& track_id = object->track_id;
      _sequence.GetTrackInTemporalWindow(
              track_id, &tracked_objects, _temporal_window);
      //CHECK(tracked_objects.size() > 0) << "Empty track found";
      //CHECK(object == tracked_objects.rbegin()->second) << "Inconsist objects found";
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
        TrackedObjects* tracked_objects) {

  if (tracked_objects == nullptr || tracked_objects->size() == 0) {
    return false;
  }

  // _current_timestamp = option.timestamp;

  //LOG_INFO << "Enter fuse with conditional probability inference";
  _fused_oneshot_probs.resize(tracked_objects->size());

  std::size_t i = 0;
  for (auto& pair : *tracked_objects) {
    ObjectPtr& object = pair.second;
    if (!RectifyObjectType(object, &_fused_oneshot_probs[i++])) {
      AERROR << "Failed to fuse one short probs in sequence.";
      return false;
    }
  }

  // Use viterbi algorithm to infer the state
  std::size_t length = tracked_objects->size();
  _fused_sequence_probs.resize(length);
  _state_back_trace.resize(length);

  _fused_sequence_probs[0] = _fused_oneshot_probs[0];
  // Add prior knowledge to suppress the sudden-appeared object types.
  _fused_sequence_probs[0] += _transition_matrix.row(0).transpose();

  for (std::size_t i = 1; i < length; ++i) {
    for (std::size_t right = 0; right < VALID_OBJECT_TYPE; ++right) {
      double prob = 0.0;
      double max_prob = -DBL_MAX;
      std::size_t id = 0;
      for (std::size_t left = 0; left < VALID_OBJECT_TYPE; ++left) {
        prob = _fused_sequence_probs[i-1](left) 
            + _transition_matrix(left, right) * _s_alpha
            + _fused_oneshot_probs[i](right);
        if (prob > max_prob) {
          max_prob = prob;
          id = left;
        }
      }
      _fused_sequence_probs[i](right) = max_prob;
      _state_back_trace[i](right) = id;
    }
  }
  ObjectPtr object = tracked_objects->rbegin()->second;
  RecoverFromLogProb(&_fused_sequence_probs.back(), 
          &object->type_probs, &object->type);

  if (_ccrf_debug) {
    ObjectPtr object = tracked_objects->rbegin()->second;
    std::cout << "Track id: " << object->track_id 
        << " length: " << tracked_objects->size() << std::endl;
    // for (std::size_t i = 0; i < object->lidar_supplement->raw_probs.size(); ++i) {
    //     sequence_type_fuser::PrintProbability(object->lidar_supplement->raw_probs[i],
    //             object->lidar_supplement->raw_classification_methods[i]);
    // }
    std::vector<float> prob_t;
    sequence_type_fuser::ToExp(&_fused_oneshot_probs.back());
    sequence_type_fuser::Normalize(&_fused_oneshot_probs.back());
    sequence_type_fuser::FromEigenVector(_fused_oneshot_probs.back(), &prob_t);
    sequence_type_fuser::PrintProbability(prob_t, "Oneshot");
    sequence_type_fuser::PrintProbability(object->type_probs, "Sequence");
    std::cout << "@@@@@@@@@@@@@@@@@" << std::endl;
  }

  return true;
}

bool SequenceTypeFuser::RectifyObjectType(const ObjectPtr& object, 
        Vectord* log_prob) {
  using namespace sequence_type_fuser;

  if (object == nullptr || log_prob == nullptr) {
    return false;
  }

  log_prob->setZero();

  Vectord single_prob;
  FromStdVector(object->type_probs, &single_prob);
  auto iter = _smooth_matrices.find("CNNSegClassifier");
  if (iter == _smooth_matrices.end()) {
    AERROR << "Failed to find CNNSegmentation classifier property.";
    return false;
  }
  static const Vectord epsilon = Vectord::Ones() * 1e-6;
  single_prob = iter->second * single_prob + epsilon;
  Normalize(&single_prob);

  double conf = object->score;
  single_prob = conf * single_prob 
          + (1.0 - conf) * _confidence_smooth_matrix * single_prob;
  ToLog(&single_prob);
  *log_prob += single_prob;
  return true;
}

bool SequenceTypeFuser::RecoverFromLogProb(Vectord* prob, 
        std::vector<float>* dst, ObjectType* type) {
  using namespace sequence_type_fuser;

  ToExp(prob);
  Normalize(prob);
  FromEigenVector(*prob, dst);
  *type = static_cast<ObjectType>(
          std::distance(dst->begin(),
              std::max_element(dst->begin(), dst->end())));
  return true;
}

// REGISTER_ONESHOTTYPEFUSION(CCRFOneShotTypeFusion);
// REGISTER_SEQUENCETYPEFUSION(CCRFSequenceTypeFusion);

}  // namespace perception
}  // namespace apollo
    
    
