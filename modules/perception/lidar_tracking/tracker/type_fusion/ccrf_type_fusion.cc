/******************************************************************************
 * Copyright 2024 The Apollo Authors. All Rights Reserved.
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

#include <limits>

#include "cyber/common/file.h"
#include "cyber/common/log.h"

#include "modules/perception/common/util.h"
#include "modules/perception/common/base/object_types.h"
#include "modules/perception/common/base/point_cloud.h"

#include "modules/perception/lidar_tracking/tracker/type_fusion/ccrf_type_fusion.h"
#include "modules/perception/lidar_tracking/tracker/type_fusion/proto/ccrf_type_filter_config.pb.h"

namespace apollo {
namespace perception {
namespace lidar {

using ObjectPtr = std::shared_ptr<apollo::perception::base::Object>;
using apollo::perception::base::ObjectType;

bool CCRFSingleShotTypeFusion::Init(const TypeFilterInitOption& options) {
    std::string config_file = "ccrf_type_fusion.conf";
    if (!options.config_file.empty()) {
        config_file = options.config_file;
    }
    config_file = GetConfigFile(options.config_path, config_file);

    CCRFTypeFusionConfig config;
    ACHECK(cyber::common::GetProtoFromFile(config_file, &config));
    std::string classifiers_property_file_path
            = GetConfigFile(options.config_path,
                config.classifiers_property_file_path());
    debug_log_ = options.debug_log;
    ACHECK(type_util::LoadMultipleMatricesFile(classifiers_property_file_path,
        &smooth_matrices_));

    for (auto& pair : smooth_matrices_) {
        type_util::NormalizeRow(&pair.second);
        pair.second.transposeInPlace();
        AINFO << "Source: " << pair.first << std::endl << pair.second;
    }

    confidence_smooth_matrix_ = Matrixd::Identity();
    auto iter = smooth_matrices_.find("Confidence");
    if (iter != smooth_matrices_.end()) {
        confidence_smooth_matrix_ = iter->second;
        smooth_matrices_.erase(iter);
    }
    AINFO << "ConfSmoothMatrix: " << std::endl << confidence_smooth_matrix_;

    return true;
}

bool CCRFSingleShotTypeFusion::TypeFusion(const TypeFilterOption& option,
    TrackedObjectPtr new_object) {
    Vectord log_prob;
    if (!FuseOneShotTypeProbs(new_object, &log_prob)) {
        return false;
    }
    type_util::ToExp(&log_prob);
    type_util::Normalize(&log_prob);
    type_util::FromEigenToVector(log_prob, &(new_object->type_probs));
    new_object->type = static_cast<ObjectType>(std::distance(
        new_object->type_probs.begin(),
        std::max_element(new_object->type_probs.begin(),
                new_object->type_probs.end())));
    return true;
}

bool CCRFSingleShotTypeFusion::FuseOneShotTypeProbs(
    const TrackedObjectConstPtr& new_object, Vectord* log_prob) {
    const auto& object = new_object->object_ptr;
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
        // avoid parsing-wrong-type-probs
        size_t type_count = static_cast<size_t>(ObjectType::MAX_OBJECT_TYPE);
        if (vec.size() != type_count) {
            AERROR << "[TypeFilter] Encounter Unknown type_probs";
            continue;
        }
        type_util::FromStdToVector(vec, &single_prob);
        auto iter = smooth_matrices_.find(names[i]);
        if (iter == smooth_matrices_.end()) {
            single_prob = single_prob + epsilon;
        } else {
            single_prob = iter->second * single_prob + epsilon;
        }
        type_util::Normalize(&single_prob);
        // p(c|x) = p(c|x,o)p(o|x)+ p(c|x,~o)p(~o|x)
        Vectord tmp = confidence_smooth_matrix_ * single_prob;
        type_util::Normalize(&tmp);
        single_prob = conf * single_prob + (1.0 - conf) * tmp;

        type_util::Normalize(&single_prob);
        type_util::ToLog(&single_prob);
        *log_prob += single_prob;
    }

    return true;
}

bool CCRFMultiShotTypeFusion::Init(const TypeFilterInitOption& options) {
    TypeFilterInitOption one_shot_fuser_options;
    one_shot_fuser_options.config_path = options.config_path;
    ACHECK(one_shot_fuser_.Init(one_shot_fuser_options));

    std::string config_file = "ccrf_type_fusion.conf";
    if (!options.config_file.empty()) {
        config_file = options.config_file;
    }
    config_file = GetConfigFile(options.config_path, config_file);
    CCRFTypeFusionConfig config;
    ACHECK(cyber::common::GetProtoFromFile(config_file, &config));

    std::string transition_property_file_path
            = GetConfigFile(options.config_path,
                config.transition_property_file_path());
    s_alpha_ = config.transition_matrix_alpha();
    debug_log_ = options.debug_log;
    ACHECK(type_util::LoadSingleMatrixFile(transition_property_file_path,
        &transition_matrix_));
    transition_matrix_ += Matrixd::Ones() * 1e-6;
    for (std::size_t i = 0; i < VALID_OBJECT_TYPE; ++i) {
        type_util::NormalizeRow(&transition_matrix_);
    }
    AINFO << "TypeTransitionMatrix: " << std::endl << transition_matrix_;

    for (std::size_t i = 0; i < VALID_OBJECT_TYPE; ++i) {
        for (std::size_t j = 0; j < VALID_OBJECT_TYPE; ++j) {
            transition_matrix_(i, j) = log(transition_matrix_(i, j));
        }
    }
    AINFO << "TypeTransitionMatrix(Log): " << std::endl << transition_matrix_;
    return true;
}

bool CCRFMultiShotTypeFusion::TypeFusion(
        const TypeFilterOption& option,
        const std::vector<TrackedObjectConstPtr>& tracked_objects,
        TrackedObjectPtr new_object) {
    return FuseWithConditionalProbabilityInference(tracked_objects,
        new_object);
}

bool CCRFMultiShotTypeFusion::FuseWithConditionalProbabilityInference(
        const std::vector<TrackedObjectConstPtr>& tracked_objects,
        TrackedObjectPtr new_object) {
    fused_oneshot_probs_.resize(tracked_objects.size() + 1);
    std::size_t i = 0;
    for (const auto& obj : tracked_objects) {
        // ObjectPtr& object = pair.second;
        if (!one_shot_fuser_.FuseOneShotTypeProbs(obj,
            &fused_oneshot_probs_[i++])) {
            AERROR << "Failed to fuse one short probs in sequence.";
            return false;
        }
    }
    if (!one_shot_fuser_.FuseOneShotTypeProbs(new_object,
        &fused_oneshot_probs_[i++])) {
        AERROR << "Failed to fuse one short probs for new object.";
        return false;
    }
    // Use viterbi algorithm to infer the state
    std::size_t length = fused_oneshot_probs_.size();
    fused_sequence_probs_.resize(length);
    state_back_trace_.resize(length);

    fused_sequence_probs_[0] = fused_oneshot_probs_[0];
    // Add priori knowledge to suppress the sudden-appeared object types.
    // fused_sequence_probs_[0] += transition_matrix_.row(0).transpose();

    for (std::size_t i = 1; i < length; ++i) {
        for (std::size_t right = 0; right < VALID_OBJECT_TYPE; ++right) {
            double prob = 0.0;
            double max_prob = -std::numeric_limits<double>::max();
            std::size_t id = 0;
            for (std::size_t left = 0; left < VALID_OBJECT_TYPE; ++left) {
                prob = fused_sequence_probs_[i - 1](left)
                       + transition_matrix_(left, right) * s_alpha_
                       + fused_oneshot_probs_[i](right);
                if (prob > max_prob) {
                    max_prob = prob;
                    id = left;
                }
            }
            fused_sequence_probs_[i](right) = max_prob;
            state_back_trace_[i](right) = static_cast<int>(id);
        }
        type_util::ToExp(&fused_sequence_probs_[i]);
        type_util::Normalize(&fused_sequence_probs_[i]);
        type_util::ToLog(&fused_sequence_probs_[i]);
    }
    // ObjectPtr object = tracked_objects->rbegin()->second;
    RecoverFromLogProbability(&fused_sequence_probs_.back(),
        &new_object->type_probs, &new_object->type);
    if (debug_log_) {
        std::stringstream sstr;
        sstr << " total_size: " << fused_oneshot_probs_.size();
        for (size_t i = 0; i < fused_oneshot_probs_.size(); i++) {
            Vectord tmp = fused_oneshot_probs_[i];
            type_util::ToExpStable(&tmp);
            type_util::Normalize(&tmp);
            sstr << " After-Smooth[" << std::to_string(i) << "]: ";
            for (size_t j = 0; j < 4; j++) {
                sstr << tmp(j) << ", ";
            }
        }
        ADEBUG << sstr.str();
    }
    return true;
}

bool CCRFMultiShotTypeFusion::RecoverFromLogProbability(Vectord* prob,
    std::vector<float>* dst, ObjectType* type) {
    type_util::ToExpStable(prob);
    type_util::Normalize(prob);
    type_util::FromEigenToVector(*prob, dst);
    *type = static_cast<ObjectType>(std::distance(dst->begin(),
        std::max_element(dst->begin(), dst->end())));
    return true;
}

PERCEPTION_REGISTER_ONESHOTTYPEFUSION(CCRFSingleShotTypeFusion);
PERCEPTION_REGISTER_SEQUENCETYPEFUSION(CCRFMultiShotTypeFusion);

}  // namespace lidar
}  // namespace perception
}  // namespace apollo
