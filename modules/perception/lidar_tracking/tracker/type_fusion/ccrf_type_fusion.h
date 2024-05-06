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

#pragma once

#include <functional>
#include <map>
#include <memory>
#include <string>
#include <vector>

#include "modules/common/util/eigen_defs.h"
#include "modules/perception/lidar_tracking/tracker/common/mlf_track_data.h"
#include "modules/perception/lidar_tracking/tracker/common/tracked_object.h"
#include "modules/perception/lidar_tracking/tracker/type_fusion/type_fusion_interface.h"
#include "modules/perception/lidar_tracking/tracker/type_fusion/util.h"

namespace apollo {
namespace perception {
namespace lidar {

class CCRFSingleShotTypeFusion : public BaseSingleShotTypeFusion {
 public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    bool Init(const TypeFilterInitOption& option) override;

    bool TypeFusion(const TypeFilterOption& option,
        TrackedObjectPtr new_object) override;

    std::string Name() const override {
        return "CCRFSingleShotTypeFusion";
    }

    bool FuseOneShotTypeProbs(const TrackedObjectConstPtr& new_object,
        Vectord* log_prob);

 protected:
    apollo::common::EigenMap<std::string, Matrixd> smooth_matrices_;
    Matrixd confidence_smooth_matrix_;
    bool debug_log_ = false;
};

class CCRFMultiShotTypeFusion : public BaseMultiShotTypeFusion {
 public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    bool Init(const TypeFilterInitOption& option) override;

    bool TypeFusion(
            const TypeFilterOption& option,
            const std::vector<TrackedObjectConstPtr>& tracked_objects,
            TrackedObjectPtr new_object) override;

    std::string Name() const override {
        return "CCRFMultiShotTypeFusion";
    }

 protected:
    // The fusion problem is modeled
    //     as inferring the discrete state in a chain CRFs.
    // Note, log(P({X}|O)) =
    //     sigma_i{E_unary(X_i,O)} + sigma_ij{E_pairwise(X_i,X_j)} - logZ;
    // E_unary(X_i,O) = sigma{logP(classifier)},
    // E_pairwise(X_i,X_j) = logTransition(X_i,X_j)
    // a) Maximize the sequence probability P(X_t|{X}^opt,O)
    //     based on optimal state inference.
    // b) (did not use) Maximize the marginal probability P(X_t|O)
    //     with history state integrated.

    // window version of Chain-CRFs inference
    bool FuseWithConditionalProbabilityInference(
            const std::vector<TrackedObjectConstPtr>& tracked_objects,
            TrackedObjectPtr new_object);
    // util
    bool RecoverFromLogProbability(Vectord* prob, std::vector<float>* dst,
        perception::base::ObjectType* type);

 protected:
    CCRFSingleShotTypeFusion one_shot_fuser_;
    // Note all in the log space
    Matrixd transition_matrix_;

    // data member for window inference version
    apollo::common::EigenVector<Vectord> fused_oneshot_probs_;
    apollo::common::EigenVector<Vectord> fused_sequence_probs_;
    apollo::common::EigenVector<Vectori> state_back_trace_;

 protected:
    double s_alpha_ = 1.8;
    bool debug_log_ = false;
};

}  // namespace lidar
}  // namespace perception
}  // namespace apollo
