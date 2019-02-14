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
#pragma once
#include <map>
#include <memory>
#include <string>
#include <vector>

#include "modules/perception/lidar/lib/classifier/fused_classifier/type_fusion_interface.h"
#include "modules/perception/lidar/lib/classifier/fused_classifier/util.h"

namespace apollo {
namespace perception {
namespace lidar {

class CCRFOneShotTypeFusion : public BaseOneShotTypeFusion {
 public:
  bool Init(const TypeFusionInitOption& option) override;
  bool TypeFusion(const TypeFusionOption& option,
                  std::shared_ptr<perception::base::Object> object) override;
  std::string Name() const override { return "CCRFOneShotTypeFusion"; }
  bool FuseOneShotTypeProbs(
      const std::shared_ptr<perception::base::Object>& object,
      Vectord* log_prob);

 protected:
  std::map<std::string, Matrixd> smooth_matrices_;
  Matrixd confidence_smooth_matrix_;
};

class CCRFSequenceTypeFusion : public BaseSequenceTypeFusion {
 public:
  bool Init(const TypeFusionInitOption& option) override;
  bool TypeFusion(const TypeFusionOption& option,
                  TrackedObjects* tracked_objects) override;
  std::string Name() const override { return "CCRFSequenceTypeFusion"; }

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
  bool FuseWithConditionalProbabilityInference(TrackedObjects* tracked_objects);
  // util
  bool RecoverFromLogProbability(Vectord* prob, std::vector<float>* dst,
                                 perception::base::ObjectType* type);

 protected:
  CCRFOneShotTypeFusion one_shot_fuser_;
  // Note all in the log space
  Matrixd transition_matrix_;

  // data member for window inference version
  std::vector<Vectord> fused_oneshot_probs_;
  std::vector<Vectord> fused_sequence_probs_;
  std::vector<Vectori> state_back_trace_;

 protected:
  double s_alpha_ = 1.8;
};

}  // namespace lidar
}  // namespace perception
}  // namespace apollo
