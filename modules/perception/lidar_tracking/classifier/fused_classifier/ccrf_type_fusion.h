/******************************************************************************
 * Copyright 2023 The Apollo Authors. All Rights Reserved.
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

#include "modules/perception/lidar_tracking/classifier/fused_classifier/proto/ccrf_type_fusion_config.pb.h"

#include "modules/common/util/eigen_defs.h"
#include "modules/perception/lidar_tracking/classifier/fused_classifier/type_fusion_interface.h"
#include "modules/perception/lidar_tracking/classifier/fused_classifier/util.h"

namespace apollo {
namespace perception {
namespace lidar {

class CCRFOneShotTypeFusion : public BaseOneShotTypeFusion {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  /**
   * @brief INit type fusion
   *
   * @param option
   * @return true
   * @return false
   */
  bool Init(const TypeFusionInitOption& option) override;
  /**
   * @brief Type fusion of objects
   *
   * @param option
   * @param object
   * @return true
   * @return false
   */
  bool TypeFusion(const TypeFusionOption& option,
                  std::shared_ptr<perception::base::Object> object) override;
  /**
   * @brief Get class name
   *
   * @return std::string
   */
  std::string Name() const override { return "CCRFOneShotTypeFusion"; }
  /**
   * @brief Get object type probabilities
   *
   * @param object update type probs of the object
   * @param log_prob
   * @return true
   * @return false
   */
  bool FuseOneShotTypeProbs(
      const std::shared_ptr<perception::base::Object>& object,
      Vectord* log_prob);

 protected:
  apollo::common::EigenMap<std::string, Matrixd> smooth_matrices_;
  Matrixd confidence_smooth_matrix_;
};

class CCRFSequenceTypeFusion : public BaseSequenceTypeFusion {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  /**
   * @brief Init type fusion
   *
   * @param option
   * @return true
   * @return false
   */
  bool Init(const TypeFusionInitOption& option) override;
  /**
   * @brief Type fusion of tracked object
   *
   * @param option
   * @param tracked_objects update the type probs of the tracked object
   * @return true
   * @return false
   */
  bool TypeFusion(const TypeFusionOption& option,
                  TrackedObjects* tracked_objects) override;
  /**
   * @brief Get class name
   *
   * @return std::string
   */
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
  apollo::common::EigenVector<Vectord> fused_oneshot_probs_;
  apollo::common::EigenVector<Vectord> fused_sequence_probs_;
  apollo::common::EigenVector<Vectori> state_back_trace_;

 protected:
  double s_alpha_ = 1.8;
};

}  // namespace lidar
}  // namespace perception
}  // namespace apollo
