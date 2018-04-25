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
#ifndef MODULES_PERCEPTION_COMMON_SEQUENCE_TYPE_FUSER_SEQUENCE_TYPE_FUSER_H_
#define MODULES_PERCEPTION_COMMON_SEQUENCE_TYPE_FUSER_SEQUENCE_TYPE_FUSER_H_

#include <map>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include "modules/perception/common/sequence_type_fuser/base_type_fuser.h"
#include "modules/perception/common/sequence_type_fuser/fuser_util.h"
#include "modules/perception/common/sequence_type_fuser/object_sequence.h"

namespace apollo {
namespace perception {

class SequenceTypeFuser : public BaseTypeFuser {
 public:
  /**
   * @brief Constructor
   */
  SequenceTypeFuser() {}

  /**
   * @ brief Destructor
   */
  virtual ~SequenceTypeFuser() {}

  /**
   * @brief Initialize configuration
   * @return True if initialize successfully, false otherwise
   */
  bool Init() override;

  /**
   * @brief Fuse type over the sequence for each object
   * @param options Some algorithm options declared in BaseTypeFuser
   * @param objects The objects with initial object type
   * @return True if fuse type successfully, false otherwise
   */
  bool FuseType(const TypeFuserOptions& options,
                std::vector<std::shared_ptr<Object>>* objects) override;

  /**
   * @brief Get module name
   * @return Name of module
   */
  std::string name() const override { return "SequenceTypeFuser"; }

 protected:
  /**
   * @brief Fuse type over object sequence by a linear-chain CRF.
   * The fusion problem is modeled as inferring the discrete state
   * in a chain CRF. Note, log(P({X}|O)) = sigma_i{E_unary(X_i,O)} +
   * sigma_ij{E_pairwise(X_i,X_j)} - logZ,
   * E_unary(X_i,O) = sigma{logP(classifier)},
   * E_pairwise(X_i,X_j) = log{Transition(X_i,X_j)}.
   * Maximize the sequence probability P(X_t|{X}^opt,O) based on the
   * Viterbi algorithm.
   * @param tracked_objects The tracked objects as a sequence
   * @return True if fuse successfully, false otherwise
   */
  bool FuseWithCCRF(
      std::map<int64_t, std::shared_ptr<Object>>* tracked_objects);

  /**
   * @brief Rectify the initial object type based on smooth matrices
   * @param object The object with initial type probabilities
   * @param log_prob The output rectified type probabilities
   * @return True if rectify successfully, false otherwise
   */
  bool RectifyObjectType(const std::shared_ptr<Object>& object,
                         Vectord* log_prob);

  /**
   * @brief Recover type probabilities and object type from the input
   * log probabilities
   * @param prob The input type probabilities in the log space
   * @param dst The output normalized type probabilities
   * @param type The output object type with the max probability
   * @return True if recover successfully, false otherwise
   */
  bool RecoverFromLogProb(Vectord* prob, std::vector<float>* dst,
                          ObjectType* type);

 protected:
  ObjectSequence sequence_;

  double temporal_window_;

  Matrixd transition_matrix_;
  Matrixd confidence_smooth_matrix_;
  std::unordered_map<std::string, Matrixd> smooth_matrices_;

  // Note all probabilities are in the log space
  std::vector<Vectord> fused_oneshot_probs_;
  std::vector<Vectord> fused_sequence_probs_;
  std::vector<Vectori> state_back_trace_;

  static constexpr double s_alpha_ = 1.8;

 private:
  DISALLOW_COPY_AND_ASSIGN(SequenceTypeFuser);
};

REGISTER_TYPEFUSER(SequenceTypeFuser);

}  // namespace perception
}  // namespace apollo

#endif  // MODULES_PERCEPTION_COMMON_SEQUENCE_TYPE_FUSER_SEQUENCE_TYPE_FUSER_H_
