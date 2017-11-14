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
#ifndef MODULES_PERCEPTION_OBSTACLE_LIDAR_SEQUENCE_TYPE_FUSER_H_
#define MODULES_PERCEPTION_OBSTACLE_LIDAR_SEQUENCE_TYPE_FUSER_H_

#include "modules/perception/obstacle/lidar/interface/base_type_fuser.h"
#include "modules/perception/obstacle/common/object_sequence.h"
#include "modules/perception/obstacle/lidar/type_fuser/sequence_type_fuser/util.h"

namespace apollo {
namespace perception {

class SequenceTypeFuser : public BaseTypeFuser {
 public:
  typedef ObjectSequence::TrackedObjects TrackedObjects;

  SequenceTypeFuser() {}

  virtual ~SequenceTypeFuser() {}

  // @brief initialize fuser's configs
  // @return true if initialize successfully, otherwise return false
  bool Init() override;

  bool FuseType(const TypeFuserOptions& options,
                std::vector<ObjectPtr>* objects) override;

  std::string name() const override {
    return "SequenceTypeFuser";
  }

 protected:
  // The fusion problem is modeled as inferring the discrete state in a chain CRFs. 
  // Note, log(P({X}|O)) = sigma_i{E_unary(X_i,O)} + sigma_ij{E_pairwise(X_i,X_j)} - logZ;
  // E_unary(X_i,O) = sigma{logP(classifier)}, E_pairwise(X_i,X_j) = logTransition(X_i,X_j)
  // Maximize the sequence probability P(X_t|{X}^opt,O) based on optimal state inference.
  bool FuseWithCCRF(TrackedObjects* tracked_objects);

  bool RectifyObjectType(const ObjectPtr& object, Vectord* log_prob);

  bool RecoverFromLogProb(Vectord* prob, std::vector<float>* dst, ObjectType* type);

 protected:
  ObjectSequence _sequence;   

  double _temporal_window;

  // Note all probabilities are in the log space
  Matrixd _transition_matrix;
  std::vector<Vectord> _fused_oneshot_probs; 
  std::vector<Vectord> _fused_sequence_probs; 
  std::vector<Vectori> _state_back_trace;

  std::map<std::string, Matrixd> _smooth_matrices;
  Matrixd _confidence_smooth_matrix;

  double _ccrf_debug = true;

  static constexpr double _s_alpha = 1.8;

 private:
  DISALLOW_COPY_AND_ASSIGN(SequenceTypeFuser);
};

REGISTER_TYPEFUSER(SequenceTypeFuser);

}  // namespace perception
}  // namespace apollo

#endif  // MODULES_PERCEPTION_OBSTACLE_LIDAR_SEQUENCE_TYPE_FUSER_H_
