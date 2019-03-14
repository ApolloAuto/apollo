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

#include "modules/prediction/predictor/interaction/interaction_predictor.h"

namespace apollo {
namespace prediction {

void InteractionPredictor::Predict(Obstacle* obstacle) {
  // TODO(kechxu) implement
  /*
  for each lane_sequence {
    Sample trajectories
    for each traj in trajectories {
      assign_cost(traj) {
        * centripetal acc
        * collision with ego vehicle if his right of way is lower
      }
    }

    Select lowest cost best_traj, compute its likelihood probability
    Associate likelihood with model prior probability -> posterior probability
  }
  output trajectories with relatively higher posterior probabilities
    */
}

}  // namespace prediction
}  // namespace apollo
