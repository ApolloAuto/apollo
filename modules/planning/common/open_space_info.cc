/******************************************************************************
 * Copyright 2019 The Apollo Authors. All Rights Reserved.
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

/**
 * @file
 **/

#include "modules/planning/common/open_space_info.h"

namespace apollo {
namespace planning {
void CopyTrajectory(
    const apollo::planning::DiscretizedTrajectory trajectory_src,
    apollo::common::Trajectory* trajectory_tgt_ptr) {
  size_t horizon = trajectory_src.NumOfPoints();
  for (size_t i = 0; i < horizon; ++i) {
    auto* added_pt = trajectory_tgt_ptr->add_trajectory_point();
    auto* added_path_pt = added_pt->mutable_path_point();
    auto picked_pt = trajectory_src.TrajectoryPointAt(i).path_point();
    added_path_pt->set_x(picked_pt.x());
    added_path_pt->set_y(picked_pt.y());
    added_path_pt->set_theta(picked_pt.theta());
  }
}

// record more trajectory information to info debug
void OpenSpaceInfo::RecordDebug(apollo::planning_internal::Debug* ptr_debug) {
  // 1, merge debug info into ptr_debug
  ptr_debug->MergeFrom(debug_instance_);

  // 2, record partitioned trajectories into debug_ptr
  auto* ptr_partitioned_trajectories = ptr_debug->mutable_planning_data()
                                           ->mutable_open_space()
                                           ->mutable_partitioned_trajectories();

  for (auto& iter : paritioned_trajectories_) {
    auto picked_trajectory = iter.first;
    auto* ptr_added_trajectory = ptr_partitioned_trajectories->add_trajectory();
    CopyTrajectory(picked_trajectory, ptr_added_trajectory);
  }

  // 3, record chosed partitioned into debug_ptr
  auto* ptr_chosen_trajectory = ptr_debug->mutable_planning_data()
                                    ->mutable_open_space()
                                    ->mutable_chosen_trajectory()
                                    ->add_trajectory();
  auto chosen_trajectory = chosen_paritioned_trajectory_.first;
  CopyTrajectory(chosen_trajectory, ptr_chosen_trajectory);

  // 4, record if the trajectory is fallback trajecotry
  ptr_debug->mutable_planning_data()
      ->mutable_open_space()
      ->set_is_fallback_trajectory(fallback_flag_);
}

}  // namespace planning
}  // namespace apollo
