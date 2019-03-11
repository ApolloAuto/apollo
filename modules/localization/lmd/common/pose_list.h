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

/**
 * @file pose_list.h
 * @brief The class of PoseList.
 */

#ifndef MODULES_LOCALIZATION_LMD_COMMON_POSE_LIST_H_
#define MODULES_LOCALIZATION_LMD_COMMON_POSE_LIST_H_

#include "modules/localization/proto/localization.pb.h"

#include "modules/localization/lmd/common/tm_list.h"

/**
 * @namespace apollo::localization
 * @brief apollo::localization
 */
namespace apollo {
namespace localization {

/**
 * @class PoseList
 *
 * @brief  A time marked list of poses.
 */
class PoseList : public TimeMarkedList<Pose> {
 public:
  explicit PoseList(double memory_cycle_sec = 0.0)
      : TimeMarkedList<Pose>(memory_cycle_sec) {}

  /**
   * @brief  Get a pose by timestamp, may need interpolation.
   * @param timestamp_sec The required timestamp.
   * @param pose The output.
   * @return True if found.
   */
  bool FindMatchingPose(double timestamp_sec, Pose *pose) const;

  /**
   * @brief  Get a nearest pose by timestamp, may need interpolation.
   * @param timestamp_sec The required timestamp.
   * @param pose The output.
   * @return True if found.
   */
  bool FindNearestPose(double timestamp_sec, Pose *pose) const;

  /**
   * @brief  Get a pose by interpolating two poses.
   * @param timestamp_sec1 The timestamp of pose 1.
   * @param pose1 The pose 1.
   * @param timestamp_sec2 The timestamp of pose 2.
   * @param pose2 The pose 2.
   * @param timestamp_sec The required timestamp, must >=timestamp_sec1 and
   * <=timestamp_sec2.
   * @param pose The output.
   */
  static void InterpolatePose(double timestamp_sec1, const Pose &pose1,
                              double timestamp_sec2, const Pose &pose2,
                              double timestamp_sec, Pose *pose);
};

}  // namespace localization
}  // namespace apollo

#endif  // MODULES_LOCALIZATION_LMD_COMMON_POSE_LIST_H_
