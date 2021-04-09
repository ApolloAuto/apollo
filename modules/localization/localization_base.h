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

/**
 * @file localization_base.h
 * @brief The class of LocalizationBase
 */

#ifndef MODULES_LOCALIZATION_LOCALIZATION_BASE_H_
#define MODULES_LOCALIZATION_LOCALIZATION_BASE_H_

#include <memory>

#include "tf2_ros/transform_broadcaster.h"

#include "modules/localization/proto/localization.pb.h"

#include "modules/common/status/status.h"

/**
 * @namespace apollo::localization
 * @brief apollo::localization
 */
namespace apollo {
namespace localization {

/**
 * @class LocalizationBase
 *
 * @brief base class for Localization factory
 */
class LocalizationBase {
 public:
  virtual ~LocalizationBase() = default;

  /**
   * @brief module start function
   * @return start status
   */
  virtual apollo::common::Status Start() = 0;

  /**
   * @brief module stop function
   * @return stop status
   */
  virtual apollo::common::Status Stop() = 0;

  void PublishPoseBroadcastTF(const LocalizationEstimate &localization);

 protected:
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf2_broadcaster_;
};

}  // namespace localization
}  // namespace apollo

#endif  // MODULES_LOCALIZATION_LOCALIZATION_BASE_H_
