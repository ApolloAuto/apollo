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
 * @file UpdaterWithChannelsBase.h
 * @brief UpdaterWithChannelsBase class
 * @date 2023/07/13
 */
#pragma once
#include <string>
#include <vector>

#include "modules/dreamview_plus/backend/updater/updater_base.h"

namespace apollo {

namespace dreamview {

/**
 * @class UpdaterWithChannelsBase
 * @brief Base Class for data updater with multiply channels.
 * The updater is responsible for maintaining and updating a data type used by
 * the dreamview module, such as simulation_world, pointcloud, camera,
 * pointcloud etc. Each type of data is a data stream that supports front-end
 * and back-end communication of dreamview.
 */
class UpdaterWithChannelsBase : public UpdaterBase {
 public:
  /**
   * @brief GetChannelMsg
   */
  virtual void GetChannelMsg(std::vector<std::string> *channels) = 0;
  void GetChannelMsgWithFilter(std::vector<std::string> *channels, const std::string& filter_message_type, const std::string& filter_channel);

  std::vector<std::string> channels_;
};

}  // namespace dreamview
}  // namespace apollo
