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

#pragma once

#include <set>
#include <string>

#include "cyber/common/macros.h"

namespace apollo {
namespace data {

/**
 * @class ChannelPool
 * @brief Provides helper functions to offer different channels
 */
class ChannelPool {
 public:
  // Getters
  const std::set<std::string>& GetSmallChannels() { return small_channels_; }
  const std::set<std::string>& GetLargeChannels() { return large_channels_; }
  const std::set<std::string>& GetAllChannels() { return all_channels_; }

 private:
  std::set<std::string> small_channels_;
  std::set<std::string> large_channels_;
  std::set<std::string> all_channels_;

  DECLARE_SINGLETON(ChannelPool)
};

}  // namespace data
}  // namespace apollo
