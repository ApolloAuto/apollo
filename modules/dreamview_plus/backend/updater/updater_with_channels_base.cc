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

#include "modules/dreamview_plus/backend/updater/updater_with_channels_base.h"

#include <set>

#include "cyber/cyber.h"
#include "modules/dreamview_plus/backend/record_player/record_player_factory.h"

namespace apollo {

namespace dreamview {
UpdaterWithChannelsBase::UpdaterWithChannelsBase(
    const std::vector<std::string> &filter_message_types,
    const std::vector<std::string> &filter_channels)
    : filter_message_types_(filter_message_types),
      filter_channels_(filter_channels) {}

bool UpdaterWithChannelsBase::IsChannelInUpdater(
    const std::string &message_type, const std::string &channel_name) {
  if (filter_message_types_.size() == 0 && filter_channels_.size() == 0) {
    return true;
  }
  for (size_t i = 0; i < filter_message_types_.size(); ++i) {
    if (message_type.rfind(filter_message_types_[i]) != std::string::npos &&
        channel_name.rfind(filter_channels_[i]) != std::string::npos) {
      return true;
    }
  }
  return false;
}

void UpdaterWithChannelsBase::GetChannelMsgWithFilter(
    std::vector<std::string> *channels) {
  auto channelManager =
      apollo::cyber::service_discovery::TopologyManager::Instance()
          ->channel_manager();
  std::vector<apollo::cyber::proto::RoleAttributes> role_attr_vec;
  channelManager->GetWriters(&role_attr_vec);
  std::vector<std::string> records;
  std::vector<std::string> other_record_node_name;
  auto *record_player_factory = RecordPlayerFactory::Instance();
  record_player_factory->GetAllRecords(&records);
  const std::string current_record = record_player_factory->GetCurrentRecord();
  for (auto iter = records.begin(); iter != records.end(); iter++) {
    if (current_record.empty() || *iter != current_record) {
      std::string other_node_name = "record_player_factory_" + *iter;
      other_record_node_name.push_back(other_node_name);
    }
  }
  std::vector<std::string> tmp_channels;
  for (auto &role_attr : role_attr_vec) {
    std::string messageType;
    std::string node_name;
    messageType = role_attr.message_type();
    node_name = role_attr.node_name();
    if (IsChannelInUpdater(messageType, role_attr.channel_name())) {
      if (current_record.empty() ||
          std::find(other_record_node_name.begin(),
                    other_record_node_name.end(),
                    node_name) == other_record_node_name.end()) {
        tmp_channels.push_back(role_attr.channel_name());
      }
    }
  }
  std::set<std::string> s(tmp_channels.begin(), tmp_channels.end());
  channels->assign(tmp_channels.begin(), tmp_channels.end());
  channels_.clear();
  channels_.assign(s.begin(), s.end());
}

}  // namespace dreamview
}  // namespace apollo
