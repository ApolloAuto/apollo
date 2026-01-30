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
void UpdaterWithChannelsBase::GetChannelMsgWithFilter(
    std::vector<std::string> *channels, const std::string &filter_message_type,
    const std::string &filter_channel, bool reverse_filter_channel) {
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
    int index = 0;
    if (!filter_message_type.empty()) {
      index = messageType.rfind(filter_message_type);
    }
    int index_channel = 0;
    bool select_current_channel = filter_channel.empty();
    if (!filter_channel.empty()) {
      index_channel = role_attr.channel_name().rfind(filter_channel);
      select_current_channel = reverse_filter_channel ? (index_channel == -1)
                                                      : (index_channel != -1);
    }
    if (index != -1 && select_current_channel) {
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
