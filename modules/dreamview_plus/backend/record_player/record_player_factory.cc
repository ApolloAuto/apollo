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
#include "modules/dreamview_plus/backend/record_player/record_player_factory.h"

namespace apollo {
namespace dreamview {
using RLock = boost::shared_lock<boost::shared_mutex>;
using WLock = boost::unique_lock<boost::shared_mutex>;

RecordPlayerFactory::RecordPlayerFactory() {
  // unified init cyber for all player related node.
  apollo::cyber::Init("record_player_factory");
  current_record_ = "";
}

void RecordPlayerFactory::Reset() {
  for (auto iter = s_record_player_map_.begin();
       iter != s_record_player_map_.end();) {
    iter->second = nullptr;
    s_record_player_map_.erase(iter++);
  }
  current_record_ = "";
}

RecordPlayerFactory::~RecordPlayerFactory() { Reset(); }

void RecordPlayerFactory::UnregisterRecordPlayer(
    const std::string& record_name) {
  auto iter = s_record_player_map_.find(record_name);
  if (iter == s_record_player_map_.end()) {
    AERROR << "Failed to get " << record_name << " related player pointer.";
    return;
  }
  iter->second = nullptr;
  s_record_player_map_.erase(record_name);
  {
    WLock wlock(mutex_);
    auto vec_iter =
        std::find(loaded_record_.begin(), loaded_record_.end(), record_name);
    if (vec_iter != loaded_record_.end()) {
      loaded_record_.erase(vec_iter);
    }
  }
  return;
}

bool RecordPlayerFactory::RegisterRecordPlayer(
    const std::string& record_name, const std::string& record_file_path) {
  apollo::cyber::record::PlayParam play_param;
  play_param.is_play_all_channels = true;
  // use play_param struct default value
  // loop playback is not allowed
  play_param.is_loop_playback = false;
  std::vector<std::string> opt_black_channels;
  // When using DV play recorder, need to block some real-time channels
  // to avoid conflicts with real-time DV information.
  opt_black_channels.push_back("/apollo/hmi/status");
  opt_black_channels.push_back("/apollo/monitor/system_status");
  opt_black_channels.push_back("/apollo/cyber/record_info");
  play_param.black_channels.insert(opt_black_channels.begin(),
                                 opt_black_channels.end());
  // play_param.play_rate = 1.0f;
  // play_param.begin_time_ns = 0;
  // play_param.end_time_ns = std::numeric_limits<uint64_t>::max();
  // play_param.start_time_s = 0;
  // play_param.delay_time_s = 0;
  // preload time and delay time is no used when nohup player process
  play_param.preload_time_s = 1;
  play_param.record_id = record_name;
  play_param.files_to_play.insert(record_file_path);
  const std::string node_name = "record_player_factory_" + record_name;
  s_record_player_map_[record_name] = std::unique_ptr<Player>(
      new Player(play_param, apollo::cyber::CreateNode(node_name), true));
  s_record_player_map_[record_name]->Init();
  {
    WLock wlock(mutex_);
    loaded_record_.push_back(record_name);
  }
  return true;
}

void RecordPlayerFactory::GetAllRecords(std::vector<std::string>* records) {
  for (auto iter = s_record_player_map_.begin();
       iter != s_record_player_map_.end(); iter++) {
    records->push_back(iter->first);
  }
}
void RecordPlayerFactory::SetCurrentRecord(const std::string& record_name) {
  current_record_ = record_name;
}

std::string RecordPlayerFactory::GetCurrentRecord() { return current_record_; }

Player* RecordPlayerFactory::GetRecordPlayer(const std::string& record_name) {
  auto iter = s_record_player_map_.find(record_name);
  if (iter == s_record_player_map_.end()) {
    AERROR << "Failed to get " << record_name << " related player pointer.";
    return nullptr;
  }
  return iter->second.get();
}

int RecordPlayerFactory::GetLoadedRecordNum() {
  {
    RLock rlock(mutex_);
    return loaded_record_.size();
  }
}

bool RecordPlayerFactory::EnableContinuePreload() {
  return GetLoadedRecordNum() < PRELOAD_RECORD_NUM;
}

bool RecordPlayerFactory::EnableContinueLoad() {
  return GetLoadedRecordNum() < LOADED_RECORD_NUM;
}

void RecordPlayerFactory::IncreaseRecordPriority(
    const std::string& record_name) {
  WLock wlock(mutex_);
  auto iter =
      std::find(loaded_record_.begin(), loaded_record_.end(), record_name);
  if (iter == loaded_record_.end()) {
    AERROR << "Current record is not loaded,invalid operation!";
    return;
  }
  loaded_record_.erase(iter);
  loaded_record_.push_back(record_name);
  return;
}

bool RecordPlayerFactory::RemoveLRURecord(std::string* remove_record_id) {
  if (EnableContinueLoad()) {
    AERROR << "No need to remove record.";
    return false;
  }
  {
    RLock rlock(mutex_);
    auto iter = loaded_record_.begin();
    *remove_record_id = *loaded_record_.begin();
    while (iter != loaded_record_.end()) {
      // Can not remove selected record
      if (*iter != current_record_) {
        *remove_record_id = *iter;
        break;
      }
      iter++;
    }
  }
  UnregisterRecordPlayer(*remove_record_id);
  return true;
}

}  // namespace dreamview
}  // namespace apollo
