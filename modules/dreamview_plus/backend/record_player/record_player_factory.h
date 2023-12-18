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
#pragma once

#include <memory>
#include <string>
#include <unordered_map>
#include <vector>
#include <boost/thread/locks.hpp>
#include <boost/thread/shared_mutex.hpp>

#include "cyber/common/macros.h"
#include "cyber/cyber.h"
#include "cyber/init.h"
#include "cyber/tools/cyber_recorder/player/player.h"

namespace apollo {
namespace dreamview {

using apollo::cyber::record::Player;

class RecordPlayerFactory {
 public:
  ~RecordPlayerFactory();
  Player* GetRecordPlayer(const std::string& record_name);
  bool RegisterRecordPlayer(const std::string& record_namem,
                            const std::string& record_file_path);
  void UnregisterRecordPlayer(const std::string& record_name);
  void Reset();
  void GetAllRecords(std::vector<std::string>* records);
  void SetCurrentRecord(const std::string& record_name);
  void IncreaseRecordPriority(const std::string& record_name);
  bool RemoveLRURecord(std::string* remove_record_id);
  std::string GetCurrentRecord();
  int GetLoadedRecordNum();
  bool EnableContinuePreload();
  bool EnableContinueLoad();

 private:
  std::unordered_map<std::string, std::unique_ptr<Player>> s_record_player_map_;
  std::string current_record_;
  std::vector<std::string> loaded_record_;
  mutable boost::shared_mutex mutex_;
  static const int32_t PRELOAD_RECORD_NUM = 3;
  static const int32_t LOADED_RECORD_NUM = 15;

  DECLARE_SINGLETON(RecordPlayerFactory);
};

}  // namespace dreamview
}  // namespace apollo
