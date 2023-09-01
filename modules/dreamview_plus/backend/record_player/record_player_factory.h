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
  std::string GetCurrentRecord();

 private:
  std::unordered_map<std::string, std::unique_ptr<Player>> s_record_player_map_;
  std::string current_record_;

  DECLARE_SINGLETON(RecordPlayerFactory);
};

}  // namespace dreamview
}  // namespace apollo
