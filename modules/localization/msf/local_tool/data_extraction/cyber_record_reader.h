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

#include <functional>
#include <string>
#include <unordered_map>
#include <vector>

namespace apollo {
namespace localization {
namespace msf {

/**
 * @class CyberRecordReader
 * @brief Read messages from cyber record.
 */
class CyberRecordReader {
 public:
  CyberRecordReader();
  ~CyberRecordReader();

  void Subscribe(const std::string& topic,
                 const std::function<void(const std::string&)> call_back);
  void Read(const std::string& file_name);

 private:
  std::vector<std::string> topics_;
  std::unordered_map<std::string, std::function<void(const std::string&)>>
      call_back_map_;
};

}  // namespace msf
}  // namespace localization
}  // namespace apollo
