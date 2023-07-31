/******************************************************************************
 * Copyright 2023 The Apollo Authors. All Rights Reserved.
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
#include <string>

namespace apollo {
namespace cyber {
namespace plugin_manager {

// plugin description struct
class PluginDescription {
 public:
  std::string name_;
  std::string description_index_path_;
  std::string description_path_;
  std::string library_path_;
  PluginDescription(const std::string& name,
                    const std::string& description_index_path,
                    const std::string& description_path,
                    const std::string& library_path)
      : name_(name),
        description_index_path_(description_index_path),
        description_path_(description_path),
        library_path_(library_path) {}
};

}  // namespace plugin_manager
}  // namespace cyber
}  // namespace apollo
