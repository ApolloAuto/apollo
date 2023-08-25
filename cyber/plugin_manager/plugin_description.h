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
#include <map>
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
  std::string actual_description_path_;
  std::string library_path_;
  std::string actual_library_path_;

  std::map<std::string, std::string> class_name_base_class_name_map_;

  PluginDescription();
  explicit PluginDescription(const std::string& name);
  PluginDescription(const std::string& name,
                    const std::string& description_index_path,
                    const std::string& description_path,
                    const std::string& actual_description_path,
                    const std::string& library_path,
                    const std::string& actual_library_path);

  /**
   * @brief parse plugin description metadata from plugin index file
   *
   * @param file_path plugin index file path
   * @return true if parse success
   */
  bool ParseFromIndexFile(const std::string& file_path);

  /**
   * @brief parse plugin description metadata from plugin description file
   *
   * @param file_path plugin description file path
   * @return true if parse success
   */
  bool ParseFromDescriptionFile(const std::string& file_path);
};

}  // namespace plugin_manager
}  // namespace cyber
}  // namespace apollo
