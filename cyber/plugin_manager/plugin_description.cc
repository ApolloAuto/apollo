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
#include "cyber/plugin_manager/plugin_description.h"

#include <map>
#include <memory>
#include <regex>
#include <string>
#include <utility>

#include <tinyxml2.h>

#include "cyber/common/environment.h"
#include "cyber/common/file.h"
#include "cyber/common/log.h"

namespace apollo {
namespace cyber {
namespace plugin_manager {

PluginDescription::PluginDescription() {}

PluginDescription::PluginDescription(const std::string& name) : name_(name) {}

PluginDescription::PluginDescription(const std::string& name,
                                     const std::string& description_index_path,
                                     const std::string& description_path,
                                     const std::string& actual_description_path,
                                     const std::string& library_path,
                                     const std::string& actual_library_path)
    : name_(name),
      description_index_path_(description_index_path),
      description_path_(description_path),
      actual_description_path_(actual_description_path),
      library_path_(library_path),
      actual_library_path_(actual_library_path) {}

bool PluginDescription::ParseFromIndexFile(const std::string& file_path) {
  this->description_index_path_ = file_path;
  this->name_ = apollo::cyber::common::GetFileName(file_path);

  if (!apollo::cyber::common::GetContent(file_path, &this->description_path_)) {
    AWARN << "plugin index[" << file_path << "] name[" << this->name_
          << "] invalid, read index file failed";
    return false;
  }
  return ParseFromDescriptionFile(this->description_path_);
}

bool PluginDescription::ParseFromDescriptionFile(const std::string& file_path) {
  if (this->description_path_.empty()) {
    this->description_path_ = file_path;
  }

  if (!apollo::cyber::common::GetFilePathWithEnv(
          this->description_path_, "APOLLO_PLUGIN_DESCRIPTION_PATH",
          &this->actual_description_path_)) {
    AWARN << "plugin index[" << file_path << "] name[" << this->name_
          << "] invalid, description[" << this->description_path_
          << "] file not found";
    return false;
  }

  tinyxml2::XMLDocument doc;
  if (doc.LoadFile(this->actual_description_path_.c_str()) !=
      tinyxml2::XML_SUCCESS) {
    AWARN << "plugin description[" << file_path << "] name[" << this->name_
          << "] invalid, parse description file failed";
    return false;
  }
  const tinyxml2::XMLElement* root = doc.RootElement();
  this->library_path_ = root->Attribute("path");

  std::string plugin_name =
      std::regex_replace(this->library_path_, std::regex("/"), "__");
  if (this->name_.empty()) {
    this->name_ = plugin_name;
  }

  // process class name and base class name from description file, this will be
  // used to build index fo lazy load
  for (const tinyxml2::XMLElement* class_element =
           root->FirstChildElement("class");
       class_element != nullptr;
       class_element = class_element->NextSiblingElement("class")) {
    std::string class_name = class_element->Attribute("type");
    std::string base_class_name = class_element->Attribute("base_class");

    if (this->class_name_base_class_name_map_.find(class_name) !=
        this->class_name_base_class_name_map_.end()) {
      AWARN << "plugin description[" << file_path << "] name[" << this->name_
            << "] invalid, class name[" << class_name << "] duplicated";
      continue;
    }

    if (class_name.empty() || base_class_name.empty()) {
      AWARN << "plugin description[" << file_path << "] name[" << this->name_
            << "] invalid, class name[" << class_name << "] base class name["
            << base_class_name << "] invalid";
      continue;
    }
    this->class_name_base_class_name_map_[class_name] = base_class_name;
  }

  if (!apollo::cyber::common::GetFilePathWithEnv(this->library_path_,
                                                 "APOLLO_PLUGIN_LIB_PATH",
                                                 &this->actual_library_path_)) {
    AWARN << "plugin description[" << file_path << "] name[" << this->name_
          << "] invalid, library[" << this->library_path_ << "] file not found";
    return false;
  }

  return true;
}

}  // namespace plugin_manager
}  // namespace cyber
}  // namespace apollo
