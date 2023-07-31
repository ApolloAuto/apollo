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
#include <memory>
#include <string>

#include "cyber/class_loader/class_loader_manager.h"
#include "cyber/common/file.h"
#include "cyber/common/log.h"
#include "cyber/common/macros.h"
#include "cyber/plugin_manager/plugin_description.h"

namespace apollo {
namespace cyber {
namespace plugin_manager {

class PluginManager {
 public:
  ~PluginManager();

  /**
   * @brief parse plugin description file and load the library
   * TODO(liangjinping): parse description to struct
   *
   * @param file_path the path of plugin description file
   * @return process result, true for success
   */
  bool ProcessPluginDescriptionFile(const std::string& file_path,
                                    std::string* library_path);

  /**
   * @brief load plugin clases from file
   *
   * @param pluin_description_file_path file path
   * @return result of loadding plugin, true for success
   */
  bool LoadPlugin(const std::string& plugin_description_file_path);

  /**
   * @brief get singleton instance of PluginManager
   *
   * @return instance pointer
   */
  static PluginManager* Instance();

  /**
   * @brief create plugin instance of derived class based on `Base`
   *
   * @param derived_class class name of the derived class
   * @return instance pointer
   */
  template <typename Base>
  std::shared_ptr<Base> CreateInstance(const std::string& derived_class);

  /**
   * @brief find plugin index file and load plugins
   *
   * @param plugin_index_path plugin index file directory
   * @return result of loadding plugins, true for success
   */
  bool FindPluginIndexAndLoad(const std::string& plugin_index_path);

  /**
   * @brief load plugins from installed path
   *
   * @return result of loadding plugins, true for success
   */
  bool LoadInstalledPlugins();

  /**
   * @bried get plugin description file location that class belongs to
   * @param class_name derived class name
   * @return location of plugin description file
   */
  template <typename Base>
  std::string GetPluginClassHomePath(const std::string& class_name);

 private:
  apollo::cyber::class_loader::ClassLoaderManager class_loader_manager_;
  std::map<std::string, std::shared_ptr<PluginDescription>>
      plugin_description_map_;

  static PluginManager* instance_;
};

template <typename Base>
std::shared_ptr<Base> PluginManager::CreateInstance(
    const std::string& derived_class) {
  AINFO << "creating plugin instance of " << derived_class;
  return class_loader_manager_.CreateClassObj<Base>(derived_class);
}

template <typename Base>
std::string PluginManager::GetPluginClassHomePath(
    const std::string& class_name) {
  std::string library_path =
      class_loader_manager_.GetClassValidLibrary<Base>(class_name);
  if (library_path == "") {
    AWARN << "plugin of class " << class_name << " not found";
    return ".";
  }
  for (auto it = plugin_description_map_.begin();
       it != plugin_description_map_.end(); ++it) {
    if (it->second->library_path_ == library_path) {
      return apollo::cyber::common::GetDirName(it->second->description_path_);
    }
  }
  // not found
  return ".";
}

#define CYBER_PLUGIN_MANAGER_REGISTER_PLUGIN(name, base) \
  CLASS_LOADER_REGISTER_CLASS(name, base)

}  // namespace plugin_manager
}  // namespace cyber
}  // namespace apollo
