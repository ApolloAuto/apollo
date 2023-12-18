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
#include <utility>
#include <vector>

#include <cxxabi.h>

#include "cyber/class_loader/class_loader_manager.h"
#include "cyber/common/environment.h"
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

  /**
   * @brief get plugin configuration file location
   * @param class_name derived class name
   * @param conf_name configuration file name
   * @return location of plugin configuration file
   */
  template <typename Base>
  std::string GetPluginConfPath(const std::string& class_name,
                                const std::string& conf_name);

  /**
   * @brief load library of plugin
   * @param library_path library path
   * @return result of loading library, true for success
   */
  bool LoadLibrary(const std::string& library_path);

  /**
   * @brief check if library of plugin is loaded
   * @param class_name derived class name of plugin
   * @return result of checking, true for loaded
   */
  template <typename Base>
  bool IsLibraryLoaded(const std::string& class_name);

  /**
   * @brief check if library of plugin is loaded, and load it if not
   * @param class_name derived class name of plugin
   * @return result of checking, true for loaded
   */
  template <typename Base>
  bool CheckAndLoadPluginLibrary(const std::string& class_name);

  /**
   * @brief Get all derived class name by base class name
   * @return all derived class name of base class name
   */
  template <typename Base>
  std::vector<std::string> GetDerivedClassNameByBaseClass();

 private:
  apollo::cyber::class_loader::ClassLoaderManager class_loader_manager_;
  std::map<std::string, std::shared_ptr<PluginDescription>>
      plugin_description_map_;
  std::map<std::string, bool> plugin_loaded_map_;
  std::map<std::pair<std::string, std::string>, std::string>
      plugin_class_plugin_name_map_;

  static PluginManager* instance_;
};

template <typename Base>
std::shared_ptr<Base> PluginManager::CreateInstance(
    const std::string& derived_class) {
  AINFO << "creating plugin instance of " << derived_class;
  if (!CheckAndLoadPluginLibrary<Base>(derived_class)) {
    AERROR << "plugin of class " << derived_class << " have not been loaded";
    return nullptr;
  }
  return class_loader_manager_.CreateClassObj<Base>(derived_class);
}

template <typename Base>
std::string PluginManager::GetPluginClassHomePath(
    const std::string& class_name) {
  if (!CheckAndLoadPluginLibrary<Base>(class_name)) {
    AERROR << "plugin of class " << class_name << " have not been loaded";
    return "";
  }
  std::string library_path =
      class_loader_manager_.GetClassValidLibrary<Base>(class_name);
  if (library_path == "") {
    AWARN << "plugin of class " << class_name << " not found";
    return "";
  }
  for (auto it = plugin_description_map_.begin();
       it != plugin_description_map_.end(); ++it) {
    if (it->second->actual_library_path_ == library_path) {
      // TODO(liangjinping): remove hard code of relative prefix
      std::string relative_prefix = "share/";
      std::string relative_plugin_home_path =
          apollo::cyber::common::GetDirName(it->second->description_path_);
      if (relative_plugin_home_path.rfind(relative_prefix, 0) == 0) {
        relative_plugin_home_path =
            relative_plugin_home_path.substr(relative_prefix.size());
      }
      return relative_plugin_home_path;
    }
  }
  // not found
  return "";
}

template <typename Base>
std::string PluginManager::GetPluginConfPath(const std::string& class_name,
                                             const std::string& conf_name) {
  std::string plugin_home_path = GetPluginClassHomePath<Base>(class_name);
  if (apollo::cyber::common::PathIsAbsolute(plugin_home_path)) {
    // can not detect the plugin relative path
    AWARN << "plugin of class " << class_name << " load from absolute path, "
          << "conf path will be relative to it's description file";
  }

  std::string relative_conf_path = plugin_home_path + "/" + conf_name;
  std::string actual_conf_path;
  if (apollo::cyber::common::GetFilePathWithEnv(
          relative_conf_path, "APOLLO_CONF_PATH", &actual_conf_path)) {
    return actual_conf_path;
  }
  return plugin_home_path + "/" + conf_name;
}

template <typename Base>
bool PluginManager::IsLibraryLoaded(const std::string& class_name) {
  int status = 0;
  std::string base_class_name =
      abi::__cxa_demangle(typeid(Base).name(), 0, 0, &status);
  if (plugin_class_plugin_name_map_.find({class_name, base_class_name}) ==
      plugin_class_plugin_name_map_.end()) {
    // not found
    return false;
  }
  std::string plugin_name =
      plugin_class_plugin_name_map_[{class_name, base_class_name}];
  if (plugin_loaded_map_.find(plugin_name) == plugin_loaded_map_.end()) {
    // not found
    return false;
  }

  return plugin_loaded_map_[plugin_name];
}

template <typename Base>
bool PluginManager::CheckAndLoadPluginLibrary(const std::string& class_name) {
  if (IsLibraryLoaded<Base>(class_name)) {
    return true;
  }
  int status = 0;
  std::string base_class_name =
      abi::__cxa_demangle(typeid(Base).name(), 0, 0, &status);
  if (plugin_class_plugin_name_map_.find({class_name, base_class_name}) ==
      plugin_class_plugin_name_map_.end()) {
    // not found
    AWARN << "plugin of class " << class_name << " not found, "
          << "please check if it's registered";
    return false;
  }
  std::string plugin_name =
      plugin_class_plugin_name_map_[{class_name, base_class_name}];
  if (plugin_description_map_.find(plugin_name) ==
      plugin_description_map_.end()) {
    // not found
    AWARN << "plugin description of class " << class_name << " not found, "
          << "please check if it's loaded";
    return false;
  }
  auto plugin_description = plugin_description_map_[plugin_name];
  return LoadLibrary(plugin_description->actual_library_path_);
}

template <typename Base>
std::vector<std::string> PluginManager::GetDerivedClassNameByBaseClass() {
  int status = 0;
  std::string base_class_name =
      abi::__cxa_demangle(typeid(Base).name(), 0, 0, &status);
  std::vector<std::string> derived_class_name;
  for (const auto& iter : plugin_class_plugin_name_map_) {
    if (iter.first.second == base_class_name) {
      derived_class_name.push_back(iter.first.first);
    }
  }
  return derived_class_name;
}

#define CYBER_PLUGIN_MANAGER_REGISTER_PLUGIN(name, base) \
  CLASS_LOADER_REGISTER_CLASS(name, base)

}  // namespace plugin_manager
}  // namespace cyber
}  // namespace apollo
