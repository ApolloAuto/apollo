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

/// @brief 用于 创建插件类的实例，并返回一个 std::shared_ptr<Base> 指针
/// @tparam Base 基类类型，表示插件的基本类型
/// @param class_name 插件的具体类名（字符串）
/// @return std::shared_ptr<Base>：插件实例的 智能指针
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

/// @brief 根据插件类的名称和配置文件名称，返回插件配置文件的路径
/// @tparam Base 
/// @param class_name 插件类的名称
/// @param conf_name 配置文件的名称
/// @return 插件配置文件的路径
template <typename Base>
std::string PluginManager::GetPluginConfPath(const std::string& class_name,
                                             const std::string& conf_name) {
// 获取插件类的主路径（即插件的根目录路径）。它将插件类名 class_name 作为参数传递，并且 Base 是模板参数类型
  std::string plugin_home_path = GetPluginClassHomePath<Base>(class_name);
  // 测路径是否为绝对路径
  if (apollo::cyber::common::PathIsAbsolute(plugin_home_path)) {
    // can not detect the plugin relative path
 // 如果插件路径是绝对路径，则输出警告信息，表示无法检测插件的相对路径，并且配置文件路径将相对于插件描述文件进行解析
    AWARN << "plugin of class " << class_name << " load from absolute path, "
          << "conf path will be relative to it's description file";
  }
// 将插件主路径 plugin_home_path 和配置文件名 conf_name 拼接成一个相对的配置文件路径 relative_conf_path
  std::string relative_conf_path = plugin_home_path + "/" + conf_name;
  std::string actual_conf_path; // 存储实际的配置文件路径‘
// 根据环境变量 APOLLO_CONF_PATH 查找实际的配置文件路径。如果能找到配置文件，函数返回 true，并将实际路径存储在 actual_conf_path 中
  if (apollo::cyber::common::GetFilePathWithEnv(
          relative_conf_path, "APOLLO_CONF_PATH", &actual_conf_path)) {
    return actual_conf_path;
  }
// 如果 GetFilePathWithEnv 没有找到配置文件路径，函数将返回由插件主路径 plugin_home_path 和配置文件名 conf_name 拼接而成的默认路径
  return plugin_home_path + "/" + conf_name;
}

/// @brief 检查一个插件库是否已经加载
/// @tparam Base 模板类型参数，表示插件的基类
/// @param class_name 传入的类名，用来在插件中查找特定的类
/// @return 
template <typename Base>
bool PluginManager::IsLibraryLoaded(const std::string& class_name) {
  int status = 0; // 接收 abi::__cxa_demangle 的返回状态（该函数用来获取类型名称的可读格式）
// typeid(Base).name() 获取 Base 类型的原始名称（通常是经过编译器编码的），
// 然后通过 abi::__cxa_demangle 转换为人类可读的类型名称，并赋值给 base_class_name
  std::string base_class_name =
      abi::__cxa_demangle(typeid(Base).name(), 0, 0, &status);
// 在 plugin_class_plugin_name_map_ 中查找一个由 class_name 和 base_class_name 组成的键值对
  if (plugin_class_plugin_name_map_.find({class_name, base_class_name}) ==
      plugin_class_plugin_name_map_.end()) {
    // not found
    return false;
  }
// 如果找到了插件，取出插件的名字，赋值给 plugin_name
  std::string plugin_name =
      plugin_class_plugin_name_map_[{class_name, base_class_name}];
// 在 plugin_loaded_map_ 中查找 plugin_name 是否存在。如果未找到，表示该插件未加载
  if (plugin_loaded_map_.find(plugin_name) == plugin_loaded_map_.end()) {
    // not found
    return false;
  }
  // 返回该插件的加载状态值
  return plugin_loaded_map_[plugin_name];
}

/// @brief 检查并加载指定的插件库
/// @tparam Base 
/// @param class_name 
// 接受一个类型 Base 和一个参数 class_name，该参数是插件类的名称，返回一个布尔值，表示插件库是否成功加载
/// @return 
template <typename Base>
bool PluginManager::CheckAndLoadPluginLibrary(const std::string& class_name) {
  // 检查是否已经加载了指定的插件库
  if (IsLibraryLoaded<Base>(class_name)) {
    return true;
  }
  int status = 0;
  // typeid(Base).name() 获取 Base 类型的名字，然后通过 abi::__cxa_demangle 函数将其转化为人类可读的格式，
  // 存储在 base_class_name 变量中。如果转换失败，status 会被设置为非零值
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
