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

#ifndef CYBER_CLASS_LOADER_CLASS_LOADER_MANAGER_H_
#define CYBER_CLASS_LOADER_CLASS_LOADER_MANAGER_H_

#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include "cyber/class_loader/class_loader.h"

namespace apollo {
namespace cyber {
namespace class_loader {

class ClassLoaderManager {
 public:
  ClassLoaderManager();
  virtual ~ClassLoaderManager();

  bool LoadLibrary(const std::string& library_path);
  void UnloadAllLibrary();
  bool IsLibraryValid(const std::string& library_path);
  template <typename Base>
  std::shared_ptr<Base> CreateClassObj(const std::string& class_name);
  template <typename Base>
  std::shared_ptr<Base> CreateClassObj(const std::string& class_name,
                                       const std::string& library_path);
  template <typename Base>
  bool IsClassValid(const std::string& class_name);
  template <typename Base>
  std::vector<std::string> GetValidClassNames();

  /**
   * @brief get pathof  library that class belongs to
   * @param class_name derived class
   * @return path of library that containing the derived class
   */
  template <typename Base>
  std::string GetClassValidLibrary(const std::string& class_name);

 private:
  ClassLoader* GetClassLoaderByLibPath(const std::string& library_path);
  std::vector<ClassLoader*> GetAllValidClassLoaders();
  std::vector<std::string> GetAllValidLibPath();
  int UnloadLibrary(const std::string& library_path);

 private:
  std::mutex libpath_loader_map_mutex_;
  std::map<std::string, ClassLoader*> libpath_loader_map_;
};

template <typename Base>
std::shared_ptr<Base> ClassLoaderManager::CreateClassObj(
    const std::string& class_name) {
  std::vector<ClassLoader*> class_loaders = GetAllValidClassLoaders();
  for (auto class_loader : class_loaders) {
    if (class_loader->IsClassValid<Base>(class_name)) {
      return (class_loader->CreateClassObj<Base>(class_name));
    }
  }
  AERROR << "Invalid class name: " << class_name;
  return std::shared_ptr<Base>();
}

template <typename Base>
std::shared_ptr<Base> ClassLoaderManager::CreateClassObj(
    const std::string& class_name, const std::string& library_path) {
  ClassLoader* loader = GetClassLoaderByLibPath(library_path);
  if (loader) {
    return (loader->CreateClassObj<Base>(class_name));
  }
  AERROR << "Could not create classobj, there is no ClassLoader in: "
         << class_name;
  return std::shared_ptr<Base>();
}

template <typename Base>
bool ClassLoaderManager::IsClassValid(const std::string& class_name) {
  std::vector<std::string> valid_classes = GetValidClassNames<Base>();
  return (valid_classes.end() !=
          std::find(valid_classes.begin(), valid_classes.end(), class_name));
}

template <typename Base>
std::vector<std::string> ClassLoaderManager::GetValidClassNames() {
  std::vector<std::string> valid_classes;
  for (auto class_loader : GetAllValidClassLoaders()) {
    std::vector<std::string> class_loaders =
        class_loader->GetValidClassNames<Base>();
    valid_classes.insert(valid_classes.end(), class_loaders.begin(),
                         class_loaders.end());
  }
  return valid_classes;
}

template <typename Base>
std::string ClassLoaderManager::GetClassValidLibrary(
    const std::string& class_name) {
  for (auto& lib_class_loader : libpath_loader_map_) {
    if (lib_class_loader.second != nullptr) {
      if (lib_class_loader.second->IsClassValid<Base>(class_name)) {
        return lib_class_loader.first;
      }
    }
  }
  return "";
}

}  // namespace class_loader
}  // namespace cyber
}  // namespace apollo

#endif  // CYBER_CLASS_LOADER_CLASS_LOADER_MANAGER_H_
