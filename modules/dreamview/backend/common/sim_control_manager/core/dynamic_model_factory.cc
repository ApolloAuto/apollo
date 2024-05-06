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
#include "modules/dreamview/backend/common/sim_control_manager/core/dynamic_model_factory.h"

#include "modules/dreamview/backend/common/sim_control_manager/proto/dynamic_model_conf.pb.h"

#include "cyber/class_loader/shared_library/exceptions.h"
#include "cyber/class_loader/shared_library/shared_library.h"
#include "modules/common/util/util.h"
#include "modules/dreamview/backend/common/dreamview_gflags.h"
#include "modules/dreamview/backend/common/map_service/map_service.h"
#include "modules/dreamview/backend/common/sim_control_manager/dynamic_model/perfect_control/sim_perfect_control.h"

namespace apollo {
namespace dreamview {

using apollo::cyber::class_loader::LibraryAlreadyLoadedException;
using apollo::cyber::class_loader::LibraryLoadException;
using apollo::cyber::class_loader::SharedLibrary;
using apollo::cyber::class_loader::SymbolNotFoundException;
using ::std::string;
using ::std::unordered_map;
using SharedLibraryPtr = std::shared_ptr<SharedLibrary>;

struct DynamicModelInfo {
  std::string dynamic_model_name;
  std::string library_name;
  SimControlBase *dynamic_model_ptr;
  std::string depend_model_package;
};
unordered_map<string, DynamicModelInfo> s_dynamic_model_map_;
unordered_map<string, int> s_dm_lib_count_;

DynamicModelFactory::DynamicModelFactory() : dynamic_model_local_path_("") {
  home_path_ = cyber::common::GetEnv("HOME");
  dynamic_model_local_path_ = home_path_ + FLAGS_resource_dynamic_model_path;
  // init should register default sim control:SimPerfectControl
  RegisterSimPerfectControl();
}

DynamicModelFactory::~DynamicModelFactory() {
  for (auto iter = s_dynamic_model_map_.begin();
       iter != s_dynamic_model_map_.end(); iter++) {
    delete iter->second.dynamic_model_ptr;
    iter->second.dynamic_model_ptr = nullptr;
  }
}

void DynamicModelFactory::RegisterSimPerfectControl() {
  // map_service当参数
  if (s_dynamic_model_map_.find(FLAGS_sim_perfect_control) ==
      s_dynamic_model_map_.end()) {
    // Avoid dumplicate register dynamic model which is already registered
    s_dynamic_model_map_[FLAGS_sim_perfect_control] = {};
    s_dynamic_model_map_[FLAGS_sim_perfect_control].dynamic_model_name =
        FLAGS_sim_perfect_control;
    s_dynamic_model_map_[FLAGS_sim_perfect_control].dynamic_model_ptr =
        new SimPerfectControl(new MapService());
  }
}

bool DynamicModelFactory::RegisterDynamicModel(const std::string &dm_dir_name) {
  std::string dynamic_model_conf_json_path;
  GetDynamicModelPath(dm_dir_name, &dynamic_model_conf_json_path, true);
  if (!cyber::common::PathExists(dynamic_model_conf_json_path)) {
    AERROR << "Failed to load Dynamic Model: " << dm_dir_name
           << ". conf file is not exists!";
    return false;
  }
  DynamicModelConf dynamic_model_conf;
  if (!cyber::common::GetProtoFromJsonFile(dynamic_model_conf_json_path,
                                           &dynamic_model_conf)) {
    AERROR << "Unable to parse Dynamic model conf from file "
           << dynamic_model_conf_json_path;
    return false;
  }
  std::string dynamic_model_name = dynamic_model_conf.dynamic_model_name();
  // get library name to load dm class
  if (!dynamic_model_conf.has_library_name() ||
      !dynamic_model_conf.has_dynamic_model_name() ||
      !dynamic_model_conf.has_depend_model_package()) {
    AERROR << "Missing required field!";
    return false;
  }
  // if is already registered
  auto iter = s_dynamic_model_map_.find(dynamic_model_name);
  if (iter != s_dynamic_model_map_.end()) {
    AERROR << "This dynamic model:  " << dynamic_model_name
           << " is already registered!";
    return false;
  }
  std::string dm_library_name = dynamic_model_conf.library_name();
  std::string depend_model_package = dynamic_model_conf.depend_model_package();
  std::replace(depend_model_package.begin(), depend_model_package.end(), '-', '_');

  std::string dynamic_model_package_library_path =
      FLAGS_dynamic_model_package_library_path +
      depend_model_package + "/" + dm_library_name;
  SharedLibraryPtr shared_library = nullptr;
  AINFO << "dm_library_name: " << dm_library_name;
  AINFO << "dynamic_model_package_library_path: " << dynamic_model_package_library_path;

  try {
    // todo(@lijin):when to unload
    shared_library =
        SharedLibraryPtr(new SharedLibrary(dynamic_model_package_library_path));
    create_t *create_dynamic_model =
        reinterpret_cast<create_t *>(shared_library->GetSymbol("create"));
    SimControlBase *dynamic_model_ptr =
        create_dynamic_model(dm_dir_name, home_path_);
    if (!dynamic_model_ptr) {
      return false;
    }
    s_dynamic_model_map_[dynamic_model_name] = {};
    s_dynamic_model_map_[dynamic_model_name].dynamic_model_name =
        dynamic_model_name;
    s_dynamic_model_map_[dynamic_model_name].dynamic_model_ptr =
        dynamic_model_ptr;
    s_dynamic_model_map_[dynamic_model_name].library_name = dm_library_name;
    s_dynamic_model_map_[dynamic_model_name].depend_model_package =
        dynamic_model_conf.depend_model_package();
    auto iter = s_dm_lib_count_.find(dm_library_name);
    if (iter == s_dm_lib_count_.end()) {
      s_dm_lib_count_[dm_library_name] = 1;
    } else {
      s_dm_lib_count_[dm_library_name]++;
    }
  } catch (const LibraryLoadException &e) {
    AERROR << "LibraryLoadException: " << e.what();
    return false;
  } catch (const LibraryAlreadyLoadedException &e) {
    AERROR << "LibraryAlreadyLoadedException: " << e.what();
    return false;
  } catch (const SymbolNotFoundException &e) {
    AERROR << "SymbolNotFoundException: " << e.what();
    return false;
  }
  return true;
}

void DynamicModelFactory::GetDynamicModelPath(
    const std::string &dynamic_model_name, std::string *path,
    bool get_conf_json) {
  CHECK_NOTNULL(path);
  *path = dynamic_model_local_path_ + dynamic_model_name;
  if (get_conf_json) {
    *path = *path + "/dynamic_model.json";
  }
  return;
}

nlohmann::json DynamicModelFactory::RegisterDynamicModels() {
  nlohmann::json result = {};
  result["result"] = true;
  result["loaded_dynamic_models"] = {};
  if (!cyber::common::PathExists(dynamic_model_local_path_)) {
    AERROR << "Failed to find DynamicModel!No dynamic model locally,Or do not "
              "place it in correct location.";
  } else {
    DIR *directory = opendir(dynamic_model_local_path_.c_str());
    if (directory == nullptr) {
      AERROR << "Cannot open directory " << dynamic_model_local_path_;
    } else {
      struct dirent *file;
      std::string dynamic_model_dir_name;
      // std::string dynamic_model_package_library_path;
      while ((file = readdir(directory)) != nullptr) {
        // skip directory_path/. and directory_path/..
        if (!strcmp(file->d_name, ".") || !strcmp(file->d_name, "..")) {
          continue;
        }
        if (file->d_type != DT_DIR) {
          continue;
        }
        dynamic_model_dir_name = file->d_name;
        // avpid dumplicate register dynamic model
        RegisterDynamicModel(dynamic_model_dir_name);
      }
      closedir(directory);
    }
  }

  // c++ map's traversal order is different from the insertion order.
  // To ensure that the default sim control is in the front,put it before other
  // dynamic models.
  result["loaded_dynamic_models"] = {FLAGS_sim_perfect_control};
  for (auto iter = s_dynamic_model_map_.begin();
       iter != s_dynamic_model_map_.end(); iter++) {
    if (iter->first != FLAGS_sim_perfect_control) {
      result["loaded_dynamic_models"].push_back(iter->first);
    }
  }
  return result;
}

SimControlBase *DynamicModelFactory::GetModelType(
    std::string dynamic_model_name) {
  auto iter = s_dynamic_model_map_.find(dynamic_model_name);
  if (iter == s_dynamic_model_map_.end()) {
    AERROR << "Failed to get " << dynamic_model_name << " related pointer.";
    return nullptr;
  }
  return iter->second.dynamic_model_ptr;
}

bool DynamicModelFactory::UnregisterDynamicModel(
    const std::string &dynamic_model_name) {
  auto iter = s_dynamic_model_map_.find(dynamic_model_name);
  if (iter == s_dynamic_model_map_.end()) {
    AERROR << "Failed to get " << dynamic_model_name << " related pointer.";
    return true;
  }
  std::string library_name = iter->second.library_name;
  s_dynamic_model_map_.erase(dynamic_model_name);
  std::string dynamic_model_dir;
  GetDynamicModelPath(dynamic_model_name, &dynamic_model_dir, false);
  std::string command = "rm -fr " + dynamic_model_dir;
  // use cyber::common::removeFiles do not support sub-directory
  // use rmdir do not support not empty directory
  if (std::system(command.data()) != 0) {
    AERROR << "Failed to delete dynamic model directory for: "
           << std::strerror(errno);
    return false;
  }
  // delete related library if library is only used by this dynamic model
  auto count_iter = s_dm_lib_count_.find(library_name);
  std::string depend_model_package = iter->second.depend_model_package;
  if (count_iter->second == 1) {
    std::string lib_path =
        home_path_ + FLAGS_dynamic_model_library_path + library_name;
    cyber::common::DeleteFile(lib_path);
    // todo(chenhuanguang): confirm delete package command
    // std::string remove_package_command = "";
    // if (std::system(command.data()) != 0) {
    //   AERROR << "Failed to remove package: " << depend_model_package 
    //         << " error: " << std::strerror(errno);
    //   return false;
    // }
    s_dm_lib_count_.erase(library_name);
  } else {
    s_dm_lib_count_[library_name]--;
  }
  auto dynamic_model_ptr = iter->second.dynamic_model_ptr;
  delete dynamic_model_ptr;

  return true;
}

}  // namespace dreamview
}  // namespace apollo
