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
#include "cyber/class_loader/shared_library/shared_library.h"
#include "cyber/class_loader/shared_library/exceptions.h"
#include "modules/dreamview/backend/sim_control_manager/core/dynamic_model_factory.h"

#include "modules/common/util/util.h"
#include "modules/dreamview/backend/common/dreamview_gflags.h"

// 专门用DynamicModel来管理这个类
namespace apollo
{
  namespace dreamview
  {

    using apollo::cyber::class_loader::LibraryAlreadyLoadedException;
    using apollo::cyber::class_loader::LibraryLoadException;
    using apollo::cyber::class_loader::SharedLibrary;
    using apollo::cyber::class_loader::SymbolNotFoundException;
    using ::std::string;
    using ::std::unordered_map;
    using SharedLibraryPtr = std::shared_ptr<SharedLibrary>;

    // 这个全局变量应该如何拿捏
    unordered_map<string, ::std::unique_ptr<SimControlBase>> s_dynamic_model_map_;

    DynamicModelFactory::DynamicModelFactory() : dynamic_model_local_path_("")
    {
      const std::string home = cyber::common::GetEnv("HOME");
      dynamic_model_local_path_ = home + FLAGS_resource_dynamic_model_path;
      // RegisterDynamicModels();
    }

    bool DynamicModelFactory::RegisterDynamicModel(std::string &dm_library_path, std::string &dm_name)
    {
      // 根据dynamic model library path to load dynamic model
      SharedLibraryPtr shared_library = nullptr;
      try
      {
        // todo(lijin):什么时候Unload。现在Unload会影响吗
        shared_library = SharedLibraryPtr(new SharedLibrary(dm_library_path));
        // Todo: use #define RegisterConditionHandler那样 全局整一下
        //  直接在这里创建类指针 指向DM类
        SimControlBase *dynamic_model_ptr = (SimControlBase *)shared_library->GetSymbol(dm_name);
        // 这里应该是调用函数 返回生成的指针
        s_dynamic_model_map_[dm_name].reset(dynamic_model_ptr);
        // Todo: load so only can load function,change to load class obj
      }
      catch (const LibraryLoadException &e)
      {
        AERROR << "LibraryLoadException: " << e.what();
        return false;
      }
      catch (const LibraryAlreadyLoadedException &e)
      {
        AERROR << "LibraryAlreadyLoadedException: " << e.what();
        return false;
      }
      catch (const SymbolNotFoundException &e)
      {
        AERROR << "SymbolNotFoundException: " << e.what();
        return false;
      }
      return true;
    }

    void DynamicModelFactory::GetDynamicModelPath(std::string &dynamic_model_name, std::string &path, bool get_library)
    {
      path = dynamic_model_local_path_ + dynamic_model_name;
      if (get_library)
      {
        path = path + "/" + dynamic_model_name + ".so";
      }
      return;
    }

    nlohmann::json DynamicModelFactory::RegisterDynamicModels()
    {
      nlohmann::json result = {};
      if (!cyber::common::PathExists(dynamic_model_local_path_))
      {
        AERROR << "Failed to find DynamicModel!No dynamic model locally,Or do not place it in correct location.";
        return result;
      }
      DIR *directory = opendir(dynamic_model_local_path_.c_str());
      if (directory == nullptr)
      {
        AERROR << "Cannot open directory " << dynamic_model_local_path_;
        return result;
      }
      struct dirent *file;
      std::string dynamic_model_name;
      std::string dynamic_model_library_path;
      while ((file = readdir(directory)) != nullptr)
      {
        // skip directory_path/. and directory_path/..
        if (!strcmp(file->d_name, ".") || !strcmp(file->d_name, ".."))
        {
          continue;
        }
        if (file->d_type != DT_DIR)
        {
          continue;
        }
        // 动态加载 注册成类
        dynamic_model_name = file->d_name;
        // 如果map里有 不要重复注册
        auto iter = s_dynamic_model_map_.find(dynamic_model_name);
        if (iter != s_dynamic_model_map_.end())
        {
          // 不要重复注册 避免重名
          continue;
        }
        // 1. 判断对应名字 eg:SimPointMass.so
        GetDynamicModelPath(dynamic_model_name, dynamic_model_library_path, true);
        // 判断路径 看是否存在要求的so+成功加载出来类
        if (!cyber::common::PathExists(dynamic_model_library_path))
        {
          AERROR << "Failed to load Dynamic Model: " << dynamic_model_name << ". Library is not exists!";
          continue;
        }
        // 去加载这个so
        bool register_res = RegisterDynamicModel(dynamic_model_library_path, dynamic_model_name);
        // 生成new class【这个地方需要写成#define去解决】
        if (register_res)
        {
          // 注册成功要返回给hmiStatus
          if (!result.contains("loaded_dynamic_models"))
          {
            result["loaded_dynamic_models"] = {};
          }
          result["loaded_dynamic_models"].push_back(dynamic_model_name);
        }
        // 存入map中
      }
      closedir(directory);
      // 给HMI回调一下
      // 让HMI管理到动力学模型
      return result;
    }

    bool DynamicModelFactory::GetModelType(std::string dynamic_model_name,SimControlBase* dynamic_model)
    {
      // 我觉得应该配置成字典
      auto iter = s_dynamic_model_map_.find(dynamic_model_name);
      if (iter == s_dynamic_model_map_.end())
      {
        // 返回空指针
        AERROR << "Failed to get " << dynamic_model_name << " related pointer.";
        return false;
      }
      // 记得参照：register condition handler
      dynamic_model = iter->second.get();
      return true;
    }

    void DynamicModelFactory::UnregisterDynamicModel(std::string &dynamic_model_name)
    {
      auto iter = s_dynamic_model_map_.find(dynamic_model_name);
      if (iter == s_dynamic_model_map_.end())
      {
        // 返回空指针
        AERROR << "Failed to get " << dynamic_model_name << " related pointer.";
        // 不需要下面的步骤，直接结束
        return;
      }
      // 动态指针自己可以管理的吧...
      s_dynamic_model_map_.erase(iter);
      std::string dynamic_model_dir;
      GetDynamicModelPath(dynamic_model_name, dynamic_model_dir, false);
      if (!cyber::common::PathExists(dynamic_model_dir))
      {
        AERROR << "Failed to find dynamic model path to delete!";
        return;
      }
      std::string command = "rm -fr " + dynamic_model_dir;
      // use cyber::common::removeFiles do not support sub-directory
      // use rmdir do not support not empty directory
      if (std::system(command.data()) != 0)
      {
        AERROR << "Failed to delete dynamic model directory for: "
               << std::strerror(errno);
        return;
      }
      return;
    }

  } // namespace dreamview
} // namespace apollo
