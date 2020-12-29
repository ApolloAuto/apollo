/******************************************************************************
 * Copyright 2020 The Apollo Authors. All Rights Reserved.
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

#ifndef CYBER_CLASS_LOADER_SHARED_LIBRARY_H_
#define CYBER_CLASS_LOADER_SHARED_LIBRARY_H_

#include <string>
#include <mutex>

#include "cyber/class_loader/shared_library/exceptions.h"

namespace apollo {
namespace cyber {
namespace class_loader {

class SharedLibrary {
 public:
  enum Flags {
    SHLIB_GLOBAL = 1,
    SHLIB_LOCAL = 2,
  };

  SharedLibrary();

  explicit SharedLibrary(const std::string& path);

  SharedLibrary(const std::string& path, int flags);

  virtual ~SharedLibrary();

  void Load(const std::string& path);

  void Load(const std::string& path, int flags);

  void Unload();

  bool IsLoaded();

  bool HasSymbol(const std::string& name);

  void* GetSymbol(const std::string& name);

  inline const std::string& GetPath() const { return path_; }

  SharedLibrary(const SharedLibrary&) = delete;
  SharedLibrary& operator=(const SharedLibrary&) = delete;

 private:
  void* handle_ = nullptr;
  std::string path_;

  std::mutex mutex_;
};

}  // namespace class_loader
}  // namespace cyber
}  // namespace apollo

#endif  // CYBER_CLASS_LOADER_SHARED_LIBRARY_H_
