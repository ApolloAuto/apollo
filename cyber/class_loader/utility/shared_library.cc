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

#include "cyber/class_loader/utility/shared_library.h"

#include <dlfcn.h>

namespace apollo {
namespace cyber {
namespace class_loader {
namespace utility {

SharedLibrary::SharedLibrary() : handle_(nullptr) {}

SharedLibrary::SharedLibrary(const std::string& filename) {
  Load(filename, 0);
}

SharedLibrary::SharedLibrary(const std::string& filename, int flags) {
  Load(filename, flags);
}

void SharedLibrary::Load(const std::string& filename) {
  Load(filename, 0);
}

void SharedLibrary::Load(const std::string& filename, int flags) {
  std::lock_guard<std::mutex> lock(mutex_);
  if (handle_) throw LibraryAlreadyLoadedException(filename);

  int real_flag = RTLD_LAZY;
  if (flags & SHLIB_LOCAL) {
    real_flag |= SHLIB_LOCAL;
  } else {
    real_flag |= SHLIB_GLOBAL;
  }
  handle_ = dlopen(filename.c_str(), real_flag);
  if (!handle_) {
    const char* err = dlerror();
    throw LibraryLoadException(err ? std::string(err) : filename);
  }

  filename_ = filename;
}

void SharedLibrary::Unload() {
  std::lock_guard<std::mutex> lock(mutex_);
  if (handle_) {
    dlclose(handle_);
    handle_ = nullptr;
  }
}

bool SharedLibrary::IsLoaded() {
  std::lock_guard<std::mutex> lock(mutex_);
  return handle_ != nullptr;
}

bool SharedLibrary::HasSymbol(const std::string& name) {
  return GetSymbol(name) != nullptr;
}

void* SharedLibrary::GetSymbol(const std::string& name) {
  std::lock_guard<std::mutex> lock(mutex_);
  if (!handle_) return nullptr;

  void* result = dlsym(handle_, name.c_str());
  if (!result) {
    throw NotFoundException(name);
  }

  return result;
}

SharedLibrary::~SharedLibrary() {
  Unload();
}


}  // namespace utility
}  // namespace class_loader
}  // namespace cyber
}  // namespace apollo
