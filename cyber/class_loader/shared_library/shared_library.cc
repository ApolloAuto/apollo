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
//
// Adapted from poco/Foundation/src/SharedLibrary_UNIX.cpp
//
// Copyright (c) 2004-2006, Applied Informatics Software Engineering GmbH.
// and Contributors.
//
// SPDX-License-Identifier: BSL-1.0
//
#include "cyber/class_loader/shared_library/shared_library.h"

#include <dlfcn.h>

namespace apollo {
namespace cyber {
namespace class_loader {

std::mutex SharedLibrary::mutex_;

SharedLibrary::SharedLibrary(const std::string& path) { Load(path, 0); }

SharedLibrary::SharedLibrary(const std::string& path, int flags) {
  Load(path, flags);
}

void SharedLibrary::Load(const std::string& path) { Load(path, 0); }

void SharedLibrary::Load(const std::string& path, int flags) {
  std::lock_guard<std::mutex> lock(mutex_);
  if (handle_) throw LibraryAlreadyLoadedException(path);

  int real_flag = RTLD_LAZY;
  if (flags & SHLIB_LOCAL) {
    real_flag |= RTLD_LOCAL;
  } else {
    real_flag |= RTLD_GLOBAL;
  }
  handle_ = dlopen(path.c_str(), real_flag);
  if (!handle_) {
    const char* err = dlerror();
    throw LibraryLoadException(err ? std::string(err) : path);
  }

  path_ = path;
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
    throw SymbolNotFoundException(name);
  }

  return result;
}

SharedLibrary::~SharedLibrary() {}

}  // namespace class_loader
}  // namespace cyber
}  // namespace apollo
