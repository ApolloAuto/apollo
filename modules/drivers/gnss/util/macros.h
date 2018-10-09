/******************************************************************************
 * Copyright 2017 The Apollo Authors. All Rights Reserved.
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

// Commonly-used macro definitions. Some of them are copied from
// https://chromium.googlesource.com/chromium/src/base/+/master/macros.h

#pragma once

#include <cstddef>

// Put this in the declarations for a class to be uncopyable.
#define DISABLE_COPY(TypeName) TypeName(const TypeName &) = delete

// Put this in the declarations for a class to be unassignable.
#define DISABLE_ASSIGN(TypeName) void operator=(const TypeName &) = delete

// A macro to disallow the copy constructor and operator= functions.
// This should be used in the private: declarations for a class.
#define DISABLE_COPY_AND_ASSIGN(TypeName) \
  DISABLE_COPY(TypeName);                 \
  DISABLE_ASSIGN(TypeName)

// A macro to disallow all the implicit constructors, namely the
// default constructor, copy constructor and operator= functions.
//
// This should be used in the private: declarations for a class
// that wants to prevent anyone from instantiating it. This is
// especially useful for classes containing only static methods.
#define DISABLE_IMPLICIT_CONSTRUCTORS(TypeName) \
  TypeName() = delete;                          \
  DISABLE_COPY_AND_ASSIGN(TypeName)

// Creates a thread-safe singleton.
#define MAKE_SINGLETON(TypeName) \
 public:                         \
  static TypeName *Instance() {  \
    static TypeName Instance;    \
    return &Instance;            \
  }                              \
                                 \
 private:                        \
  DISABLE_COPY_AND_ASSIGN(TypeName)

namespace apollo {

// array_size(a) returns the number of elements in a.
template <class T, size_t N>
constexpr size_t array_size(T (&)[N]) {
  return N;
}

}  // namespace apollo
