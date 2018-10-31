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

#ifndef CYBER_COMMON_MACROS_H_
#define CYBER_COMMON_MACROS_H_

#include <iostream>
#include <memory>

// There must be many copy-paste versions of these macros which are same
// things, undefine them to avoid conflict.
#undef UNUSED
#undef DISALLOW_COPY_AND_ASSIGN

#define UNUSED(param) (void)param

#define DISALLOW_COPY_AND_ASSIGN(classname) \
  classname(const classname &) = delete;    \
  classname &operator=(const classname &) = delete;

#define DECLARE_SINGLETON(classname)                                \
 public:                                                            \
  static const std::shared_ptr<classname> &Instance() {             \
    static auto instance =                                          \
        std::shared_ptr<classname>(new (std::nothrow) classname()); \
    return instance;                                                \
  }                                                                 \
                                                                    \
 private:                                                           \
  classname();                                                      \
  DISALLOW_COPY_AND_ASSIGN(classname)

#endif  // CYBER_COMMON_MACROS_H_
