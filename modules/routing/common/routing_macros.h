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

#ifndef BAIDU_ADU_ROUTING_COMMON_ROUTING_MACROS_H
#define BAIDU_ADU_ROUTING_COMMON_ROUTING_MACROS_H

#include <cstddef>

#define DISALLOW_COPY_AND_ASSIGN(classname) \
 private:                                   \
  classname(const classname&);              \
  classname& operator=(const classname&);

#define DISALLOW_IMPLICIT_CONSTRUCTORS(classname) \
 private:                                         \
  classname();                                    \
  DISALLOW_COPY_AND_ASSIGN(classname);

#define DECLARE_ARBITER_SINGLETON(classname) \
 public:                                     \
  static classname* instance() {             \
    static classname instance;               \
    return &instance;                        \
  };                                         \
                                             \
  DISALLOW_IMPLICIT_CONSTRUCTORS(classname)  \
                                             \
 private:

#endif  // BAIDU_ADU_ROUTING_COMMON_ROUTING_MACROS_H
