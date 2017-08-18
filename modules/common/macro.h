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

/**
 * @file
 */

#ifndef MODULES_COMMON_MACRO_H_
#define MODULES_COMMON_MACRO_H_

#include <ctime>
#include <iostream>

#define DISALLOW_COPY_AND_ASSIGN(classname) \
 private:                                   \
  classname(const classname &);             \
  classname &operator=(const classname &);

#define DISALLOW_IMPLICIT_CONSTRUCTORS(classname) \
 private:                                         \
  classname();                                    \
  DISALLOW_COPY_AND_ASSIGN(classname);

#define DECLARE_SINGLETON(classname)        \
 public:                                    \
  static classname *instance() {            \
    static classname instance;              \
    return &instance;                       \
  }                                         \
  DISALLOW_IMPLICIT_CONSTRUCTORS(classname) \
 private:

// Measure run time of a code block, for debugging puprpose only, don't check in
// code with this macro!
// Example usage:
// PERF_BLOCK("Function Foo took: ") {
//  Foo();
// }
#define PERF_BLOCK(message)                                                 \
  for (long block_start_time = 0;                                           \
       (block_start_time == 0 ? (block_start_time = std::clock()) : false); \
       std::cout << std::fixed << message << ": "                           \
                 << static_cast<double>(std::clock() - block_start_time) /  \
                        CLOCKS_PER_SEC                                      \
                 << std::endl)

#endif  // MODULES_COMMON_MACRO_H_
