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

#ifndef PYTHON_WRAPPER_PY_INIT_H_
#define PYTHON_WRAPPER_PY_INIT_H_

#include <unistd.h>
#include <string>

#include "cyber/cyber.h"
#include "cyber/init.h"

namespace apollo {
namespace cyber {

bool py_init(const std::string& module_name) {
  static bool inited = false;
  if (inited) {
    AINFO << "cyber already inited.";
    return true;
  }

  if (!Init(module_name.c_str())) {
    AERROR << "cyber::Init failed:" << module_name;
    return false;
  }
  inited = true;
  AINFO << "cyber init succ.";
  return true;
}

bool py_ok() { return OK(); }

void py_shutdown() { return Clear(); }

bool py_is_shutdown() { return IsShutdown(); }

void py_waitforshutdown() { return WaitForShutdown(); }

}  // namespace cyber
}  // namespace apollo

#endif  // PYTHON_WRAPPER_PY_INIT_H_
