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
#include "cybertron/croutine/system/system_hook.h"

#include <dlfcn.h>
#include <stddef.h>
#include <unistd.h>

#include "cybertron/croutine/croutine.h"

extern "C" {
usleep_t g_usleep_func = NULL;
}

using apollo::cybertron::croutine::CRoutine;
using apollo::cybertron::croutine::RoutineState;

void system_hook_init() {
  g_usleep_func = (usleep_t)dlsym(RTLD_NEXT, "usleep");
}

int usleep(useconds_t usec) {
  if (!g_usleep_func) {
    system_hook_init();
  }

  auto routine = CRoutine::GetCurrentRoutine();
  if (routine == nullptr) {
    return g_usleep_func(usec);
  }
  routine->Sleep(usec);
  return 0;
}
