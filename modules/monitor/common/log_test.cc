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

// TODO(xiaoxq): complete this using gmock & gtest, capture logging contents and
// do checks.

#include "modules/monitor/common/log.h"

int main() {
  apollo::monitor::log::LogModule m1 = {
      "test", 5, 10, apollo::monitor::log::platform_log_printf};
  m1.set_log_lvl(7);

  PLATFORM_LOG(&m1, 3, "test");

  return 0;
}
