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

#include "cybertron/init.h"
#include "modules/dreamview/backend/dreamview.h"

int main(int argc, char *argv[]) {
  google::ParseCommandLineFlags(&argc, &argv, true);
  apollo::cybertron::Init(argv[0]);

  apollo::dreamview::Dreamview dreamview_;
  const bool init = dreamview_.Init().ok() && dreamview_.Start().ok();
  if (!init) {
    AERROR << "Failed to initialize dreamview server";
    return 0;
  }

  uint64_t count = 0;
  auto timer = apollo::cybertron::Timer(100,
                                        [&count]() {
                                          AINFO << "timer shot count: "
                                                << count;
                                          count++;
                                        },
                                        false);
  timer.Start();
  apollo::cybertron::WaitForShutdown();
  return 0;
}
