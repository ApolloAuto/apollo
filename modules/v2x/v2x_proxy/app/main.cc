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

/**
 * @file main.cc
 * @brief v2x proxy main function
 */
#include "modules/v2x/v2x_proxy/app/v2x_proxy.h"

int main(int argc, char *argv[]) {
  google::ParseCommandLineFlags(&argc, &argv, true);
  ::apollo::cyber::Init(argv[0]);
  ::apollo::v2x::V2xProxy v2x_proxy;
  if (!v2x_proxy.InitFlag()) {
    AERROR << "Failed to initialize v2x proxy";
    ::apollo::cyber::Clear();
    return -1;
  }
  ::apollo::cyber::WaitForShutdown();
  return 0;
}
