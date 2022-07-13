/******************************************************************************
 * Copyright 2021 The Apollo Authors. All Rights Reserved.
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

#include "modules/v2x/proto/v2x_config.pb.h"

#include "cyber/common/file.h"
#include "modules/v2x/common/v2x_proxy_gflags.h"
#include "modules/v2x/v2x_proxy/app/hik_proxy.h"
#include "modules/v2x/v2x_proxy/app/v2x_proxy.h"

using apollo::v2x::V2xConfig;

int main(int argc, char *argv[]) {
  google::ParseCommandLineFlags(&argc, &argv, true);
  apollo::cyber::Init(argv[0]);
  std::unique_ptr<apollo::v2x::Proxy> proxy_ptr = nullptr;

  V2xConfig v2x_config;
  if (!apollo::cyber::common::GetProtoFromFile(
          apollo::v2x::FLAGS_v2x_config_file, &v2x_config)) {
    AERROR << "Failed to load config file: "
           << apollo::v2x::FLAGS_v2x_config_file;
    return 0;
  }

  AINFO << "The v2x config conf file is loaded: "
        << apollo::v2x::FLAGS_v2x_config_file;
  AINFO << "Conf is: " << v2x_config.ShortDebugString();

  switch (v2x_config.v2x_device()) {
    case V2xConfig::HIK: {
      proxy_ptr.reset(new apollo::v2x::HikProxy);
      AINFO << "sellect v2x proxy is HIK";
      break;
    }
    case V2xConfig::GRPC: {
      proxy_ptr.reset(new apollo::v2x::V2xProxy);
      AINFO << "sellect v2x proxy is GRPC";
      break;
    }
    default:
      break;
  }

  if (!proxy_ptr->InitFlag()) {
    AERROR << "Failed to initialize v2x proxy.";
    ::apollo::cyber::Clear();
    return -1;
  }
  ::apollo::cyber::WaitForShutdown();
  return 0;
}
