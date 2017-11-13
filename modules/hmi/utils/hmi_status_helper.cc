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

#include "modules/hmi/utils/hmi_status_helper.h"

#include "gflags/gflags.h"
#include "modules/common/util/file.h"
#include "modules/common/util/string_util.h"
#include "modules/hmi/proto/config.pb.h"
#include "modules/hmi/utils/restful_client.h"

DEFINE_string(hmi_config_file, "modules/hmi/conf/config.pb.txt",
              "HMI config file, which should be text-formatted config proto.");

namespace apollo {
namespace hmi {
namespace {

RestfulClient* InitRestfulClient() {
  Config conf_pb;
  if (!apollo::common::util::GetProtoFromASCIIFile(FLAGS_hmi_config_file,
                                                   &conf_pb)) {
    return nullptr;
  }
  const auto& server = conf_pb.server();
  const auto url = apollo::common::util::StrCat(
      server.https().enabled() ? "https://" : "http://",
      "127.0.0.1:", server.port(), "/runtime_status");
  return new RestfulClient(url);
}

void ReportRuntimeStatus(const RuntimeStatus& runtime_status) {
  static auto* client = InitRestfulClient();
  if (client) {
    client->Post(runtime_status);
  } else {
    ADEBUG << "Cannot connect to HMI server.";
  }
}

}  // namespace

void HMIStatusHelper::ReportHardwareStatus(
    const std::vector<HardwareStatus> &hardware_status) {
  RuntimeStatus runtime_status;
  *runtime_status.mutable_hardware() = {hardware_status.begin(),
                                        hardware_status.end()};
  ReportRuntimeStatus(runtime_status);
}

void HMIStatusHelper::ReportModuleStatus(const ModuleStatus &module_status) {
  RuntimeStatus runtime_status;
  *runtime_status.add_modules() = module_status;
  ReportRuntimeStatus(runtime_status);
}

}  // namespace hmi
}  // namespace apollo
