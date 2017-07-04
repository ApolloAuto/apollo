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
#include "modules/hmi/utils/restful_client.h"

DEFINE_string(hmi_runtime_status_api,
              "http://127.0.0.1:8887/runtime_status_api",
              "Address of HMI runtime status restful api.");

namespace apollo {
namespace hmi {
namespace {

template <class T>
void VectorToRepeatedPtrField(const std::vector<T>& src,
                              google::protobuf::RepeatedPtrField<T>* dst) {
  *dst = google::protobuf::RepeatedPtrField<T>(src.begin(), src.end());
}

}  // namespace


void HMIStatusHelper::ReportHardwareStatus(
    const std::vector<HardwareStatus>& hardware_status) {
  auto runtime_status = RuntimeStatus();
  VectorToRepeatedPtrField(hardware_status, runtime_status.mutable_hardware());

  RestfulClient client(FLAGS_hmi_runtime_status_api);
  client.Post(runtime_status);
}

void HMIStatusHelper::ReportModuleStatus(const ModuleStatus& module_status) {
  auto runtime_status = RuntimeStatus();
  *runtime_status.add_modules() = module_status;

  RestfulClient client(FLAGS_hmi_runtime_status_api);
  client.Post(runtime_status);
}

}  // namespace hmi
}  // namespace apollo
