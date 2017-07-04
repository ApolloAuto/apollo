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

#include "modules/hmi/utils/restful_client.h"

#include "gflags/gflags.h"
#include "gtest/gtest.h"

#include "modules/hmi/proto/runtime_status.pb.h"

DEFINE_string(hmi_runtime_status_api,
              "http://127.0.0.1:123/runtime_status_api_fake",
              "Address of HMI runtime status restful api.");

namespace apollo {
namespace hmi {

TEST(RestfulClientTest, Post) {
  ModuleStatus module_status;
  module_status.set_name("control");
  module_status.set_status(::apollo::hmi::ModuleStatus::STARTED);

  RestfulClient client(FLAGS_hmi_runtime_status_api);
  EXPECT_TRUE(client.Post(module_status) == RestfulClient::RUNTIME_ERROR);
}

}  // namespace hmi
}  // namespace apollo
