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
 * @file grpc_server_test.cc
 * @brief test v2x proxy module and onboard unit interface grpc implement
 */

#include "modules/v2x/v2x_proxy/obu_interface/grpc_interface/grpc_server.h"

#include "gtest/gtest.h"

namespace apollo {
namespace v2x {

TEST(GrpcServerImplTest, Construct) {
  apollo::cyber::Init("grpc_server_test");
  bool init_succ = false;
  GrpcServerImpl grpc_server;
  init_succ = grpc_server.InitFlag();
  EXPECT_TRUE(init_succ);
  std::shared_ptr<::apollo::v2x::obu::ObuTrafficLight> ptr = nullptr;
  grpc_server.GetMsgFromGrpc(&ptr);
  EXPECT_EQ(nullptr, ptr);
}
}  // namespace v2x
}  // namespace apollo
