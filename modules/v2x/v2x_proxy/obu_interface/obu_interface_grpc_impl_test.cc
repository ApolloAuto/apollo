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
 * @file obu_interface_grpc_impl_test.cc
 * @brief test v2x proxy module and onboard unit interface grpc impl class
 */

#include "modules/v2x/v2x_proxy/obu_interface/obu_interface_grpc_impl.h"

#include "gtest/gtest.h"

namespace apollo {
namespace v2x {

TEST(ObuInterFaceGrpcImplTest, Construct) {
  apollo::cyber::Init("obu_interface_grpc_impl_test");
  ObuInterFaceGrpcImpl obuinteface;
  EXPECT_TRUE(obuinteface.InitFlag());
}
}  // namespace v2x
}  // namespace apollo
