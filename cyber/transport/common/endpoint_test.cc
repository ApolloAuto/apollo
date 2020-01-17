/******************************************************************************
 * Copyright 2019 The Apollo Authors. All Rights Reserved.
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

#include "cyber/transport/common/endpoint.h"

#include "gtest/gtest.h"

#include "cyber/common/global_data.h"

namespace apollo {
namespace cyber {
namespace transport {

TEST(EndpointTest, construction) {
  proto::RoleAttributes role;

  {
    Endpoint e0(role);
    const proto::RoleAttributes& erole = e0.attributes();
    EXPECT_EQ(erole.host_name(), common::GlobalData::Instance()->HostName());
    EXPECT_EQ(erole.process_id(), common::GlobalData::Instance()->ProcessId());
    EXPECT_EQ(erole.id(), e0.id().HashValue());
  }
  {
    role.set_host_name("123");
    role.set_process_id(54321);
    role.set_id(123);

    Endpoint e0(role);
    const proto::RoleAttributes& erole = e0.attributes();
    EXPECT_EQ(erole.host_name(), "123");
    EXPECT_EQ(erole.process_id(), 54321);
    EXPECT_NE(erole.id(), e0.id().HashValue());
  }
}

}  // namespace transport
}  // namespace cyber
}  // namespace apollo
