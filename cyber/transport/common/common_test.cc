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

#include <sys/epoll.h>
#include <unistd.h>

#include <string>
#include "gtest/gtest.h"

#include "cyber/common/global_data.h"
#include "cyber/transport/common/endpoint.h"
#include "cyber/transport/common/identity.h"

namespace apollo {
namespace cyber {
namespace transport {

TEST(IdentityTest, identity_test) {
  Identity id1;
  Identity id2;
  Identity id3;
  EXPECT_NE(id1, id2);
  EXPECT_NE(id2, id3);
  EXPECT_NE(id1, id3);
  EXPECT_NE(id1.HashValue(), id3.HashValue());

  id2 = id2;
  EXPECT_NE(id2, id3);
  id2 = id3;
  EXPECT_EQ(id2, id3);

  Identity id4(id1);
  EXPECT_EQ(id1, id4);
  EXPECT_EQ(id1.ToString(), id4.ToString());
  EXPECT_EQ(id1.HashValue(), id4.HashValue());
}

TEST(EndpointTest, endpoint_test) {
  // empty attr
  RoleAttributes attr;
  Endpoint endpoint1(attr);
  EXPECT_EQ(common::GlobalData::Instance()->HostName(),
            endpoint1.attributes().host_name());
  EXPECT_EQ(common::GlobalData::Instance()->ProcessId(),
            endpoint1.attributes().process_id());
  EXPECT_NE(0, endpoint1.attributes().id());

  attr.set_host_name("caros");
  attr.set_process_id(1024);
  attr.set_id(123);
  Endpoint endpoint2(attr);
  EXPECT_EQ("caros", endpoint2.attributes().host_name());
  EXPECT_EQ(1024, endpoint2.attributes().process_id());
  EXPECT_EQ(123, endpoint2.attributes().id());
  EXPECT_NE(std::string("endpoint"), std::string(endpoint2.id().data()));
}

}  // namespace transport
}  // namespace cyber
}  // namespace apollo
