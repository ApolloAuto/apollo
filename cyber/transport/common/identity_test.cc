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

#include "cyber/transport/common/identity.h"

#include "gtest/gtest.h"

namespace apollo {
namespace cyber {
namespace transport {

TEST(IdentityTest, testConstructFalse) {
  Identity it(false);

  EXPECT_EQ(it.HashValue(), static_cast<uint64_t>(0));
  EXPECT_EQ(it.ToString(), "0");
}

TEST(IdentityTest, testConstructTrue) {
  Identity it(true);

  EXPECT_NE(it.HashValue(), static_cast<uint64_t>(0));
  EXPECT_NE(it.ToString(), "0");
}

TEST(IdentityTest, testIdentityEqual) {
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

TEST(IdentityTest, testOperatorEqual) {
  Identity it(true);

  Identity it1(false);

  it1 = it;

  EXPECT_EQ(it.HashValue(), it1.HashValue());
  EXPECT_EQ(it.ToString(), it1.ToString());
  EXPECT_EQ(it1.Length(), it.Length());

  it.set_data(nullptr);
  Identity it2;

  it2.set_data(it.data());
  EXPECT_EQ(it2.HashValue(), it1.HashValue());
  EXPECT_EQ(it2.ToString(), it1.ToString());
  EXPECT_EQ(it1.Length(), it2.Length());
}

}  // namespace transport
}  // namespace cyber
}  // namespace apollo
