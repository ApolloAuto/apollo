/******************************************************************************
 * Copyright 2023 The Apollo Authors. All Rights Reserved.
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

#include "cyber/context/context.h"

#include <memory>

#include "gtest/gtest.h"

namespace apollo {
namespace cyber {
namespace context {

TEST(Context, Set) {
  Context ctx;
  ctx.Set("key", 1);
  ctx.Set("key", std::make_shared<int>(2));
  EXPECT_EQ(*ctx.Get<int>("key"), 2);

  struct Foo {
    int a;
    int b;
  };
  Foo foo{1, 2};
  ctx.Set("key", foo);
  EXPECT_EQ(ctx.Get<Foo>("key")->a, 1);
  EXPECT_EQ(ctx.Get<Foo>("key")->b, 2);
}

TEST(Context, Get) {
  Context ctx;
  ctx.Set("key1", 1);
  ctx.Set("key2", std::make_shared<int>(2));
  EXPECT_EQ(*ctx.Get<int>("key1"), 1);
  EXPECT_EQ(*ctx.Get<int>("key2"), 2);
}

}  // namespace context
}  // namespace cyber
}  // namespace apollo
