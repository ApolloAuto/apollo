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

#include "cyber/base/unbounded_queue.h"

#include "gtest/gtest.h"

namespace apollo {
namespace cyber {
namespace base {

TEST(UnboundedQueueTest, all_in_one) {
  UnboundedQueue<int> q;
  EXPECT_TRUE(q.Empty());
  EXPECT_EQ(q.Size(), 0);

  int front = 0;
  EXPECT_FALSE(q.Dequeue(&front));

  q.Enqueue(1);
  EXPECT_FALSE(q.Empty());
  EXPECT_EQ(q.Size(), 1);

  q.Enqueue(2);
  EXPECT_EQ(q.Size(), 2);

  EXPECT_TRUE(q.Dequeue(&front));
  EXPECT_EQ(front, 1);
  EXPECT_EQ(q.Size(), 1);

  EXPECT_TRUE(q.Dequeue(&front));
  EXPECT_EQ(front, 2);
  EXPECT_TRUE(q.Empty());
  EXPECT_EQ(q.Size(), 0);

  EXPECT_FALSE(q.Dequeue(&front));

  q.Enqueue(3);
  EXPECT_FALSE(q.Empty());
  EXPECT_EQ(q.Size(), 1);

  q.Clear();

  EXPECT_FALSE(q.Dequeue(&front));
  EXPECT_TRUE(q.Empty());
  EXPECT_EQ(q.Size(), 0);

  q.Enqueue(4);
  EXPECT_FALSE(q.Empty());
  EXPECT_EQ(q.Size(), 1);
  EXPECT_TRUE(q.Dequeue(&front));
  EXPECT_EQ(front, 4);
  EXPECT_TRUE(q.Empty());
  EXPECT_EQ(q.Size(), 0);
}

}  // namespace base
}  // namespace cyber
}  // namespace apollo
