#include "cyber/base/bounded_queue.h"
#include <thread>
#include "gtest/gtest.h"

namespace apollo {
namespace cyber {
namespace base {

TEST(BoundedQueueTest, Enqueue) {
  BoundedQueue<int> queue;
  queue.Init(100);
  EXPECT_EQ(0, queue.Size());
  EXPECT_TRUE(queue.Empty());
  for (int i = 1; i <= 100; i++) {
    EXPECT_TRUE(queue.Enqueue(i));
    EXPECT_EQ(i, queue.Size());
  }
  EXPECT_FALSE(queue.Enqueue(101));
}

TEST(BoundedQueueTest, Dequeue) {
  BoundedQueue<int> queue;
  queue.Init(100);
  int value = 0;
  for (int i = 0; i < 100; i++) {
    EXPECT_TRUE(queue.Enqueue(i));
    EXPECT_TRUE(queue.Dequeue(&value));
    EXPECT_EQ(i, value);
  }
  EXPECT_FALSE(queue.Dequeue(&value));
}

TEST(BoundedQueueTest, concurrency) {
  BoundedQueue<int> queue;
  queue.Init(100000);
  std::atomic_int count = {0};
  std::thread threads[48];
  for (int i = 0; i < 48; ++i) {
    if (i % 2 == 0) {
      threads[i] = std::thread([&]() {
        for (int j = 0; j < 10000; ++j) {
          if (queue.Enqueue(j)) {
            count++;
          }
        }
      });
    } else {
      threads[i] = std::thread([&]() {
        for (int j = 0; j < 10000; ++j) {
          int value = 0;
          if (queue.Dequeue(&value)) {
            count--;
          }
        }
      });
    }
  }
  for (int i = 0; i < 48; ++i) {
    threads[i].join();
  }
  EXPECT_EQ(count.load(), queue.Size());
}

TEST(BoundedQueueTest, WaitDequeue) {
  BoundedQueue<int> queue;
  queue.Init(100);
  queue.Enqueue(10);
  std::thread t([&]() {
    int value = 0;
    queue.WaitDequeue(&value);
    EXPECT_EQ(10, value);
    queue.WaitDequeue(&value);
    EXPECT_EQ(100, value);
  });
  queue.Enqueue(100);
  t.join();
}

TEST(BoundedQueueTest, block_wait) {
  BoundedQueue<int> queue(new BlockWaitStrategy());
  queue.Init(100);
  std::thread t([&]() {
    int value = 0;
    queue.WaitDequeue(&value);
    EXPECT_EQ(100, value);
  });
  queue.Enqueue(100);
  t.join();
}

TEST(BoundedQueueTest, yield_wait) {
  BoundedQueue<int> queue(new YieldWaitStrategy());
  queue.Init(100);
  std::thread t([&]() {
    int value = 0;
    queue.WaitDequeue(&value);
    EXPECT_EQ(100, value);
  });
  queue.Enqueue(100);
  t.join();
}

TEST(BoundedQueueTest, spin_wait) {
  BoundedQueue<int> queue(new BusySpinWaitStrategy());
  queue.Init(100);
  std::thread t([&]() {
    int value = 0;
    queue.WaitDequeue(&value);
    EXPECT_EQ(100, value);
  });
  queue.Enqueue(100);
  t.join();
}

TEST(BoundedQueueTest, busy_wait) {
  BoundedQueue<int> queue(new BusySpinWaitStrategy());
  queue.Init(100);
  std::thread t([&]() {
    int value = 0;
    queue.WaitDequeue(&value);
    EXPECT_EQ(100, value);
  });
  queue.Enqueue(100);
  t.join();
}

}  // namespace base
}  // namespace cyber
}  // namespace apollo

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
