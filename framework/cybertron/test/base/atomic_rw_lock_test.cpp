#include "cybertron/base/atomic_rw_lock.h"
#include <thread>
#include "gtest/gtest.h"

namespace apollo {
namespace cybertron {
namespace base {

TEST(AtomicRWLockTest, read_lock) {
  int count = 0;
  int thread_init = 0;
  bool flag = true;
  AtomicRWLock lock;
  EXPECT_EQ(0, lock.lock_num_.load());
  auto f = [&]() {
    ReadLockGuard lg(lock);
    count++;
    thread_init++;
    while (flag) {
      std::this_thread::yield();
    }
  };
  std::thread t1(f);
  std::thread t2(f);
  while (thread_init != 2) {
    std::this_thread::yield();
  }
  EXPECT_EQ(2, count);
  EXPECT_EQ(2, lock.lock_num_.load());
  flag = false;
  t1.join();
  t2.join();
  {
    ReadLockGuard lg1(lock);
    EXPECT_EQ(1, lock.lock_num_.load());
    {
      ReadLockGuard lg2(lock);
      EXPECT_EQ(2, lock.lock_num_.load());
      {
        ReadLockGuard lg3(lock);
        EXPECT_EQ(3, lock.lock_num_.load());
        {
          ReadLockGuard lg4(lock);
          EXPECT_EQ(4, lock.lock_num_.load());
        }
        EXPECT_EQ(3, lock.lock_num_.load());
      }
      EXPECT_EQ(2, lock.lock_num_.load());
    }
    EXPECT_EQ(1, lock.lock_num_.load());
  }
  EXPECT_EQ(0, lock.lock_num_.load());
}

TEST(AtomicRWLockTest, write_lock) {
  int count = 0;
  int thread_run = 0;
  bool flag = true;
  AtomicRWLock lock(false);
  auto f = [&]() {
    thread_run++;
    WriteLockGuard lg(lock);
    count++;
    while (flag) {
      std::this_thread::yield();
    }
  };
  std::thread t1(f);
  std::thread t2(f);
  while (thread_run != 2) {
    std::this_thread::yield();
  }
  EXPECT_EQ(-1, lock.lock_num_.load());
  EXPECT_EQ(1, count);
  flag = false;
  t1.join();
  t2.join();

  {
    WriteLockGuard lg1(lock);
    EXPECT_EQ(-1, lock.lock_num_.load());
    {
      WriteLockGuard lg2(lock);
      EXPECT_EQ(-2, lock.lock_num_.load());
      {
        ReadLockGuard lg3(lock);
        EXPECT_EQ(-2, lock.lock_num_.load());
      }
    }
    EXPECT_EQ(-1, lock.lock_num_.load());
  }
  EXPECT_EQ(0, lock.lock_num_.load());
}

}  // namespace base
}  // namespace cybertron
}  // namespace apollo

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
