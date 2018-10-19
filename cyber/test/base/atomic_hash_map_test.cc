#include "cyber/base/atomic_hash_map.h"
#include <string>
#include <thread>
#include "gtest/gtest.h"

namespace apollo {
namespace cyber {
namespace base {

TEST(AtomicHashMapTest, int_int) {
  AtomicHashMap<int, int> map;
  int value = 0;
  for (int i = 0; i < 1000; i++) {
    map.Set(i, i);
    EXPECT_TRUE(map.Has(i));
    EXPECT_TRUE(map.Get(i, &value));
    EXPECT_EQ(i, value);
    EXPECT_TRUE(map.Remove(i));
    EXPECT_FALSE(map.Has(i));
    EXPECT_FALSE(map.Get(i, &value));
    EXPECT_FALSE(map.Remove(i));
  }

  for (int i = 0; i < 1000; i++) {
    map.Set(1000 - i, i);
    EXPECT_TRUE(map.Has(1000 - i));
    EXPECT_TRUE(map.Get(1000 - i, &value));
    EXPECT_EQ(i, value);
    EXPECT_TRUE(map.Remove(1000 - i));
    EXPECT_FALSE(map.Has(1000 - i));
    EXPECT_FALSE(map.Get(1000 - i, &value));
    EXPECT_FALSE(map.Remove(1000 - i));
  }
}

TEST(AtomicHashMapTest, int_str) {
  AtomicHashMap<int, std::string> map;
  std::string value("");
  for (int i = 0; i < 1000; i++) {
    map.Set(i, std::to_string(i));
    EXPECT_TRUE(map.Has(i));
    EXPECT_TRUE(map.Get(i, &value));
    EXPECT_EQ(std::to_string(i), value);
    EXPECT_TRUE(map.Remove(i));
    EXPECT_FALSE(map.Has(i));
    EXPECT_FALSE(map.Get(i, &value));
    EXPECT_FALSE(map.Remove(i));
  }
}

TEST(AtomicHashMapTest, concurrency) {
  AtomicHashMap<int, int, 1024> map;
  int thread_num = 32;
  std::thread t[thread_num];

  for (int i = 0; i < thread_num; i++) {
    t[i] = std::thread([&, i]() {
      for (int j = 0; j < thread_num * 1000; j++) {
        if (j % thread_num == i) {
          map.Set(j, j);
        }
      }
    });
  }
  for (int i = 0; i < thread_num; i++) {
    t[i].join();
  }

  int value = 0;
  for (int i = 0; i < thread_num * 1000; i++) {
    EXPECT_TRUE(map.Get(i, &value));
    EXPECT_EQ(i, value);
  }
}

}  // namespace base
}  // namespace cyber
}  // namespace apollo

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
