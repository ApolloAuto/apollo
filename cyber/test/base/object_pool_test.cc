#include "cyber/base/object_pool.h"
#include <thread>
#include "gtest/gtest.h"

namespace apollo {
namespace cyber {
namespace base {

class TestNode {
 public:
  TestNode() {}
  explicit TestNode(int data) : value(data) {}
  int value = 0;
};

TEST(ObjectPoolTest, get_object) {
  auto pool = new ObjectPool<TestNode>(10, 100);
  for (int i = 0; i < 10; i++) {
    EXPECT_EQ(100, pool->GetObject()->value);
  }
  EXPECT_NE(nullptr, pool->GetObject());
  delete pool;

  auto pool2 = new ObjectPool<TestNode>(10);
  for (int i = 0; i < 10; i++) {
    EXPECT_EQ(0, pool2->GetObject()->value);
  }
  EXPECT_NE(nullptr, pool2->GetObject());
  delete pool2;
}

}  // namespace base
}  // namespace cyber
}  // namespace apollo

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
