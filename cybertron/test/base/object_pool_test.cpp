#include "cybertron/base/object_pool.h"
#include <thread>
#include "gtest/gtest.h"

namespace apollo {
namespace cybertron {
namespace base {

class TestNode {
 public:
  TestNode() {}
  explicit TestNode(int data) : value(data) {}
  int value = 0;
};

TEST(ObjectPoolTest, get_object) {
  auto pool = ObjectPool<TestNode>::Instance(10, 100);
  for (int i = 0; i < 10; i++) {
    EXPECT_EQ(100, pool->GetObject()->value);
  }
  EXPECT_NE(nullptr, pool->GetObject());

  auto pool2 = ObjectPool<TestNode>::Instance(10);
  for (int i = 0; i < 10; i++) {
    EXPECT_EQ(0, pool2->GetObject()->value);
  }
  EXPECT_NE(nullptr, pool2->GetObject());
}

}  // namespace base
}  // namespace cybertron
}  // namespace apollo

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
