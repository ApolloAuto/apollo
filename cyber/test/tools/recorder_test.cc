#include <sys/epoll.h>
#include <unistd.h>

#include "gtest/gtest.h"

namespace apollo {
namespace cyber {
namespace transport {}  // namespace transport
}  // namespace cyber
}  // namespace apollo

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
