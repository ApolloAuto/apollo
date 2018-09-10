#include <sys/epoll.h>
#include <unistd.h>

#include "gtest/gtest.h"

namespace apollo {
namespace cybertron {
namespace transport {

}  // namespace transport
}  // namespace cybertron
}  // namespace apollo

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
