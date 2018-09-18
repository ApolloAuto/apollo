#include "cybertron/common/log.h"
#include "glog/logging.h"
#include "gtest/gtest.h"

namespace apollo {
namespace cybertron {
namespace common {

TEST(LogTest, TestAll) {
  AINFO << "11111";
}

}  // namespace logger
}  // namespace cybertron
}  // namespace apollo

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  google::InitGoogleLogging(argv[0]);
  FLAGS_alsologtostderr = true;
  return RUN_ALL_TESTS();
}
