#include "cyber/common/log.h"
#include "glog/logging.h"
#include "gtest/gtest.h"

namespace apollo {
namespace cyber {
namespace common {

TEST(LogTest, TestAll) { AINFO << "11111"; }

}  // namespace logger
}  // namespace cyber
}  // namespace apollo

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  google::InitGoogleLogging(argv[0]);
  FLAGS_alsologtostderr = true;
  return RUN_ALL_TESTS();
}
