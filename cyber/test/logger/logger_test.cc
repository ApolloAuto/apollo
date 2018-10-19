#include "cyber/logger/logger.h"
#include "glog/logging.h"
#include "gtest/gtest.h"
#include "time.h"

namespace apollo {
namespace cyber {
namespace logger {

TEST(LoggerTest, init_and_write) {
  Logger logger(google::base::GetLogger(google::INFO));
  time_t timep;
  time(&timep);
  std::string message = "cyber logger test";
  logger.Write(false, timep, message.c_str(), 20);
  message = "**[CyberLoggerTest]** cyber logger test";
  logger.Write(false, timep, message.c_str(), 20);
  logger.LogSize();
  logger.Flush();
}

}  // namespace logger
}  // namespace cyber
}  // namespace apollo

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
