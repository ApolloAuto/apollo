#include "gtest/gtest.h"
#include "glog/logging.h"
#include "time.h"
#include "cybertron/logger/logger.h"

namespace apollo {
namespace cybertron {
namespace logger {

TEST(LoggerTest, init_and_write) {
  Logger logger(google::base::GetLogger(google::INFO));
  time_t timep;
  time(&timep);
  std::string message = "cybertron logger test";
  logger.Write(false, timep, message.c_str(), 20);
  message = "**[CybertronLoggerTest]** cybertron logger test";
  logger.Write(false, timep, message.c_str(), 20);
  logger.LogSize();
  logger.Flush();
}

}  // namespace logger
}  // namespace cybertron
}  // namespace apollo

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
