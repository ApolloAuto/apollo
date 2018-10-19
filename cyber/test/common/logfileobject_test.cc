#include "cyber/cyber.h"
#include "cyber/logger/log_file_object.h"
#include "glog/logging.h"
#include "gtest/gtest.h"
#include "time.h"

namespace apollo {
namespace cyber {
namespace logger {

TEST(LogFileObjectTest, init_and_write) {
  std::string basename = "logfile";
  LogFileObject logfileobject(google::INFO, basename.c_str());
  logfileobject.SetBasename("base");
  time_t timep;
  time(&timep);
  std::string message = "cyber logger test";
  logfileobject.Write(false, timep, message.c_str(), 20);
  logfileobject.SetExtension("unittest");
  logfileobject.Flush();
}

}  // namespace logger
}  // namespace cyber
}  // namespace apollo

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
