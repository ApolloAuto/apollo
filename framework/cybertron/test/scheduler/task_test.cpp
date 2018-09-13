#include "cybertron/scheduler/task.h"
#include "cybertron/common/log.h"
#include "cybertron/component/component.h"
#include "cybertron/component/timer_component.h"
#include "cybertron/cybertron.h"
#include "cybertron/message/raw_message.h"
#include "cybertron/proto/driver.pb.h"
#include "gtest/gtest.h"
using apollo::cybertron::proto::CarStatus;

namespace apollo {
namespace cybertron {
namespace scheduler {
void VoidTask() { AINFO << "VoidTask running"; }
void UserTask(const std::shared_ptr<CarStatus>& msg) { AINFO << "receive msg"; }
TEST(TaskTest, all) {
  std::shared_ptr<apollo::cybertron::Task<CarStatus>> task_ = nullptr;
  std::shared_ptr<apollo::cybertron::Task<void>> void_task_ = nullptr;
  task_.reset(new apollo::cybertron::Task<CarStatus>("task", &UserTask));
  void_task_.reset(new apollo::cybertron::Task<void>("void_task", &VoidTask));
}

}  // namespace scheduler
}  // namespace cybertron
}  // namespace apollo

int main(int argc, char** argv) {
  apollo::cybertron::Init(argv[0]);
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}