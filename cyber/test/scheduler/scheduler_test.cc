#include "cyber/scheduler/scheduler.h"
#include "cyber/croutine/routine_factory.h"
#include "cyber/cyber.h"
#include "gtest/gtest.h"

namespace apollo {
namespace cyber {
namespace scheduler {

auto sched = Scheduler::Instance();

void proc() {}

TEST(SchedulerTest, create_task) {
  std::string croutine_name = "DriverProc";
  EXPECT_TRUE(sched->CreateTask(&proc, croutine_name));
}

TEST(SchedulerTest, remove_task) {
  Scheduler scheduler;
  EXPECT_TRUE(scheduler.RemoveTask("remove_task"));
  scheduler.ShutDown();
  EXPECT_TRUE(scheduler.RemoveTask("remove_task"));
}

}  // namespace scheduler
}  // namespace cyber
}  // namespace apollo

int main(int argc, char** argv) {
  apollo::cyber::Init(argv[0]);
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
