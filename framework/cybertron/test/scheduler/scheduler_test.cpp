#include "gtest/gtest.h"
#include "cybertron/croutine/routine_factory.h"
#include "cybertron/scheduler/scheduler.h"
#include "cybertron/cybertron.h"

namespace apollo {
namespace cybertron {
namespace scheduler {

auto sched = Scheduler::Instance();

TEST(SchedulerTest, remove_task) {
  Scheduler scheduler;
  EXPECT_FALSE(scheduler.RemoveTask("remove_task"));
  scheduler.ShutDown();
  EXPECT_TRUE(scheduler.RemoveTask("remove_task"));
}

TEST(SchedulerTest, print_statistics) {
  Scheduler scheduler;
  scheduler.PrintStatistics();
}

void proc() {
}
TEST(SchedulerTest, create_task) {
  std::string croutine_name = "DriverProc";
  EXPECT_TRUE(sched->CreateTask(&proc, croutine_name));
}

}  // namespace scheduler
}  // namespace cybertron
}  // namespace apollo

int main(int argc, char** argv) {
  apollo::cybertron::Init(argv[0]);
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
