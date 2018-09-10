#include "gtest/gtest.h"
#include "cybertron/scheduler/routine_balancer.h"
#include "cybertron/cybertron.h"

namespace apollo {
namespace cybertron {
namespace scheduler {

TEST(RoutineBalancerTest, run) {
  RoutineBalancer routine_balancer;
  EXPECT_TRUE(routine_balancer.Run());
}
TEST(RoutineBalancerTest, print_statistics) {
  RoutineBalancer routine_balancer;
  routine_balancer.PrintStatistics();
}

void proc() {
}

TEST(RoutineBalancerTest, Push) {
  RoutineBalancer routine_balancer;

  std::shared_ptr<apollo::cybertron::proto::RoutineConf> routine_conf_ =
      nullptr;
  auto global_conf = common::GlobalData::Instance()->Config();
  if (global_conf.has_routine_conf()) {
    routine_conf_ = std::make_shared<proto::RoutineConf>();
    routine_conf_->CopyFrom(global_conf.routine_conf());
  }
  std::shared_ptr<apollo::cybertron::proto::RoutineConfInfo> conf =
      std::make_shared<apollo::cybertron::proto::RoutineConfInfo>();
  if (routine_conf_ != nullptr) {
    for (int i = 0; i < routine_conf_->routine_info_size(); ++i) {
      auto routine_conf_info = routine_conf_->routine_info(i);
      conf->CopyFrom(routine_conf_info);
    }
  }
  auto id = common::GlobalData::Instance()->RegisterTaskName("Push");
  auto rt = std::make_shared<CRoutine>(proc);
  rt->SetId(id);
  EXPECT_TRUE(routine_balancer.Push(rt));
  EXPECT_FALSE(routine_balancer.Push(rt));
  routine_balancer.PrintStatistics();
  EXPECT_TRUE(routine_balancer.Run());
}

}  // namespace scheduler
}  // namespace cybertron
}  // namespace apollo

int main(int argc, char** argv) {
  apollo::cybertron::Init(argv[0]);
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
