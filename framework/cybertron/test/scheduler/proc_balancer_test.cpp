#include "gtest/gtest.h"

#include "cybertron/croutine/croutine.h"
#include "cybertron/cybertron.h"
#include "cybertron/scheduler/proc_balancer.h"

namespace apollo {
namespace cybertron {
namespace scheduler {

using apollo::cybertron::croutine::CRoutine;

void Proc() {}

TEST(ProcBalancerTest, run) {
  auto result = ProcBalancer::Instance()->Run();
  EXPECT_FALSE(result);
}

TEST(ProcBalancerTest, Push_routine) {
  auto rt = std::make_shared<CRoutine>(Proc);
  rt->SetId(123);
  EXPECT_FALSE(ProcBalancer::Instance()->Push(rt));
}

TEST(ProcBalancerTest, get_proc) {
  auto rt = std::make_shared<CRoutine>(Proc);
  uint32_t processor_id = 0;
  auto result = ProcBalancer::Instance()->GetProcessor(rt->Id(), &processor_id);
  EXPECT_FALSE(result);
}

TEST(ProcBalancerTest, get_proper_proc) {
  auto rt = std::make_shared<CRoutine>(Proc);
  rt->SetProcessorId(0);
  auto proc = ProcBalancer::Instance()->GetProperProcessor(rt);
  EXPECT_EQ(0, proc->Id());
}

TEST(ProcBalancerTest, print_statistics) {
  ProcBalancer::Instance()->PrintStatistics();
}
/*
void proc() {
}
TEST(ProcBalancerTest, Push_id_routine) {
  std::shared_ptr<Routine> rt = std::make_shared<Routine>();

  CRoutine cr(proc);
  std::shared_ptr<CRoutine> crp = std::make_shared<CRoutine>(cr);
  rt->SetCRoutine(crp);
  EXPECT_FALSE(ProcBalancer::Instance()->Push(0, rt));
}
*/
}  // namespace scheduler
}  // namespace cybertron
}  // namespace apollo

int main(int argc, char** argv) {
  apollo::cybertron::Init(argv[0]);
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
