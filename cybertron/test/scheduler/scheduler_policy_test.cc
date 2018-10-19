#include "gtest/gtest.h"

#include "cybertron/common/util.h"
#include "cybertron/cybertron.h"
#include "cybertron/scheduler/policy/task_choreo.h"
#include "cybertron/scheduler/processor.h"

namespace apollo {
namespace cybertron {
namespace scheduler {

void func() {}
TEST(SchedulerPolicyTest, choreo) {
  auto processor = std::make_shared<Processor>();
  std::shared_ptr<ProcessorContext> ctx;
  ctx.reset(new TaskChoreoContext(processor));

  std::shared_ptr<CRoutine> cr = std::make_shared<CRoutine>(func);
  EXPECT_TRUE(ctx->RqEmpty());
  cr->SetId(common::Hash("choreo"));
  ctx->Push(cr);
  // repeat Push the same CRoutine
  EXPECT_TRUE(ctx->NextRoutine() != nullptr);
  ctx->ShutDown();
  EXPECT_TRUE(ctx->NextRoutine() == nullptr);
  ctx->PrintStatistics();
}

}  // namespace scheduler
}  // namespace cybertron
}  // namespace apollo

int main(int argc, char** argv) {
  apollo::cybertron::Init(argv[0]);
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
