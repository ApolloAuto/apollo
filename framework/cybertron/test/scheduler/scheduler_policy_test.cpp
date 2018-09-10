#include "gtest/gtest.h"

#include "cybertron/common/util.h"
#include "cybertron/cybertron.h"
#include "cybertron/scheduler/policy/cfs_context.h"
#include "cybertron/scheduler/policy/fcfs_context.h"
#include "cybertron/scheduler/policy/rq_context.h"
#include "cybertron/scheduler/processor.h"
namespace apollo {
namespace cybertron {
namespace scheduler {

void func() {}
TEST(SchedulerPolicyTest, rq) {
  auto processor = std::make_shared<Processor>();
  std::shared_ptr<ProcessorContext> ctx;
  ctx.reset(new RQContext(processor));

  std::shared_ptr<CRoutine> cr = std::make_shared<CRoutine>(func);
  cr->SetId(common::Hash("rq"));
  ctx->Push(cr);
  // repeat Push the same CRoutine
  EXPECT_TRUE(ctx->RqEmpty());
  EXPECT_TRUE(ctx->NextRoutine() != nullptr);
  std::future<std::shared_ptr<CRoutine>> fut;
  // Pop cr that not exist
  EXPECT_FALSE(ctx->Pop(1234, fut));
  // Pop cr that exist
  EXPECT_TRUE(ctx->Pop(cr->Id(), fut));
}

TEST(SchedulerPolicyTest, fcfs) {
  auto processor = std::make_shared<Processor>();
  std::shared_ptr<ProcessorContext> ctx;
  ctx.reset(new FCFSContext(processor));

  std::shared_ptr<CRoutine> cr = std::make_shared<CRoutine>(func);
  EXPECT_TRUE(ctx->RqEmpty());
  EXPECT_TRUE(ctx->NextRoutine() == nullptr);
  cr->SetId(common::Hash("fcfs"));
  ctx->Push(cr);
  EXPECT_FALSE(ctx->NextRoutine() == nullptr);
}

TEST(SchedulerPolicyTest, cfs) {
  auto processor = std::make_shared<Processor>();
  std::shared_ptr<ProcessorContext> ctx;
  ctx.reset(new CFSContext(processor));

  std::shared_ptr<CRoutine> cr = std::make_shared<CRoutine>(func);
  EXPECT_TRUE(ctx->RqEmpty());
  cr->SetId(common::Hash("cfs"));
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
