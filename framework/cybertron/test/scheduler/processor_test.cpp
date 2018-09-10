#include "gtest/gtest.h"

#include "cybertron/cybertron.h"
#include "cybertron/scheduler/policy/rq_context.h"
#include "cybertron/scheduler/processor.h"

namespace apollo {
namespace cybertron {
namespace scheduler {

TEST(ProcessorTest, init_and_run) {
  std::shared_ptr<Processor> processor = std::make_shared<Processor>(false);
  std::shared_ptr<ProcessorContext> ctx;
  ctx.reset(new RQContext(processor));
  processor->BindContext(ctx);
  processor->Start();
  processor->Notify();
  processor->Stop();
}

}  // namespace scheduler
}  // namespace cybertron
}  // namespace apollo

int main(int argc, char** argv) {
  apollo::cybertron::Init(argv[0]);
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
