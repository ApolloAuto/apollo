#include <chrono>

#include "cybertron/croutine/croutine.h"
#include "cybertron/croutine/routine_context.h"
#include "cybertron/cybertron.h"
#include "gtest/gtest.h"

namespace apollo {
namespace cybertron {
namespace croutine {

void Sleep(uint64_t ms) {
  std::this_thread::sleep_for(std::chrono::milliseconds(ms));
}

TEST(CRoutineTest, croutine_basic) {
  bool finish;
  auto cr = std::make_shared<CRoutine>([&finish]() { finish = true; });
  auto state = cr->Resume();
  EXPECT_TRUE(finish);
  EXPECT_EQ(state, RoutineState::FINISHED);
  cr->PrintStatistics();
}

TEST(CRoutineTest, croutine_switch) {
  int count = 0;
  auto cr1 = std::make_shared<CRoutine>([&count]() {
    count += 1;
    CRoutine::Yield();
    EXPECT_EQ(count, 2);
  });
  auto cr2 = std::make_shared<CRoutine>([&count]() {
    EXPECT_EQ(count, 1);
    count += 1;
  });
  auto state = cr1->Resume();
  EXPECT_EQ(state, RoutineState::READY);
  state = cr2->Resume();
  EXPECT_EQ(state, RoutineState::FINISHED);
  state = cr1->Resume();
  EXPECT_EQ(state, RoutineState::FINISHED);
}

TEST(CRoutineTest, croutine_sleep) {
  auto cr = std::make_shared<CRoutine>(
      []() { croutine::CRoutine::GetCurrentRoutine()->Sleep(1000); });
  auto state = cr->Resume();
  EXPECT_EQ(state, RoutineState::SLEEP);
  Sleep(5);
  state = cr->Resume();
  EXPECT_EQ(state, RoutineState::FINISHED);
}

TEST(CRoutineTest, croutine_hangup) {
  auto cr = std::make_shared<CRoutine>(
      []() { CRoutine::GetCurrentRoutine()->HangUp(); });
  auto state = cr->Resume();
  EXPECT_EQ(state, RoutineState::IO_WAIT);
  cr->Wake();
  state = cr->Resume();
  EXPECT_EQ(state, RoutineState::FINISHED);
}

}  // namespace croutine
}  // namespace cybertron
}  // namespace apollo

int main(int argc, char** argv) {
  apollo::cybertron::Init(argv[0]);
  auto context =
      std::make_shared<apollo::cybertron::croutine::RoutineContext>();
  apollo::cybertron::croutine::CRoutine::SetMainContext(context);
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
