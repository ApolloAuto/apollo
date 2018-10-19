#include <chrono>

#include "cyber/croutine/croutine.h"
#include "cyber/croutine/routine_context.h"
#include "cyber/cyber.h"
#include "gtest/gtest.h"

namespace apollo {
namespace cyber {
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
  auto cr = std::make_shared<CRoutine>([]() {
    croutine::CRoutine::GetCurrentRoutine()->Sleep(croutine::Duration(1000));
  });
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
  EXPECT_EQ(state, RoutineState::DATA_WAIT);
  cr->Wake();
  state = cr->Resume();
  EXPECT_EQ(state, RoutineState::FINISHED);
}

TEST(CRoutineTest, croutine_lock) {
  auto cr = std::make_shared<CRoutine>(
      []() { CRoutine::GetCurrentRoutine()->HangUp(); });

  EXPECT_TRUE(bool(cr->GetLock()));
  EXPECT_TRUE(bool(cr->TryLock()));
  EXPECT_FALSE(bool(cr->DeferLock()));

  auto lock = cr->GetLock();
  EXPECT_TRUE(lock.owns_lock());
  EXPECT_FALSE(bool(cr->TryLock()));
  EXPECT_FALSE(bool(cr->DeferLock()));
  lock.unlock();

  auto lock2 = cr->DeferLock();
  EXPECT_TRUE(lock2.try_lock());
  EXPECT_FALSE(cr->TryLock());
  auto lock3 = cr->DeferLock();
  EXPECT_FALSE(lock3.try_lock());
  lock2.unlock();

  EXPECT_TRUE(lock3.try_lock());
  EXPECT_TRUE(bool(lock3));
  lock3.unlock();
}

}  // namespace croutine
}  // namespace cyber
}  // namespace apollo

int main(int argc, char** argv) {
  apollo::cyber::Init(argv[0]);
  auto context =
      std::make_shared<apollo::cyber::croutine::RoutineContext>();
  apollo::cyber::croutine::CRoutine::SetMainContext(context);
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
