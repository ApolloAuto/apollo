#include <chrono>

#include "cyber/croutine/routine_factory.h"
#include "cyber/cyber.h"
#include "cyber/proto/chatter.pb.h"
#include "gtest/gtest.h"

namespace apollo {
namespace cyber {
namespace croutine {

std::hash<std::string> str_hash;

TEST(RoutineFactoryTest, routine_factory) {
  bool finish = false;
  auto msg = std::make_shared<proto::Chatter>();
  msg->set_timestamp(12345);

  auto dv = std::make_shared<data::DataVisitor<proto::Chatter>>(
      str_hash("channel1"), uint32_t(1));
  auto f1 = [&](const std::shared_ptr<proto::Chatter>& message) {
    EXPECT_EQ(msg->timestamp(), message->timestamp());
    finish = true;
  };
  RoutineFactory factory = CreateRoutineFactory<proto::Chatter>(f1, dv);

  auto cr = std::make_shared<CRoutine>(factory.create_routine());
  cr->Resume();
  EXPECT_FALSE(finish);
  data::DataDispatcher<proto::Chatter>::Instance()->Dispatch(
      str_hash("channel1"), msg);
  std::this_thread::sleep_for(std::chrono::milliseconds(10));
  cr->Wake();
  cr->Resume();
  EXPECT_TRUE(finish);
}

}  // namespace croutine
}  // namespace cyber
}  // namespace apollo
