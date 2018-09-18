#include "cybertron/scheduler/task.h"
#include "cybertron/common/log.h"
#include "cybertron/component/component.h"
#include "cybertron/component/timer_component.h"
#include "cybertron/cybertron.h"
#include "cybertron/message/raw_message.h"
#include "cybertron/proto/driver.pb.h"
#include "gtest/gtest.h"

using apollo::cybertron::proto::CarStatus;
using apollo::cybertron::Task;

namespace apollo {
namespace cybertron {
namespace scheduler {

struct Message {
  uint64_t id;
};

void Task1() { ADEBUG << "Task1 running"; }

void Task2(const std::shared_ptr<Message>& input) { ADEBUG << "Task2 running"; }

uint64_t Task3(const std::shared_ptr<Message>& input) {
  ADEBUG << "Task3 running";
  return input->id;
}

uint64_t Task4(const std::shared_ptr<Message>& input) {
  ADEBUG << "Task4 running";
  usleep(10000);
  return input->id;
}

/*
Message UserTask(const std::shared_ptr<CarStatus>& msg) {
  AINFO << "receive msg";
  return 0;
}
*/

TEST(TaskTest, create_task) {
  auto task_1 = std::make_shared<Task<>>("task1", &Task1);
  task_1.reset();
  auto task_2 = std::make_shared<Task<void>>("task2", &Task1);
  task_2.reset();
  auto task_3 = std::make_shared<Task<void, void>>("task3", &Task1);
  task_3.reset();
}

TEST(TaskTest, return_value) {
  auto msg = std::make_shared<Message>();
  msg->id = 1;

  auto task_1 = std::make_shared<Task<>>("task1", &Task1);
  usleep(100000);
  task_1.reset();

  auto task_2 = std::make_shared<Task<Message, void>>("task2", &Task2);
  auto ret_2 = task_2->Execute(msg);
  ret_2.get();

  auto msg3 = std::make_shared<Message>();
  msg3->id = 1;
  auto task_3 = std::make_shared<Task<Message, int>>("task3", &Task3);
  auto ret_3 = task_3->Execute(msg3);
  EXPECT_EQ(ret_3.get(), 1);
  ret_3 = task_3->Execute(msg3);
  usleep(100000);
  EXPECT_EQ(ret_3.get(), 1);

  auto task_4 = std::make_shared<Task<Message, uint64_t>>("task4", &Task4, 20);
  std::vector<std::future<uint64_t>> results;
  for (int i = 0; i < 1000; i++) {
    results.emplace_back(task_4->Execute(msg));
  }
  for (auto& result : results) {
    result.get();
  }
}

}  // namespace scheduler
}  // namespace cybertron
}  // namespace apollo

int main(int argc, char** argv) {
  apollo::cybertron::Init(argv[0]);
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
