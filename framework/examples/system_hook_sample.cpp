#include <chrono>
#include <iostream>
#include <thread>

#include <unistd.h>

#include "cybertron/croutine/croutine.h"
#include "cybertron/scheduler/scheduler.h"

/*
class MyRoutine1 : public apollo::cybertron::CRoutine {
 public:
  MyRoutine1(const int& num) {
    num_ = num + 1;
  }
  void Routine() override;

 private:
  int num_ = 0;
};

class MyRoutine2 : public apollo::cybertron::CRoutine {
 public:
  MyRoutine2(const int& num) {
    num_ = num + 1;
  }
  void Routine() override;

 private:
  int num_ = 0;
};

void MyRoutine1::Routine() {
    std::cout << std::this_thread::get_id() << ": bolck routine " << num_ << "
switch to sleep" << std::endl;
    usleep(num_ * 1000000);
    std::cout << std::this_thread::get_id() << ": bolck routine " << num_ << "
finish" << std::endl;
}

void MyRoutine2::Routine() {
    std::this_thread::sleep_for(std::chrono::milliseconds(num_ * 10));
    std::cout << std::this_thread::get_id() << ": non-block routine " << num_ <<
" finish" << std::endl;
}
*/

void MyRoutine1(const int& num_) {
  std::cout << std::this_thread::get_id() << ": bolck routine " << num_
            << " switch to sleep" << std::endl;
  usleep(num_ * 1000000);
  std::cout << std::this_thread::get_id() << ": bolck routine " << num_
            << " finish" << std::endl;
}

void MyRoutine2(const int& num_) {
  std::this_thread::sleep_for(std::chrono::milliseconds(num_ * 10));
  std::cout << std::this_thread::get_id() << ": non-block routine " << num_
            << " finish" << std::endl;
}

using apollo::cybertron::croutine::CRoutine;

int main() {
  usleep(1000000);
  auto sched = apollo::cybertron::scheduler::Scheduler::Instance();
  std::this_thread::sleep_for(std::chrono::milliseconds(100));
  for (int i = 0; i < 10; ++i) {
    std::string croutine_name = "MyRoutine1_" + std::to_string(i);
    auto func = [i]() { MyRoutine1(i); };
    sched->CreateTask(func, croutine_name);
    croutine_name = "MyRoutine2_" + std::to_string(i);
    auto func1 = [i]() { MyRoutine2(i); };
    sched->CreateTask(func1, croutine_name);
  }

  std::this_thread::sleep_for(std::chrono::seconds(100));
  return 0;
}
