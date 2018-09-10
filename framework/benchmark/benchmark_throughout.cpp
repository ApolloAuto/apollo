#include <chrono>
#include <functional>
#include <iostream>
#include <memory>
#include <mutex>
#include <thread>

#include <unistd.h>

//#include "scheduler/processor.h"
#include "cybertron/scheduler/scheduler.h"
//#include "scheduler/croutine.h"
#include "cybertron/croutine/routine_factory.h"
#include "cybertron/data/data_visitor.h"

using apollo::cybertron::croutine::CRoutine;
using apollo::cybertron::scheduler::Scheduler;

std::mutex data_mutex;

int enqueue_num = 0;
int proc_num = 0;

bool stop = false;

uint64_t mono_time() {
  timespec now;
  clock_gettime(CLOCK_MONOTONIC, &now);
  return static_cast<uint64_t>(now.tv_sec) * 1000000000L +
         static_cast<uint64_t>(now.tv_nsec);
}

void nsec_spin(uint64_t spin_time) {
  if (spin_time == 0) {
    return;
  }

  timespec now;
  timespec start;
  uint64_t start_ns;
  clock_gettime(CLOCK_MONOTONIC, &start);
  start_ns = static_cast<uint64_t>(start.tv_sec) * 1000000000L +
             static_cast<uint64_t>(start.tv_nsec);

  while (1) {
    clock_gettime(CLOCK_MONOTONIC, &now);
    uint64_t now_ns = static_cast<uint64_t>(now.tv_sec) * 1000000000L +
                      static_cast<uint64_t>(now.tv_nsec);
    if ((now_ns - start_ns) >= spin_time) {
      break;
    }
  }
  return;
}

void Proc(uint64_t proc_time) {
  // int type = std::rand() % 3;
  nsec_spin(proc_time);
  {
    std::lock_guard<std::mutex> lg(data_mutex);
    proc_num += 1;
  }
}

void Run(uint64_t proc_time_ms, int is_random, int index) {
  if (stop) {
    return;
  }
  auto sched = Scheduler::Instance();
  std::string croutine_name = "DriverProc" + std::to_string(index);
  // simulate 100ms poll event
  auto func = [=]() {
    uint64_t _proc_time_ms;
    if (is_random) {
      _proc_time_ms = std::rand() % proc_time_ms;
    } else {
      _proc_time_ms = proc_time_ms;
    }
    Proc(_proc_time_ms * 1000 * 1000);
  };
  if (sched->CreateTask(func, croutine_name)) {
    std::lock_guard<std::mutex> lg(data_mutex);
    enqueue_num += 1;
  }
}

int main(int argc, char* argv[]) {
  if (argc < 5) {
    return -1;
  }
  int concurrent_num = atoi(argv[1]);
  int test_time_sec = atoi(argv[2]);
  int proc_time_s = atoi(argv[3]);
  int is_random = atoi(argv[4]);
  // enqueue tasks
  std::thread _thread([=]() {
    for (int i = 1; i <= concurrent_num; ++i) {
      Run(static_cast<uint64_t>(proc_time_s), is_random, i);
    }
  });
  _thread.detach();
  sleep(test_time_sec);
  stop = true;
  std::cout << "enqueue_num[" << enqueue_num << "] proc_num[" << proc_num
            << "] test_time(s)[" << test_time_sec << "]" << std::endl;
  return 0;
}
