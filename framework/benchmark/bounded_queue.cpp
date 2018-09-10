#include "cybertron/base/bounded_queue.h"
#include "cybertron/base/thread_safe_queue.h"
#include <atomic>
#include <mutex>
#include <queue>
#include <string>
#include "cybertron/time/time.h"

using apollo::cybertron::base::BoundedQueue;
using apollo::cybertron::base::ThreadSafeQueue;

volatile bool ready = false;
int main(int argc, char const *argv[]) {

  if (argc != 4) {
    std::cout << "Usage:" << std::endl;
    std::cout << "     argv[0] program name" << std::endl;
    std::cout << "     argv[1] consumer num" << std::endl;
    std::cout << "     argv[2] producer num" << std::endl;
    std::cout << "     argv[3] proc times" << std::endl;
    return 0;
  }
  BoundedQueue<std::shared_ptr<int>> queue;
  queue.Init(2000000);
  ThreadSafeQueue<std::shared_ptr<int>> std_queue;

  auto start = apollo::cybertron::Time::MonoTime();
  for (int i = 0; i < 1000000; i++) {
    queue.Enqueue(std::make_shared<int>(i));
  }
  auto end = apollo::cybertron::Time::MonoTime();
  std::cout << "bounded queue Enqueue 1000000 integer: " << end - start
            << std::endl;

  start = apollo::cybertron::Time::MonoTime();
  for (int i = 0; i < 1000000; i++) {
    std_queue.Enqueue(std::make_shared<int>(i));
  }
  end = apollo::cybertron::Time::MonoTime();
  std::cout << "standard queue Enqueue 1000000 integer: " << end - start
            << std::endl;

  int consumer_num = std::stoi(argv[1]);
  int producer_num = std::stoi(argv[2]);
  int proc_times = std::stoi(argv[3]);
  std::atomic<int> count = {0};
  // auto  value = std::make_shared<int>();
  std::thread consumers[consumer_num];
  std::thread producers[producer_num];

  for (int i = 0; i < consumer_num; i++) {
    consumers[i] = std::thread([&]() {
      while (!ready) {
        asm volatile("rep; nop" ::: "memory");
      }
      auto  value = std::make_shared<int>();
      for (int j = 0; j < proc_times; j++) {
        queue.Dequeue(&value);
      }
      count++;
    });
  }

  for (int i = 0; i < producer_num; i++) {
    producers[i] = std::thread([&]() {
      while (!ready) {
        asm volatile("rep; nop" ::: "memory");
      }
      for (int j = 0; j < proc_times; j++) {
        queue.Enqueue(std::make_shared<int>(j));
      }
      count++;
    });
  }
  sleep(1);
  ready = true;
  start = apollo::cybertron::Time::MonoTime();
  while (count < consumer_num + producer_num) {
    continue;
  }
  end = apollo::cybertron::Time::MonoTime();
  std::cout << "bounded queue : \t" << consumer_num << " consumers and " << producer_num
            << " producers each process " << proc_times
            << " integer: " << end - start << std::endl;
  for (int i = 0; i < consumer_num; i++) {
    consumers[i].join();
  }

  for (int i = 0; i < producer_num; i++) {
    producers[i].join();
  }

  ready = false;
  count.store(0);
  std::mutex mutex;
  for (int i = 0; i < consumer_num; i++) {
    consumers[i] = std::thread([&]() {
      while (!ready) {
        asm volatile("rep; nop" ::: "memory");
      }
      auto  value = std::make_shared<int>();
      for (int j = 0; j < proc_times; j++) {
        std_queue.Dequeue(&value);
      }
      count++;
    });
  }

  for (int i = 0; i < producer_num; i++) {
    producers[i] = std::thread([&]() {
      while (!ready) {
        asm volatile("rep; nop" ::: "memory");
      }
      for (int j = 0; j < proc_times; j++) {
        std_queue.Enqueue(std::make_shared<int>(j));
      }
      count++;
    });
  }
  sleep(1);
  ready = true;
  start = apollo::cybertron::Time::MonoTime();
  while (count < consumer_num + producer_num) {
    continue;
  }
  end = apollo::cybertron::Time::MonoTime();
  std::cout << "standard queue : \t" << consumer_num << " consumers and " << producer_num
            << " producers each process " << proc_times
            << " integer: " << end - start << std::endl;
  for (int i = 0; i < consumer_num; i++) {
    consumers[i].join();
  }

  for (int i = 0; i < producer_num; i++) {
    producers[i].join();
  }

  return 0;
}
