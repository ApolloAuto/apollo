#include "cybertron/base/atomic_hash_map.h"
#include <atomic>
#include <mutex>
#include <string>
#include <thread>
#include <memory>
#include <unordered_map>
#include "cybertron/base/macros.h"
#include "cybertron/time/time.h"

using apollo::cybertron::base::AtomicHashMap;

volatile bool ready = false;
int main(int argc, char const *argv[]) {
  if (argc != 2) {
    std::cout << "Usage:" << std::endl;
    std::cout << "     argv[0] program name" << std::endl;
    std::cout << "     argv[1] thread num" << std::endl;
    return 0;
  }
  auto thread_num = std::stoi(argv[1]);
  std::thread t1[thread_num];
  std::thread t2[thread_num];
  AtomicHashMap<int, std::shared_ptr<int>, 1024> table;
  std::unordered_map<int, std::shared_ptr<int>> map;
  std::mutex mutex;

  std::cout << thread_num
            << " threads each insert 10000 item cost:" << std::endl;
  volatile bool ready = false;
  std::atomic<uint64_t> count = {0};

  for (int j = 0; j < thread_num; j++) {
    t1[j] = std::thread([&]() {
      while (!ready) {
        cpu_relax();
      }
      for (int i = 0; i < 10000; i++) {
        table.Set(i, std::make_shared<int>(i));
        count++;
      }
    });
  }
  ready = true;
  auto start = std::chrono::steady_clock::now();
  while (count != 10000 * thread_num) {
    cpu_relax();
  }
  auto end = std::chrono::steady_clock::now();
  auto diff = std::chrono::duration<double>(end - start).count();
  std::cout << "atomic hash map: \t" << diff << "s" << std::endl;
  for (int j = 0; j < thread_num; j++) {
    t1[j].join();
  }

  ready = false;
  count = 0;
  for (int j = 0; j < thread_num; j++) {
    t2[j] = std::thread([&]() {
      while (!ready) {
        cpu_relax();
      }
      for (int i = 0; i < 10000; i++) {
        std::lock_guard<std::mutex> lock(mutex);
        map.emplace(i, std::make_shared<int>(i));
        count++;
      }
    });
  }
  ready = true;
  start = std::chrono::steady_clock::now();
  while (count != 10000 * thread_num) {
    cpu_relax();
  }
  end = std::chrono::steady_clock::now();
  diff = std::chrono::duration<double>(end - start).count();
  std::cout << "std unordered map: \t" << diff << "s" << std::endl;
  for (int j = 0; j < thread_num; j++) {
    t2[j].join();
  }

  std::cout << std::endl;
  std::cout << thread_num << " threads each get 10000 item cost:" << std::endl;
  ready = false;
  count = {0};
  for (int j = 0; j < thread_num; j++) {
    t1[j] = std::thread([&]() {
      while (!ready) {
        cpu_relax();
      }
      for (int i = 0; i < 10000; i++) {
        std::shared_ptr<int>* value;
        table.Get(i, &value);
        count++;
      }
    });
  }
  ready = true;
  start = std::chrono::steady_clock::now();
  while (count != 10000 * thread_num) {
    cpu_relax();
  }
  end = std::chrono::steady_clock::now();
  diff = std::chrono::duration<double>(end - start).count();
  std::cout << "atomic hash map: \t" << diff << "s" << std::endl;
  for (int j = 0; j < thread_num; j++) {
    t1[j].join();
  }

  ready = false;
  count = 0;
  for (int j = 0; j < thread_num; j++) {
    t2[j] = std::thread([&]() {
      while (!ready) {
        cpu_relax();
      }
      for (int i = 0; i < 10000; i++) {
        std::lock_guard<std::mutex> lock(mutex);
        std::shared_ptr<int> value = map.at(i);
        count++;
      }
    });
  }
  ready = true;
  start = std::chrono::steady_clock::now();
  while (count != 10000 * thread_num) {
    cpu_relax();
  }
  end = std::chrono::steady_clock::now();
  diff = std::chrono::duration<double>(end - start).count();
  std::cout << "std unordered map: \t" << diff << "s" << std::endl;
  for (int j = 0; j < thread_num; j++) {
    t2[j].join();
  }

  std::cout << std::endl;
  std::cout << thread_num
            << " threads each remove 10000 item cost:" << std::endl;
  ready = false;
  count = {0};
  for (int j = 0; j < thread_num; j++) {
    t1[j] = std::thread([&]() {
      while (!ready) {
        cpu_relax();
      }
      for (int i = 0; i < 10000; i++) {
        table.Remove(i);
        count++;
      }
    });
  }
  ready = true;
  start = std::chrono::steady_clock::now();
  while (count != 10000 * thread_num) {
    cpu_relax();
  }
  end = std::chrono::steady_clock::now();
  diff = std::chrono::duration<double>(end - start).count();
  std::cout << "atomic hash map: \t" << diff << "s" << std::endl;
  for (int j = 0; j < thread_num; j++) {
    t1[j].join();
  }

  ready = false;
  count = 0;
  for (int j = 0; j < thread_num; j++) {
    t2[j] = std::thread([&]() {
      while (!ready) {
        cpu_relax();
      }
      for (int i = 0; i < 10000; i++) {
        std::lock_guard<std::mutex> lock(mutex);
        map.erase(i);
        count++;
      }
    });
  }
  ready = true;
  start = std::chrono::steady_clock::now();
  while (count != 10000 * thread_num) {
    cpu_relax();
  }
  end = std::chrono::steady_clock::now();
  diff = std::chrono::duration<double>(end - start).count();
  std::cout << "std unordered map: \t" << diff << "s" << std::endl;
  for (int j = 0; j < thread_num; j++) {
    t2[j].join();
  }
  return 0;
}
