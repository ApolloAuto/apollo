#include <iostream>
#include <thread>
#include <mutex>
#include <chrono>
#include "cybertron/base/atomic_rw_lock.h"

using namespace apollo::cybertron::base;

// using AtomicRWLock = std::mutex;
// using ReadLockGuard = std::lock_guard<std::mutex>;
// using WriteLockGuard = std::lock_guard<std::mutex>;

int x = 0;
AtomicRWLock rw_lock;

void readlock() {
  ReadLockGuard lock(rw_lock);
//   std::cout << "read lock x: " << x << std::endl;
  usleep(1);
}

void writelock() {
  WriteLockGuard lock(rw_lock);
  x += 1;
  // std::cout << "write lock x: " << x << std::endl;
//   usleep(1);
  readlock();
}

void readloop() {
  for (int i = 0; i < 10000; ++i) {
    readlock();
  }
}

void writeloop() {
  for (int i = 0; i < 10000; ++i) {
    writelock();
  }
}

int main() {
  auto start = std::chrono::steady_clock::now();
  std::thread rt[30];
  for (int i = 0; i < 30; ++i) {
    rt[i] = std::thread(readloop);
  }

  std::thread t(writeloop);

  t.join();
  for (int i = 0; i < 30; ++i) {
    rt[i].join();
  }
  auto end = std::chrono::steady_clock::now();
  std::chrono::duration<double> diff = end - start;
  std::cout << std::fixed << diff.count() << " s\n";
}