#include <chrono>
#include <iostream>
#include <mutex>
#include <thread>
#include "cybertron/base/atomic_rw_lock.h"
#include "cybertron/base/reentrant_rw_lock.h"

using apollo::cybertron::base::AtomicRWLock;
using apollo::cybertron::base::ReadLockGuard;
using apollo::cybertron::base::ReentrantRWLock;
using apollo::cybertron::base::WriteLockGuard;

int x = 0;
AtomicRWLock rw_lock;
ReentrantRWLock reentrant_lock;

void read_lock() {
  ReadLockGuard<AtomicRWLock> lock(rw_lock);
  std::cout << "read lock x: " << x << std::endl;
}

void write_lock() {
  WriteLockGuard<AtomicRWLock> lock(rw_lock);
  std::cout << "write lock x: " << ++x << std::endl;
}

void reentrant_read_lock() {
  ReadLockGuard<ReentrantRWLock> lock(reentrant_lock);
  std::cout << "read lock x: " << x << std::endl;
}

void reentrant_write_lock() {
  WriteLockGuard<ReentrantRWLock> lock(reentrant_lock);
  std::cout << "write lock x: " << x++ << std::endl;
}

void reentrant_read_write() {
  WriteLockGuard<AtomicRWLock> lock(rw_lock);
  std::cout << "write lock x: " << x++ << std::endl;
  reentrant_read_lock();
  reentrant_write_lock();
}

int main() {
  read_lock();
  write_lock();
  reentrant_read_write();
}