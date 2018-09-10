
#include <iostream>
#include <mutex>
#include <queue>
#include <thread>
#include <vector>

#include "cybertron/base/atomic_fifo.h"

using namespace apollo::cybertron;

#define SIZE 1024 * 20
#define NUM_OBJECT 100
#define NUM_THREAD 30

struct Task {
  const char *name;
};

Task *t;
Task *t1;

std::queue<Task *> fifo;
std::mutex mtx;

void fatomic() {
  AtomicFIFO<Task *> *fifo = AtomicFIFO<Task *>::GetInstance();

  auto start = std::chrono::steady_clock::now();

  for (int i = 0; i < NUM_OBJECT; i++) {
    fifo->Push(t);
    fifo->Pop(&t1);
  }

  auto end = std::chrono::steady_clock::now();

  std::chrono::duration<double> diff = end - start;
  std::cout << std::fixed << diff.count() << " s\n";
}

void fstd() {
  auto start = std::chrono::steady_clock::now();

  for (int i = 0; i < NUM_OBJECT; i++) {
    {
      std::lock_guard<std::mutex> lk(mtx);
      fifo.push(t);
    }
    {
      std::lock_guard<std::mutex> lk(mtx);
      t1 = fifo.front();
      fifo.pop();
    }
  }

  auto end = std::chrono::steady_clock::now();

  std::chrono::duration<double> diff = end - start;
  std::cout << std::fixed << diff.count() << " s\n";
}

int main(int argc, char *argv[]) {
  if (argc < 2) {
    std::cout << "Usage:program atomic/std.\n";
    return 0;
  }

  AtomicFIFO<Task *> *fifo;
  std::vector<std::thread> v;
  t = new Task();

  if (!strcmp(argv[1], "atomic")) fifo = AtomicFIFO<Task *>::GetInstance(SIZE);

  for (int i = 0; i < NUM_THREAD; i++) {
    if (!strcmp(argv[1], "atomic"))
      v.push_back(std::thread(fatomic));
    else
      v.push_back(std::thread(fstd));
  }

  for (int i = 0; i < NUM_THREAD; i++) v[i].join();

  return 0;
}
