
#include <thread>
#include "cybertron/base/object_pool.h"

using namespace apollo::cybertron::base;

#define POOL_SIZE 1024
#define NUM_OBJECT 512

struct Test {
  // int a[100];
  int a[4096 * 3];
};

void fpool() {
  auto pool = ObjectPool<Test>::Instance(100);

  auto start = std::chrono::steady_clock::now();

  int c = 0;
  while (c < NUM_OBJECT)
    if (pool->GetObject()) c++;

  auto end = std::chrono::steady_clock::now();

  std::chrono::duration<double> diff = end - start;
  std::cout << std::fixed << diff.count() << " s\n";
}

void fnpool() {
  auto start = std::chrono::steady_clock::now();

  int c = 0;
  while (c < NUM_OBJECT)
    if (new Test()) c++;

  auto end = std::chrono::steady_clock::now();

  std::chrono::duration<double> diff = end - start;
  std::cout << std::fixed << diff.count() << " s\n";
}

int main(int argc, char *argv[]) {
  if (argc < 2) {
    std::cout << "Usage:program pool/npool.\n";
    return 0;
  }

  std::shared_ptr<ObjectPool<Test>> pool;
  std::thread t;

  if (!strcmp(argv[1], "pool")) pool = ObjectPool<Test>::Instance(POOL_SIZE);

  if (!strcmp(argv[1], "pool"))
    t = std::thread(fpool);
  else
    t = std::thread(fnpool);

  t.join();

  return 0;
}
