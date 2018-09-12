
#include <thread>
#include <vector>

#include "cybertron/base/concurrent_object_pool.h"

using namespace apollo::cybertron::base;

#define POOL_SIZE 1024 * 20
#define NUM_OBJECT 100
#define NUM_THREAD 1

struct Test {
  int a[256 * 1024];
  //int a[100];
};

void fpool()
{
  auto pool = CCObjectPool<Test>::Instance(100);

  auto start = std::chrono::steady_clock::now();

  int c = 0;
  std::shared_ptr<Test> t;
  while (c < NUM_OBJECT)
      if ( t = pool->GetObject()) c++;
  
  auto end = std::chrono::steady_clock::now();

  std::chrono::duration<double> diff = end-start;
  std::cout << std::fixed << diff.count() << " s\n";
}

void fnpool()
{
  auto start = std::chrono::steady_clock::now();
  
  int c = 0;
  int i = 0;
  Test *t[NUM_OBJECT];
  while (c < NUM_OBJECT)
      if (t[i++] = new Test()) c++;

  auto end = std::chrono::steady_clock::now();

  std::chrono::duration<double> diff = end-start;
  std::cout << std::fixed << diff.count() << " s\n";

  for (i = 0; i < NUM_OBJECT - 1; i++)
    free(t[i]);
}

int main(int argc, char *argv[])
{
  if (argc < 2) {
    std::cout << "Usage:program pool/npool.\n";
    return 0;
  }

  int i;
  std::vector<std::thread> v;

  for (i = 0; i < NUM_THREAD; i++) {
    if (!strcmp(argv[1], "pool"))
      v.push_back(std::thread(fpool));
    else
      v.push_back(std::thread(fnpool));
  }

  for (i = 0; i < NUM_THREAD; i++)
    v[i].join();

  return 0;
}
