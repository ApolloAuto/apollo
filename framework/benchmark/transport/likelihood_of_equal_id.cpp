#include <iostream>
#include <string>
#include <unordered_set>

#include "cybertron/time/time.h"
#include "cybertron/transport/common/identity.h"

using apollo::cybertron::Time;
using apollo::cybertron::transport::Identity;

const uint64_t LARGE_TIME = 123456789;

struct RunConfig {
  RunConfig() : id_num(1000000) {}

  void Display() {
    std::cout << "We will generate[" << id_num << "] ids." << std::endl;
  }

  uint64_t id_num;
};

struct RunStatistics {
  RunStatistics()
      : min_gen_time(LARGE_TIME),
        max_gen_time(0),
        avg_gen_time(0),
        total_gen_time(0),
        unique_id_num(0),
        unique_hash_num(0) {}

  void Display() {
    std::cout << "***** Results *****" << std::endl;
    std::cout << "   min_gen_time:" << min_gen_time << std::endl;
    std::cout << "   max_gen_time:" << max_gen_time << std::endl;
    std::cout << "   avg_gen_time:" << avg_gen_time << std::endl;
    std::cout << " total_gen_time:" << total_gen_time << std::endl;
    std::cout << "  unique_id_num:" << unique_id_num << std::endl;
    std::cout << "unique_hash_num:" << unique_hash_num << std::endl;
  }

  uint64_t min_gen_time;
  uint64_t max_gen_time;
  uint64_t avg_gen_time;
  uint64_t total_gen_time;

  uint64_t unique_id_num;
  uint64_t unique_hash_num;
};

void Usage() {
  std::cout << "Usage:" << std::endl;
  std::cout << "    argv[0] program name" << std::endl;
  std::cout << "    argv[1] id number to generate" << std::endl;
}

bool Parse(const char* param, RunConfig* cfg) {
  if (cfg == nullptr) {
    return false;
  }

  int tmp = 0;
  tmp = atoi(param);
  if (tmp <= 0) {
    std::cout << "param invalid" << std::endl;
    return false;
  }

  cfg->id_num = tmp;
  return true;
}

void Run(const uint64_t& num) {
  if (num == 0) {
    return;
  }

  RunStatistics stat;
  std::unordered_set<std::string> ids;
  std::unordered_set<uint64_t> id_hash_values;

  for (uint64_t i = 0; i < num; ++i) {
    uint64_t start = Time::Now().ToNanosecond();
    Identity id;
    uint64_t end = Time::Now().ToNanosecond();
    uint64_t cost = end - start;
    if (cost > stat.max_gen_time) {
      stat.max_gen_time = cost;
    }
    if (cost < stat.min_gen_time) {
      stat.min_gen_time = cost;
    }
    stat.total_gen_time += cost;
    ids.emplace(id.ToString());
    id_hash_values.emplace(id.HashValue());
  }

  stat.avg_gen_time = stat.total_gen_time / num;
  stat.unique_id_num = ids.size();
  stat.unique_hash_num = id_hash_values.size();
  stat.Display();

  if (stat.unique_id_num != num) {
    std::cout << "\nWarning: [" << num - stat.unique_id_num
              << "] id(s) duplicate." << std::endl;
    std::cout << "*****************************" << std::endl;
  }

  if (stat.unique_hash_num != num) {
    std::cout << "\nWarning: [" << num - stat.unique_hash_num
              << "] id hash value(s) duplicate." << std::endl;
    std::cout << "*****************************" << std::endl;
  }
}

int main(int argc, char* argv[]) {
  if (argc > 2) {
    Usage();
    return -1;
  }

  RunConfig cfg;
  if (argc == 2 && !Parse(argv[1], &cfg)) {
    return -1;
  }

  cfg.Display();

  Run(cfg.id_num);

  return 0;
}