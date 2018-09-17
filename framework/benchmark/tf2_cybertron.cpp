#include <memory>
#include <iostream>
#include <chrono>
#include <cstdio>
#include <thread>
#include <mutex>
#include <atomic>

#include <cybertron/cybertron.h>
#include <cybertron/tf2_cybertron/buffer.h>
#include <cybertron/tf2_cybertron/transform_broadcaster.h>
#include <cybertron/tf2_cybertron/static_transform_broadcaster.h>

using std::cout;
using std::endl;
using std::cerr;

using std::chrono::system_clock;
using std::chrono::steady_clock;
using std::chrono::high_resolution_clock;

using apollo::cybertron::tf2_cybertron::TransformBroadcaster;
using apollo::cybertron::tf2_cybertron::Buffer;

std::atomic_ullong can_total_cost(0);
std::atomic_ullong can_cnt(0);
std::atomic_ullong lookup_total_cost(0);
std::atomic_ullong lookup_cnt(0);

uint64_t now_time_ns() {
  high_resolution_clock::time_point now = high_resolution_clock::now();
  auto nano_now = std::chrono::time_point_cast<std::chrono::nanoseconds>(now);
  auto epoch = nano_now.time_since_epoch();
  uint64_t nano_time =
      std::chrono::duration_cast<std::chrono::nanoseconds>(epoch).count();
  return nano_time;
}

uint64_t now_time_us() {
  uint64_t us = now_time_ns() / 1e3;
  return us;
}

uint64_t now_time_ms() {
  uint64_t ms = now_time_ns() / 1e6;
  return ms;
}

int thread_nums = 1;

std::atomic<bool> is_stop(false);

void set_transform_200hz() {
  uint64_t total_cost = 0;
  adu::common::TransformStamped msg;
  msg.mutable_transform()->mutable_translation()->set_x(0.002035109898165613);
  msg.mutable_transform()->mutable_translation()->set_y(1.568507035900799);
  msg.mutable_transform()->mutable_translation()->set_z(1.168094515800476);
  msg.mutable_transform()->mutable_rotation()->set_qx(-0.002020572112387558);
  msg.mutable_transform()->mutable_rotation()->set_qy(0.003409817242305666);
  msg.mutable_transform()->mutable_rotation()->set_qz(0.7101986689700617);
  msg.mutable_transform()->mutable_rotation()->set_qw(0.7039901569112067);
  msg.mutable_header()->set_stamp(
      apollo::cybertron::Time::Now().ToNanosecond());
  msg.mutable_header()->set_frame_id("novatel");
  msg.set_child_frame_id("velodyne64");

  std::shared_ptr<apollo::cybertron::Node> static_node(
      apollo::cybertron::CreateNode(
          "static_transform_publisher" +
          std::to_string(apollo::cybertron::Time::Now().ToNanosecond())));
  apollo::cybertron::tf2_cybertron::StaticTransformBroadcaster static_broadcaster(
      static_node);

  static_broadcaster.sendTransform(msg);

  TransformBroadcaster broadcaster;
  while(true) {
    adu::common::TransformStamped msgtf;
    uint64_t stamp = now_time_ns();
    msgtf.set_child_frame_id("novatel");
    msgtf.mutable_header()->set_stamp(stamp);
    msgtf.mutable_header()->set_frame_id("world");

    msgtf.mutable_transform()->mutable_rotation()->set_qx(0.0);
    msgtf.mutable_transform()->mutable_rotation()->set_qy(0.0);
    msgtf.mutable_transform()->mutable_rotation()->set_qz(0.0);
    msgtf.mutable_transform()->mutable_rotation()->set_qw(1.0);

    msgtf.mutable_transform()->mutable_translation()->set_x(1.0);
    msgtf.mutable_transform()->mutable_translation()->set_y(1.0);
    msgtf.mutable_transform()->mutable_translation()->set_z(1.0);
    uint64_t start = now_time_ns();
    broadcaster.sendTransform(msgtf);
    uint64_t end = now_time_ns();
    total_cost += end - start;
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
    if (is_stop) {
      break;
    }
  }
}

void can_thread(const std::string& source_frame, const std::string& target_frame) {
  std::shared_ptr<Buffer> buffer = Buffer::Instance();
  std::string errstr;
  for (uint64_t i = 0; i < 1000; ++i ) {
    uint64_t stamp = now_time_ns() - 10 * 1e6;
    apollo::cybertron::Time time(stamp);
    uint64_t start = now_time_ns();
    bool ret = buffer->canTransform(target_frame, source_frame, time, 10, &errstr);
    uint64_t end = now_time_ns();
    if (thread_nums == 1) {
      // cout << "canTransform, source_frame: " << source_frame
      //      << ", target_frame: " << target_frame
      //      << ", ret: " << ret << endl;
    }
    can_total_cost.fetch_add((end - start));
    can_cnt++;
  }
}

void lookup_thread(const std::string& source_frame,
                   const std::string& target_frame) {
  std::shared_ptr<Buffer> buffer = Buffer::Instance();
  // Buffer buffer;
  std::string errstr;
  for (uint64_t i = 0; i < 1000; ++i) {
    uint64_t stamp = now_time_ns() - 10 * 1e6;
    apollo::cybertron::Time time(stamp);
    uint64_t start = now_time_ns();
    adu::common::TransformStamped ret;
    try {
        ret = buffer->lookupTransform(target_frame, source_frame, time, 1);
    } catch(...) {
      std::cout << "lookupTransform Exception" << std::endl;
    }
    uint64_t end = now_time_ns();
    if (thread_nums == 1) {
      cout << "lookupTransform, source_frame: " << source_frame
           << ", target_frame: " << target_frame
           << ", stamp: " << ret.header().stamp()
           << ", frame_id: " << ret.header().frame_id()
           << ", child_frame_id: " << ret.child_frame_id()
           << ", x: " << ret.transform().translation().x()
           << ", y: " << ret.transform().translation().y()
           << ", z: " << ret.transform().translation().z()
           << ", qx: " << ret.transform().rotation().qx()
           << ", qy: " << ret.transform().rotation().qy()
           << ", qz: " << ret.transform().rotation().qz()
           << ", qw: " << ret.transform().rotation().qw() << endl;
    }
    lookup_total_cost.fetch_add((end - start));
    lookup_cnt++;
  }
}

int main(int argc, char** argv) {
  apollo::cybertron::Init(argv[0]);
  if (argc  == 2) {
    thread_nums = atoi(argv[1]);
  }

  std::thread set_200hz(set_transform_200hz);
  // set_200hz.detach();

  std::string source_frame = "velodyne64";
  std::string target_frame = "world";

  cout << "Max Thread Nums: " << thread_nums << endl;
  std::vector<std::thread> th_vecs;
  std::vector<std::thread> lookup_threads;
  for (int i = 1; i <= thread_nums; ++i ) {
    can_total_cost = 0;
    lookup_total_cost = 0;
    can_cnt = 0;
    lookup_cnt = 0;
    for ( int j = 1; j <= i; ++j) {
      th_vecs.push_back(std::thread(can_thread, source_frame, target_frame));
      lookup_threads.push_back(std::thread(lookup_thread, source_frame, target_frame));
    }
    for (auto &th: th_vecs) {
      th.join();
    }
    for (auto &th: lookup_threads) {
      th.join();
    }
    th_vecs.clear();
    lookup_threads.clear();
    std::cout << "canTransform, Threads: " << i
              << ", Exec Cnt: " << can_cnt.load()
              << ", Total Time(ms): " << static_cast<double>(can_total_cost.load()) / 1e6
              << ", Avg Time(ms): " << static_cast<double>(can_total_cost.load()) / 1e6 / can_cnt.load()
              << endl;

    std::cout << "lookupTransform, Threads: " << i
              << ", Exec Cnt: " << lookup_cnt.load()
              << ", Total Time(ms): " << static_cast<double>(lookup_total_cost.load()) / 1e6
              << ", Avg Time(ms): " << static_cast<double>(lookup_total_cost.load()) / 1e6 / lookup_cnt.load()
              << endl;
  }
  is_stop = true;
  if (set_200hz.joinable()) {
    set_200hz.join();
  }
  apollo::cybertron::Shutdown();
  return 0;
}
