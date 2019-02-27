/*
 * Copyright (c) 2010, Willow Garage, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include <stdio.h>

#include <tf2/buffer_core.h>
#include "tf2/time.h"
#include <boost/lexical_cast.hpp>
#include <chrono>
#include <thread>
#include <mutex>
#include <atomic>

using std::chrono::system_clock;
using std::chrono::steady_clock;
using std::chrono::high_resolution_clock;

uint64_t now_time() {
  high_resolution_clock::time_point now = high_resolution_clock::now();
  auto nano_now = std::chrono::time_point_cast<std::chrono::nanoseconds>(now);
  auto epoch = nano_now.time_since_epoch();
  uint64_t nano_time =
      std::chrono::duration_cast<std::chrono::nanoseconds>(epoch).count();
  return nano_time;
}

tf2::BufferCore bc;
std::atomic<bool> is_stop(false);

void set_trans_form_1000() {
  for (uint64_t i = 0; i < 1000; ++i) {
    geometry_msgs::TransformStamped t;
    t.header.stamp = i;
    t.header.frame_id = "world";
    t.child_frame_id = "novatel";
    t.transform.translation.x = 1;
    t.transform.rotation.w = 1.0;
    bc.setTransform(t, "test");
  }
}

std::mutex cout_mutex;

std::atomic<uint64_t> total_cost_time(0);
std::atomic<uint64_t> total_exec_cnt(0);

void look_transform(int count, int look_idx = 0) {
  std::string frame_target = "world";
  std::string frame_source = "velodyne64";
  if (look_idx >= 1000) {
    look_idx = 999;
  }
  geometry_msgs::TransformStamped out_t;
  for (int i = 0; i < count; ++i) {
    uint64_t start = now_time(); 
    out_t = bc.lookupTransform(frame_target, frame_source, look_idx);
    uint64_t end = now_time();
    double dur = (double)end - (double)start;
    total_cost_time.fetch_add(dur);
    total_exec_cnt.fetch_add(1);
  }
}

std::atomic<uint64_t> can_total_cost(0);
std::atomic<uint64_t> can_exec_cnt(0);

void can_transform(int count, int look_idx = 0) {
  std::string frame_target = "world";
  std::string frame_source = "velodyne64";
  if (look_idx >= 1000) {
    look_idx = 999;
  }
  for (int i = 0; i < count; ++i) {
    uint64_t start = now_time(); 
    bc.canTransform(frame_target, frame_source, look_idx);
    uint64_t end = now_time();
    double dur = (double)end - (double)start;
    can_total_cost.fetch_add(dur);
    can_exec_cnt.fetch_add(1);
  }
}

int main(int argc, char **argv) {
  set_trans_form_1000();
  geometry_msgs::TransformStamped t;
  t.header.stamp = 0;
  t.header.frame_id = "novatel";
  t.child_frame_id = "velodyne64";
  t.transform.translation.x = 1;
  t.transform.rotation.w = 1.0;
  bc.setTransform(t, "test", true);
  int th_nums = 1;
  int lookup_index = 0;
  // if (argc >= 2) {
  //   th_nums = boost::lexical_cast<int>(argv[1]);
  // }
  // if (argc >= 3) {
  //   lookup_index = boost::lexical_cast<int>(argv[2]);
  // }
  std::cout << "lookup max thread nums: " << th_nums << ", lookup index: " << lookup_index << std::endl;
  std::vector<std::thread> td_vec;
  std::vector<std::thread> can_tds;
  for (int i = 1; i <= th_nums; ++i) {
    total_cost_time = 0;
    total_exec_cnt = 0;

    can_total_cost = 0;
    can_exec_cnt = 0;
    for (int j = 1; j <= i; ++j) {
      td_vec.push_back(std::thread(look_transform, 100, lookup_index));
      can_tds.push_back(std::thread(can_transform, 100, lookup_index));
    }
    for (auto &td : td_vec) {
      td.join();
    }

    for (auto &td : can_tds) {
      td.join();
    }
    td_vec.clear();
    can_tds.clear();
    std::cout << "Thread Nums: " << i
              << ", lookup cnt: " << total_exec_cnt.load()
              << ", Total Time(ms): "
              << static_cast<double>(total_cost_time.load()) / 1e6
              << ", Avg Time(ms): "
              << static_cast<double>(total_cost_time.load()) / 1e6 /
                     total_exec_cnt.load()
              << std::endl;

    std::cout << "Thread Nums: " << i
              << ", can cnt: " << can_exec_cnt.load()
              << ", Total Time(ms): "
              << static_cast<double>(can_total_cost.load()) / 1e6
              << ", Avg Time(ms): "
              << static_cast<double>(can_total_cost.load()) / 1e6 /
                     can_exec_cnt.load()
              << std::endl;
  }
}
