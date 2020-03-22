/******************************************************************************
 * Copyright 2019 The Apollo Authors. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *****************************************************************************/

#include <atomic>
#include <chrono>
#include <thread>

#include "gtest/gtest.h"

#include "cyber/record/file/record_file_base.h"
#include "cyber/record/file/record_file_reader.h"
#include "cyber/record/file/record_file_writer.h"
#include "cyber/record/header_builder.h"

namespace apollo {
namespace cyber {
namespace record {

class CpuSchedulerLatency {
 public:
  CpuSchedulerLatency() : periodic_thread_([this] { this->Callback(); }) {}

  ~CpuSchedulerLatency() {
    running_ = false;
    periodic_thread_.join();
  }

  std::chrono::nanoseconds GetMaxJitter() {
    return std::chrono::nanoseconds(max_jitter_);
  }

  int64_t GetNumSamples() { return samples_; }

 private:
  void Callback() {
    static constexpr std::chrono::milliseconds kSleepDuration(10);
    auto prev_time = std::chrono::steady_clock::now();
    std::this_thread::sleep_for(kSleepDuration);
    while (running_) {
      const auto current_time = std::chrono::steady_clock::now();
      const auto time_since_sleep = current_time - prev_time;
      const auto current_jitter =
          std::abs((time_since_sleep - kSleepDuration).count());
      prev_time = current_time;
      max_jitter_ = std::max(current_jitter, max_jitter_);
      ++samples_;
      std::this_thread::sleep_for(kSleepDuration);
    }
  }

  std::atomic<bool> running_{true};
  int64_t max_jitter_ = 0;
  int64_t samples_ = 0;

  std::thread periodic_thread_;
};

const char kTestFile[] = "integration_test.record";

TEST(RecordFileTest, SmallMessageHighThroughputOKThreadJitter) {
  CpuSchedulerLatency cpu_jitter;

  RecordFileWriter rfw;

  ASSERT_TRUE(rfw.Open(kTestFile));

  proto::Header hdr1 = HeaderBuilder::GetHeaderWithSegmentParams(0, 0);
  hdr1.set_chunk_interval(1000);
  hdr1.set_chunk_raw_size(0);
  ASSERT_TRUE(rfw.WriteHeader(hdr1));
  ASSERT_FALSE(rfw.GetHeader().is_complete());

  // write chunk section
  static const std::string kChannelName = "small_message";

  static constexpr int kMaxIterations = 1000000000;
  static constexpr int64_t kMaxSamples = 1000;
  for (int i = 0;
       i < kMaxIterations && cpu_jitter.GetNumSamples() < kMaxSamples; ++i) {
    proto::SingleMessage msg1;
    msg1.set_channel_name(kChannelName);
    msg1.set_content("0123456789");
    msg1.set_time(i);
    ASSERT_TRUE(rfw.WriteMessage(msg1));
    ASSERT_EQ(i + 1, rfw.GetMessageNumber(kChannelName));
  }

  EXPECT_GE(cpu_jitter.GetNumSamples(), kMaxSamples)
      << "This system may be to fast. Consider increasing kMaxIterations";
  static constexpr int64_t kMaxJitterMS = 20;
  const int64_t max_cpu_jitter_ms =
      std::chrono::duration_cast<std::chrono::milliseconds>(
          cpu_jitter.GetMaxJitter())
          .count();
  EXPECT_LT(max_cpu_jitter_ms, kMaxJitterMS);
  ASSERT_FALSE(remove(kTestFile));
}

}  // namespace record
}  // namespace cyber
}  // namespace apollo

int main(int argc, char** argv) {
  testing::GTEST_FLAG(catch_exceptions) = 1;
  testing::InitGoogleTest(&argc, argv);
  google::InitGoogleLogging(argv[0]);
  FLAGS_logtostderr = true;
  const int ret_val = RUN_ALL_TESTS();
  google::protobuf::ShutdownProtobufLibrary();
  google::ShutDownCommandLineFlags();
  return ret_val;
}
