/******************************************************************************
 * Copyright 2023 The Apollo Authors. All Rights Reserved.
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

#ifndef CYBER_PROFILER_BLOCK_H_
#define CYBER_PROFILER_BLOCK_H_

#include <chrono>
#include <string>

namespace apollo {
namespace cyber {
namespace profiler {

class Block {
 public:
  using time_point = std::chrono::time_point<std::chrono::steady_clock>;

 public:
  Block();
  explicit Block(const std::string& name);
  virtual ~Block();

  void Start();
  void End();

  const std::string& name() const { return name_; }
  std::uint32_t depth() const { return depth_; }
  void set_depth(std::uint32_t depth) { depth_ = depth; }

  const time_point& begin_time() const { return begin_time_; }
  const time_point& end_time() const { return end_time_; }

  std::uint64_t begin_time_since_epoch() const;
  std::uint64_t end_time_since_epoch() const;
  std::uint64_t duration() const;

  bool finished() const { return end_time_ > begin_time_; }

 private:
  std::string name_;
  std::uint32_t depth_;
  time_point begin_time_;
  time_point end_time_;
};

}  // namespace profiler
}  // namespace cyber
}  // namespace apollo

#endif  // CYBER_PROFILER_BLOCK_H_
