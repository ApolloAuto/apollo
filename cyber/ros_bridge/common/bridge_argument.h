/******************************************************************************
 * Copyright 2018 The Apollo Authors. All Rights Reserved.
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

#pragma once

#include <list>
#include <string>

#include "cyber/common/global_data.h"
#include "cyber/common/log.h"
#include "cyber/common/types.h"

namespace apollo {
namespace cyber {

class BridgeArgument {
 public:
  BridgeArgument() = default;
  virtual ~BridgeArgument() = default;
  void DisplayUsage();
  void ParseArgument(int argc, char* const argv[]);
  void GetOptions(const int argc, char* const argv[]);
  const std::string& GetBinaryName() const;
  const bool GetEnableCpuprofile() const { return enable_cpuprofile_; }
  const std::string GetProfileFilename() const { return profile_filename_; }
  const bool GetEnableHeapprofile() const { return enable_heapprofile_; }
  const std::string GetHeapProfileFilename() const {
    return heapprofile_filename_;
  }

 private:
  std::string binary_name_;
  bool enable_cpuprofile_ = false;
  std::string profile_filename_;
  bool enable_heapprofile_ = false;
  std::string heapprofile_filename_;
};

inline const std::string& BridgeArgument::GetBinaryName() const {
  return binary_name_;
}

}  // namespace cyber
}  // namespace apollo
