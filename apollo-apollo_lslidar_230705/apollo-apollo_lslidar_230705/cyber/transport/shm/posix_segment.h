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

#ifndef CYBER_TRANSPORT_SHM_POSIX_SEGMENT_H_
#define CYBER_TRANSPORT_SHM_POSIX_SEGMENT_H_

#include <string>

#include "cyber/transport/shm/segment.h"

namespace apollo {
namespace cyber {
namespace transport {

class PosixSegment : public Segment {
 public:
  explicit PosixSegment(uint64_t channel_id);
  virtual ~PosixSegment();

  static const char* Type() { return "posix"; }

 private:
  void Reset() override;
  bool Remove() override;
  bool OpenOnly() override;
  bool OpenOrCreate() override;

  std::string shm_name_;
};

}  // namespace transport
}  // namespace cyber
}  // namespace apollo

#endif  // CYBER_TRANSPORT_SHM_POSIX_SEGMENT_H_
