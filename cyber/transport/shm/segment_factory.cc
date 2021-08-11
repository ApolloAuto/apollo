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

#include "cyber/transport/shm/segment_factory.h"

#include <memory>
#include <string>

#include "cyber/common/global_data.h"
#include "cyber/common/log.h"
#include "cyber/transport/shm/posix_segment.h"
#include "cyber/transport/shm/xsi_segment.h"

namespace apollo {
namespace cyber {
namespace transport {

using apollo::cyber::common::GlobalData;

auto SegmentFactory::CreateSegment(uint64_t channel_id) -> SegmentPtr {
  std::string segment_type(XsiSegment::Type());
  auto& shm_conf = GlobalData::Instance()->Config();
  if (shm_conf.has_transport_conf() &&
      shm_conf.transport_conf().has_shm_conf() &&
      shm_conf.transport_conf().shm_conf().has_shm_type()) {
    segment_type = shm_conf.transport_conf().shm_conf().shm_type();
  }

  ADEBUG << "segment type: " << segment_type;

  if (segment_type == PosixSegment::Type()) {
    return std::make_shared<PosixSegment>(channel_id);
  }

  return std::make_shared<XsiSegment>(channel_id);
}

}  // namespace transport
}  // namespace cyber
}  // namespace apollo
