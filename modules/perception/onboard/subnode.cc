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

#include "modules/perception/onboard/subnode.h"

#include <sstream>

#include "modules/common/log.h"

namespace apollo {
namespace perception {

using apollo::common::ErrorCode;
using apollo::common::Status;
using std::vector;
using std::string;
using std::ostringstream;

bool Subnode::Init(const DAGConfig::Subnode &subnode_config,
                   const vector<EventID> &sub_events,
                   const vector<EventID> &pub_events,
                   EventManager *event_manager,
                   SharedDataManager *shared_data_manager) {
  name_ = subnode_config.name();
  id_ = subnode_config.id();
  reserve_ = subnode_config.reserve();
  if (subnode_config.has_type()) {
    type_ = subnode_config.type();
  }

  CHECK(event_manager != nullptr) << "event_manager == nullptr";
  event_manager_ = event_manager;
  CHECK(shared_data_manager != nullptr) << "shared_data_manager == nullptr";
  shared_data_manager_ = shared_data_manager;

  // fill sub and pub meta events.
  if (!event_manager_->GetEventMeta(sub_events, &sub_meta_events_)) {
    AERROR << "failed to get Sub EventMeta. node: <" << name_ << ", " << id_
           << ">";
    return false;
  }

  if (!event_manager_->GetEventMeta(pub_events, &pub_meta_events_)) {
    AERROR << "failed to get Pub EventMeta. node: <" << id_ << ", " << name_
           << ">";
    return false;
  }

  if (!InitInternal()) {
    AERROR << "failed to Init inner members.";
    return false;
  }

  inited_ = true;
  return true;
}

void Subnode::Run() {
  if (!inited_) {
    AERROR << "Subnode not inited, run failed. node: <" << id_ << ", " << name_
           << ">";
    return;
  }

  if (type_ == DAGConfig::SUBNODE_IN) {
    AINFO << "Subnode == SUBNODE_IN, EXIT THREAD. subnode:" << DebugString();
    return;
  }

  while (!stop_) {
    Status status = ProcEvents();
    ++total_count_;
    if (status.code() == ErrorCode::PERCEPTION_ERROR) {
      ++failed_count_;
      AWARN << "Subnode: " << name_ << " proc event failed. "
            << " total_count: " << total_count_
            << " failed_count: " << failed_count_;
      continue;
    }

    // FATAL error, so exit thread.
    if (status.code() == ErrorCode::PERCEPTION_FATAL) {
      AERROR << "Subnode: " << name_ << " proc event FATAL error, EXIT. "
             << " total_count: " << total_count_
             << " failed_count: " << failed_count_;
      break;
    }
  }
}

string Subnode::DebugString() const {
  ostringstream oss;
  oss << "{id: " << id_ << ", name: " << name_ << ", reserve: " << reserve_
      << ", type:" << DAGConfig::SubnodeType_Name(type_);

  oss << ", SubEvents: [";
  for (size_t idx = 0; idx < sub_meta_events_.size(); ++idx) {
    oss << "<" << sub_meta_events_[idx].to_string() << ">";
  }

  oss << "], PubEvents: [";
  for (size_t idx = 0; idx < pub_meta_events_.size(); ++idx) {
    oss << "<" << pub_meta_events_[idx].to_string() << ">";
  }
  oss << "]}";

  return oss.str();
}

Status CommonSubnode::ProcEvents() {
  CHECK(sub_meta_events_.size() == 1u) << "CommonSubnode sub_meta_events == 1";
  CHECK(pub_meta_events_.size() == 1u) << "CommonSubnode pub_meta_events == 1";

  Event sub_event;
  if (!event_manager_->Subscribe(sub_meta_events_[0].event_id, &sub_event)) {
    AERROR << "failed to subscribe. meta_event: <"
           << sub_meta_events_[0].to_string() << ">";
    return Status(ErrorCode::PERCEPTION_ERROR, "Failed to subscribe event.");
  }

  Event pub_event = sub_event;
  pub_event.event_id = pub_meta_events_[0].event_id;

  // user defined logic api.
  if (!HandleEvent(sub_event, &pub_event)) {
    AWARN << "failed to call handle_event_. sub_event: <"
          << sub_event.to_string() << "> pub_event: <" << pub_event.to_string();
    return Status(ErrorCode::PERCEPTION_ERROR, "Failed to call handle_event_.");
  }

  if (!event_manager_->Publish(pub_event)) {
    AERROR << "failed to publish pub_event: <" << pub_event.to_string() << ">";
    return Status(ErrorCode::PERCEPTION_ERROR, "Failed to publish pub_event.");
  }

  return Status::OK();
}

}  // namespace perception
}  // namespace apollo
