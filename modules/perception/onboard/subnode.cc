// Copyright (c) 2016 Baidu.com, Inc. All Rights Reserved

// @file subnode.h
// @author lihengyu(lihengyu@baidu.com)
// @date 2016/05/26 10:24:43
// @brief

#include "modules/perception/onboard/subnode.h"

#include <sstream>
#include <string>
#include <vector>

#include "modules/common/log.h"
#include "modules/perception/onboard/event_manager.h"

namespace apollo {
namespace perception {

using std::vector;
using std::string;
using std::ostringstream;

bool Subnode::Init(const DAGConfig::Subnode &subnode_config,
                   EventManager *event_manager,
                   SharedDataManager *shared_data_manager,
                   const vector<EventID> &sub_events,
                   const vector<EventID> &pub_events) {
  name_ = subnode_config.name();
  id_ = subnode_config.id();
  reserve_ = subnode_config.reserve();
  if (subnode_config.has_type()) {
    type_ = subnode_config.type();
  }

  CHECK(event_manager != NULL) << "event_manager == NULL";
  event_manager_ = event_manager;
  CHECK(shared_data_manager != NULL) << "shared_data_manager == NULL";
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
    StatusCode status = ProcEvents();
    ++total_count_;
    if (status == FAIL) {
      ++failed_count_;
      AWARN << "Subnode: " << name_ << " proc event failed. "
            << " total_count: " << total_count_
            << " failed_count: " << failed_count_;
      continue;
    }

    // FATAL error, so exit thread.
    if (status == FATAL) {
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

StatusCode CommonSubnode::ProcEvents() {
  CHECK(sub_meta_events_.size() == 1u) << "CommonSubnode sub_meta_events == 1";
  CHECK(pub_meta_events_.size() == 1u) << "CommonSubnode pub_meta_events == 1";

  Event sub_event;
  if (!event_manager_->Subscribe(sub_meta_events_[0].event_id, &sub_event)) {
    AERROR << "failed to subscribe. meta_event: <"
           << sub_meta_events_[0].to_string() << ">";
    return FAIL;
  }

  Event pub_event = sub_event;
  pub_event.event_id = pub_meta_events_[0].event_id;

  // user defined logic api.
  if (!HandleEvent(sub_event, &pub_event)) {
    AWARN << "failed to call _handle_event. sub_event: <"
          << sub_event.to_string() << "> pub_event: <" << pub_event.to_string();
    return FAIL;
  }

  if (!event_manager_->Publish(pub_event)) {
    AERROR << "failed to publish pub_event: <" << pub_event.to_string() << ">";
    return FAIL;
  }

  return SUCC;
}

}  // namespace perception
}  // namespace apollo
