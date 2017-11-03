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

bool Subnode::init(const DAGConfig::Subnode& subnode_config,
                   EventManager* event_manager,
                   SharedDataManager* shared_data_manager,
                   const vector<EventID>& sub_events,
                   const vector<EventID>& pub_events) {
  _name = subnode_config.name();
  _id = subnode_config.id();
  _reserve = subnode_config.reserve();
  if (subnode_config.has_type()) {
    _type = subnode_config.type();
  }

  CHECK(event_manager != NULL) << "event_manager == NULL";
  _event_manager = event_manager;
  CHECK(shared_data_manager != NULL) << "shared_data_manager == NULL";
  _shared_data_manager = shared_data_manager;

  // fill sub and pub meta events.
  if (!_event_manager->get_event_meta(sub_events, &_sub_meta_events)) {
    AERROR << "failed to get Sub EventMeta. node: <" << _name << ", " << _id
           << ">";
    return false;
  }

  if (!_event_manager->get_event_meta(pub_events, &_pub_meta_events)) {
    AERROR << "failed to get Pub EventMeta. node: <" << _id << ", " << _name
           << ">";
    return false;
  }

  if (!init_internal()) {
    AERROR << "failed to init inner members.";
    return false;
  }

  _inited = true;
  return true;
}

void Subnode::run() {
  if (!_inited) {
    AERROR << "Subnode not inited, run failed. node: <" << _id << ", " << _name
           << ">";
    return;
  }

  if (_type == DAGConfig::SUBNODE_IN) {
    AINFO << "Subnode == SUBNODE_IN, EXIT THREAD. subnode:" << debug_string();
    return;
  }

  while (!_stop) {
    StatusCode status = proc_events();
    ++_total_count;
    if (status == FAIL) {
      ++_failed_count;
      AWARN << "Subnode: " << _name << " proc event failed. "
            << " total_count: " << _total_count
            << " failed_count: " << _failed_count;
      continue;
    }

    // FATAL error, so exit thread.
    if (status == FATAL) {
      AERROR << "Subnode: " << _name << " proc event FATAL error, EXIT. "
             << " total_count: " << _total_count
             << " failed_count: " << _failed_count;
      break;
    }
  }
}

string Subnode::debug_string() const {
  ostringstream oss;
  oss << "{id: " << _id << ", name: " << _name << ", reserve: " << _reserve
      << ", type:" << DAGConfig::SubnodeType_Name(_type);

  oss << ", SubEvents: [";
  for (size_t idx = 0; idx < _sub_meta_events.size(); ++idx) {
    oss << "<" << _sub_meta_events[idx].to_string() << ">";
  }

  oss << "], PubEvents: [";
  for (size_t idx = 0; idx < _pub_meta_events.size(); ++idx) {
    oss << "<" << _pub_meta_events[idx].to_string() << ">";
  }
  oss << "]}";

  return oss.str();
}

StatusCode CommonSubnode::proc_events() {
  CHECK(_sub_meta_events.size() == 1u) << "CommonSubnode sub_meta_events == 1";
  CHECK(_pub_meta_events.size() == 1u) << "CommonSubnode pub_meta_events == 1";

  Event sub_event;
  if (!_event_manager->subscribe(_sub_meta_events[0].event_id, &sub_event)) {
    AERROR << "failed to subscribe. meta_event: <"
           << _sub_meta_events[0].to_string() << ">";
    return FAIL;
  }

  Event pub_event = sub_event;
  pub_event.event_id = _pub_meta_events[0].event_id;

  // user defined logic api.
  if (!handle_event(sub_event, &pub_event)) {
    AWARN << "failed to call _handle_event. sub_event: <"
          << sub_event.to_string() << "> pub_event: <" << pub_event.to_string();
    return FAIL;
  }

  if (!_event_manager->publish(pub_event)) {
    AERROR << "failed to publish pub_event: <" << pub_event.to_string() << ">";
    return FAIL;
  }

  return SUCC;
}

}  // namespace perception
}  // namespace apollo
