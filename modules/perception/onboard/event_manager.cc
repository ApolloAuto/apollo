/******************************************************************************
 * Copyright 2017 The Apollo Authors. All Rights Reserved.
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

#include "modules/perception/onboard/event_manager.h"

#include <algorithm>

#include "gflags/gflags.h"
#include "modules/common/log.h"

namespace apollo {
namespace perception {

using std::vector;
using std::string;

DEFINE_int32(max_event_queue_size, 1000, "The max size of event queue.");

bool EventManager::Init(const DAGConfig::EdgeConfig &edge_config) {
  if (inited_) {
    AWARN << "EventManager Init twice.";
    return true;
  }

  for (const DAGConfig::Edge &edge : edge_config.edges()) {
    for (const DAGConfig::Event event_pb : edge.events()) {
      if (event_queue_map_.find(event_pb.id()) != event_queue_map_.end()) {
        AERROR << "duplicate event id in config. id: " << event_pb.id();
        return false;
      }

      event_queue_map_[event_pb.id()].reset(
          new EventQueue(FLAGS_max_event_queue_size));

      EventMeta event_meta;
      event_meta.event_id = event_pb.id();
      event_meta.name = event_pb.name();
      event_meta.from_node = edge.from_node();
      event_meta.to_node = edge.to_node();
      event_meta_map_.emplace(event_pb.id(), event_meta);
      AINFO << "load EventMeta: " << event_meta.to_string();
    }
  }
  AINFO << "load " << event_queue_map_.size() << " events in DAGSreaming.";
  inited_ = true;
  return true;
}

bool EventManager::Publish(const Event &event) {
  EventQueue *queue = GetEventQueue(event.event_id);
  if (queue == nullptr) {
    AERROR << "Fail to get event with id: " << event.event_id;
    return false;
  }

  if (!queue->try_push(event)) {
    // Critical errors: queue is full.
    AERROR << "EventQueue is FULL. id: " << event.event_id;
    // Clear all blocked data.
    AERROR << "clear EventQueue. id: " << event.event_id
           << " size: " << queue->size();
    queue->clear();

    // try second time.
    queue->try_push(event);
  }
  return true;
}

bool EventManager::Subscribe(EventID event_id, Event *event, bool nonblocking) {
  EventQueue *queue = GetEventQueue(event_id);
  if (queue == nullptr) {
    AERROR << "Fail to get event with id: " << event_id;
    return false;
  }

  if (nonblocking) {
    return queue->try_pop(event);
  }

  ADEBUG << "EVENT_ID: " << event_id << "QUEUE LENGTH:" << queue->size();
  queue->pop(event);
  return true;
}

bool EventManager::Subscribe(EventID event_id, Event *event) {
  return Subscribe(event_id, event, false);
}

EventManager::EventQueue *EventManager::GetEventQueue(const EventID &event_id) {
  EventQueueMapIterator iter = event_queue_map_.find(event_id);
  if (iter == event_queue_map_.end()) {
    AERROR << "event: " << event_id << " not exist in EventQueueMap.";
    return nullptr;
  }
  return iter->second.get();
}

bool EventManager::GetEventMeta(EventID event_id, EventMeta *event_meta) const {
  EventMetaMapConstIterator citer = event_meta_map_.find(event_id);
  if (citer == event_meta_map_.end()) {
    AWARN << "event not found in EventManager. id: " << event_id;
    return false;
  }

  *event_meta = citer->second;
  return true;
}

bool EventManager::GetEventMeta(const vector<EventID> &event_ids,
                                vector<EventMeta> *event_metas) const {
  event_metas->reserve(event_ids.size());
  for (EventID event_id : event_ids) {
    EventMeta meta;
    if (!GetEventMeta(event_id, &meta)) {
      return false;
    }
    event_metas->push_back(meta);
  }
  return true;
}

bool EventManager::GetEventMeta(EventID event_id, string *str) const {
  EventMeta event_meta;
  if (!GetEventMeta(event_id, &event_meta)) {
    return false;
  }
  *str = event_meta.to_string();
  return true;
}

bool EventManager::GetEventMeta(const vector<EventID> &event_ids,
                                vector<string> *str_list) const {
  str_list->reserve(event_ids.size());
  for (EventID event_id : event_ids) {
    string str;
    if (!GetEventMeta(event_id, &str)) {
      return false;
    }
    str_list->push_back(str);
  }
  return true;
}

int EventManager::AvgLenOfEventQueues() const {
  if (event_queue_map_.empty()) {
    return 0;
  }

  int total_length = 0;
  for (const auto &event : event_queue_map_) {
    total_length += event.second->size();
  }
  return total_length / event_queue_map_.size();
}

int EventManager::MaxLenOfEventQueues() const {
  int max_length = 0;
  for (const auto &event : event_queue_map_) {
    max_length = std::max(max_length, event.second->size());
  }
  return max_length;
}

void EventManager::Reset() {
  EventQueueMapIterator iter = event_queue_map_.begin();
  for (; iter != event_queue_map_.end(); ++iter) {
    iter->second->clear();
  }
}

}  // namespace perception
}  // namespace apollo
