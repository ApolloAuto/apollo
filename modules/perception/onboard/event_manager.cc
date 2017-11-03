#include "modules/perception/onboard/event_manager.h"

#include <algorithm>
#include <string>
#include <vector>

#include <gflags/gflags.h>
#include "modules/common/log.h"

namespace apollo {
namespace perception {

using std::vector;
using std::string;

DEFINE_int32(max_event_queue_size, 1000, "The max size of event queue.");

bool EventManager::init(const DAGConfig::EdgeConfig& edge_config) {
  if (_inited) {
    AWARN << "EventManager init twice.";
    return true;
  }

  for (const DAGConfig::Edge& edge : edge_config.edges()) {
    for (const DAGConfig::Event event_pb : edge.events()) {
      if (_event_queue_map.find(event_pb.id()) != _event_queue_map.end()) {
        AERROR << "duplicate event id in config. id: " << event_pb.id();

        return false;
      }

      _event_queue_map[event_pb.id()].reset(
          new EventQueue(FLAGS_max_event_queue_size));

      EventMeta event_meta;
      event_meta.event_id = event_pb.id();
      event_meta.name = event_pb.name();
      event_meta.from_node = edge.from_node();
      event_meta.to_node = edge.to_node();
      _event_meta_map.emplace(event_pb.id(), event_meta);
      AINFO << "load EventMeta: " << event_meta.to_string();
    }
  }
  AINFO << "load " << _event_queue_map.size() << " events in DAGSreaming.";
  _inited = true;
  return true;
}

bool EventManager::publish(const Event& event) {
  EventQueue* queue = NULL;
  if (!get_event_queue(event.event_id, &queue)) {
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

  // TODO(Yangguang Li): add debug log.
  // XLOG(DEBUG) << ""

  return true;
}

bool EventManager::subscribe(EventID event_id, Event* event, bool nonblocking) {
  EventQueue* queue = NULL;
  if (!get_event_queue(event_id, &queue)) {
    return false;
  }

  if (nonblocking) {
    return queue->try_pop(event);
  }

  ADEBUG << "EVENT_ID: " << event_id << "QUEUE LENGTH:" << queue->size();
  queue->pop(event);
  return true;
}

bool EventManager::subscribe(EventID event_id, Event* event) {
  return subscribe(event_id, event, false);
}

bool EventManager::get_event_queue(EventID event_id, EventQueue** queue) {
  EventQueueMapIterator iter = _event_queue_map.find(event_id);
  if (iter == _event_queue_map.end()) {
    AERROR << "event: " << event_id << " not exist in EventQueueMap.";
    return false;
  }

  *queue = iter->second.get();
  CHECK(queue != NULL) << " event_id: " << event_id;
  return true;
}

bool EventManager::get_event_meta(EventID event_id,
                                  EventMeta* event_meta) const {
  EventMetaMapConstIterator citer = _event_meta_map.find(event_id);
  if (citer == _event_meta_map.end()) {
    AWARN << "event not found in EventManager. id: " << event_id;
    return false;
  }

  *event_meta = citer->second;
  return true;
}

bool EventManager::get_event_meta(const vector<EventID>& event_ids,
                                  vector<EventMeta>* event_metas) const {
  event_metas->reserve(event_ids.size());
  for (EventID event_id : event_ids) {
    EventMeta meta;
    if (!get_event_meta(event_id, &meta)) {
      return false;
    }
    event_metas->push_back(meta);
  }
  return true;
}

bool EventManager::get_event_meta(EventID event_id, string* str) const {
  EventMeta event_meta;
  if (!get_event_meta(event_id, &event_meta)) {
    return false;
  }
  *str = event_meta.to_string();
  return true;
}

bool EventManager::get_event_meta(const vector<EventID>& event_ids,
                                  vector<string>* str_list) const {
  str_list->reserve(event_ids.size());
  for (EventID event_id : event_ids) {
    string str;
    if (!get_event_meta(event_id, &str)) {
      return false;
    }
    str_list->push_back(str);
  }
  return true;
}

int EventManager::avg_len_of_event_queues() const {
  if (_event_queue_map.empty()) {
    return 0;
  }

  int total_length = 0;
  for (const auto& event : _event_queue_map) {
    total_length += event.second->size();
  }
  return total_length / _event_queue_map.size();
}

int EventManager::max_len_of_event_queues() const {
  int max_length = 0;
  for (const auto& event : _event_queue_map) {
    max_length = std::max(max_length, event.second->size());
  }
  return max_length;
}

void EventManager::reset() {
  EventQueueMapIterator iter = _event_queue_map.begin();
  for (; iter != _event_queue_map.end(); ++iter) {
    iter->second->clear();
  }
}

}  // namespace perception
}  // namespace apollo
