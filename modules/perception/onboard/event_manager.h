#ifndef apollo_PERCEPTION_ONBOARD_EVENT_MANAGER_H
#define apollo_PERCEPTION_ONBOARD_EVENT_MANAGER_H

#include <memory>
#include <sstream>
#include <unordered_map>
#include <vector>

#include "modules/common/macro.h"
#include "modules/perception/onboard/proto/dag_config.pb.h"
#include "modules/perception/onboard/types.h"

namespace apollo {
namespace perception {

class EventManager {
 public:
  EventManager() = default;
  ~EventManager() = default;

  // not thread-safe.
  bool init(const DAGConfig::EdgeConfig& edge_config);

  // thread-safe.
  bool publish(const Event& event);

  // if no event arrive, this api would be block.
  // thread-safe.
  bool subscribe(EventID event_id, Event* event);

  bool subscribe(EventID event_id, Event* event, bool nonblocking);

  // clear all the event queues.
  void reset();
  int avg_len_of_event_queues() const;
  int max_len_of_event_queues() const;

  bool get_event_meta(EventID event_id, EventMeta* event_meta) const;
  bool get_event_meta(const std::vector<EventID>& event_ids,
                      std::vector<EventMeta>* event_metas) const;

  bool get_event_meta(EventID event_id, std::string* str) const;
  bool get_event_meta(const std::vector<EventID>& event_id,
                      std::vector<std::string>* str_list) const;

  int num_events() const {
    return _event_queue_map.size();
  }

 private:
  using EventQueue = FixedSizeConQueue<Event>;
  using EventQueueMap =
      std::unordered_map<EventID, std::unique_ptr<EventQueue>>;
  using EventQueueMapIterator = EventQueueMap::iterator;
  using EventQueueMapConstIterator = EventQueueMap::const_iterator;
  using EventMetaMap = std::unordered_map<EventID, EventMeta>;
  using EventMetaMapIterator = EventMetaMap::iterator;
  using EventMetaMapConstIterator = EventMetaMap::const_iterator;

  bool get_event_queue(EventID event_id, EventQueue** queue);

  EventQueueMap _event_queue_map;
  // for debug.
  EventMetaMap _event_meta_map;
  bool _inited = false;

  DISALLOW_COPY_AND_ASSIGN(EventManager);
};

}  // namespace perception
}  // namespace apollo

#endif  // apollo_PERCEPTION_ONBOARD_EVENT_MANAGER_H
