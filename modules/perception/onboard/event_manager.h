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

#ifndef MODULES_PERCEPTION_ONBOARD_EVENT_MANAGER_H_
#define MODULES_PERCEPTION_ONBOARD_EVENT_MANAGER_H_

#include <memory>
#include <sstream>
#include <string>
#include <unordered_map>
#include <vector>

#include "modules/perception/onboard/proto/dag_config.pb.h"

#include "modules/common/macro.h"
#include "modules/perception/onboard/types.h"

namespace apollo {
namespace perception {

class EventManager {
 public:
  EventManager() = default;
  ~EventManager() = default;

  // not thread-safe.
  bool Init(const DAGConfig::EdgeConfig &edge_config);

  // thread-safe.
  bool Publish(const Event &event);

  // if no event arrive, this api would be block.
  // thread-safe.
  bool Subscribe(EventID event_id, Event *event);

  bool Subscribe(EventID event_id, Event *event, bool nonblocking);

  // clear all the event queues.
  void Reset();

  int AvgLenOfEventQueues() const;

  int MaxLenOfEventQueues() const;

  bool GetEventMeta(EventID event_id, EventMeta *event_meta) const;

  bool GetEventMeta(const std::vector<EventID> &event_ids,
                    std::vector<EventMeta> *event_metas) const;

  bool GetEventMeta(EventID event_id, std::string *str) const;

  bool GetEventMeta(const std::vector<EventID> &event_id,
                    std::vector<std::string> *str_list) const;

  int NumEvents() const { return event_queue_map_.size(); }

 private:
  using EventQueue = FixedSizeConQueue<Event>;
  using EventQueueMap =
      std::unordered_map<EventID, std::unique_ptr<EventQueue>>;
  using EventQueueMapIterator = EventQueueMap::iterator;
  using EventQueueMapConstIterator = EventQueueMap::const_iterator;
  using EventMetaMap = std::unordered_map<EventID, EventMeta>;
  using EventMetaMapIterator = EventMetaMap::iterator;
  using EventMetaMapConstIterator = EventMetaMap::const_iterator;

  EventQueue *GetEventQueue(const EventID &event_id);

  EventQueueMap event_queue_map_;
  // for debug.
  EventMetaMap event_meta_map_;
  bool inited_ = false;

  DISALLOW_COPY_AND_ASSIGN(EventManager);
};

}  // namespace perception
}  // namespace apollo

#endif  // MODULES_PERCEPTION_ONBOARD_EVENT_MANAGER_H_
