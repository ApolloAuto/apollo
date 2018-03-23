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

#ifndef MODULES_PERCEPTION_ONBOARD_SUBNODE_H_
#define MODULES_PERCEPTION_ONBOARD_SUBNODE_H_

#include <unistd.h>
#include <cstdio>
#include <string>
#include <vector>

#include "modules/perception/onboard/proto/dag_config.pb.h"

#include "modules/common/macro.h"
#include "modules/common/status/status.h"
#include "modules/perception/lib/base/registerer.h"
#include "modules/perception/lib/base/thread.h"
#include "modules/perception/onboard/event_manager.h"
#include "modules/perception/onboard/shared_data_manager.h"
#include "modules/perception/onboard/types.h"

namespace apollo {
namespace perception {

class EventManager;
class SharedDataManager;

// @brief Subnode virtual class, all business subnodes, including SubnodeIn and
//        SubnodeOut, are derived this one.
class Subnode : public Thread {
 public:
  Subnode() : Thread(true) {}

  virtual ~Subnode() {}

  // @brief Initial DataMgr, RosIO and EventMgr.
  //        It is same for all the subnodes in one stream;
  // @return  bool
  // @retval
  virtual bool Init(const DAGConfig::Subnode &config,
                    const std::vector<EventID> &sub_events,
                    const std::vector<EventID> &pub_events,
                    EventManager *event_manager,
                    SharedDataManager *shared_data_manager);

  void Stop() { stop_ = true; }

  // @brief Subnode process interface, should be realized in derived class.
  // @return Status.
  virtual apollo::common::Status ProcEvents() = 0;

  SubnodeID id() const { return id_; }

  std::string name() const { return name_; }

  std::string reserve() const { return reserve_; }

  virtual std::string DebugString() const;

 protected:
  // @brief init the inner members ( default do nothing )
  // @return true/false
  virtual bool InitInternal() {
    // do nothing.
    return true;
  }

  // @brief inner run
  void Run() override;

  // following variable can be accessed by Derived Class.
  SubnodeID id_ = 0;
  std::string name_;
  std::string reserve_;
  DAGConfig::SubnodeType type_ = DAGConfig::SUBNODE_NORMAL;
  EventManager *event_manager_ = nullptr;
  SharedDataManager *shared_data_manager_ = nullptr;

  std::vector<EventMeta> sub_meta_events_;
  std::vector<EventMeta> pub_meta_events_;

 private:
  volatile bool stop_ = false;
  bool inited_ = false;
  int total_count_ = 0;
  int failed_count_ = 0;
  DISALLOW_COPY_AND_ASSIGN(Subnode);
};

REGISTER_REGISTERER(Subnode);

#define REGISTER_SUBNODE(name) REGISTER_CLASS(Subnode, name)

// common subnode, subscribe one event, and publish one evnet,
// we implement the sub event and pub event details, you can only to
// implement the handle_event().
class CommonSubnode : public Subnode {
 public:
  CommonSubnode() : Subnode() {}
  virtual ~CommonSubnode() {}

  virtual apollo::common::Status ProcEvents();

 protected:
  // Derive class implement this api.
  virtual bool HandleEvent(const Event &sub_event, Event *pub_event) = 0;

 private:
  DISALLOW_COPY_AND_ASSIGN(CommonSubnode);
};

// Just a sample, showing how subnode works.
// class SubnodeSample : public Subnode {
// public:
//     virtual apollo::common::Status proc_events() {
//         // SubnodeNormal
//         event_mgr_->sub(EVENT_TYPE_A, event_a);
//         data_mgr_->get_data(data)
//         do something.
//         data_mgr_->set_data(data)
//         event_mgr_->pub(event_b);
//
//         //SubnodeIn
//         ros_io_->sub(Topic, message_a);
//         do something.
//         data_mgr_->set_data(data)
//         event_mgr_->pub(event_c);
//
//         //SubnodeOut
//         event_mgr_->sub(EVENT_TYPE_D, event_d);
//         data_mgr_->get_data(data)
//         do something.
//         ros_io_->pub(message_e);
//
//
//         printf("Process one event.\n");
//         sleep(1);
//
//         return apollo::common::Status::OK();
//     };
// };

}  // namespace perception
}  // namespace apollo

#endif  // MODULES_PERCEPTION_ONBOARD_SUBNODE_H_
