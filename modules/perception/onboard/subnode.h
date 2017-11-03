#ifndef MODEULES_PERCEPTION_ONBOARD_SUBNODE_H_
#define MODEULES_PERCEPTION_ONBOARD_SUBNODE_H_

#include <stdio.h>
#include <unistd.h>

#include <string>
#include <vector>

#include "modules/common/macro.h"
#include "modules/perception/lib/base/registerer.h"
#include "modules/perception/lib/base/thread.h"
#include "modules/perception/onboard/proto/dag_config.pb.h"
#include "modules/perception/onboard/types.h"
#include "modules/perception/onboard/shared_data_manager.h"
#include "modules/perception/onboard/event_manager.h"

namespace apollo {
namespace perception {

class EventManager;
class SharedDataManager;

// Subnode virtual class, all business subnodes, including SubnodeIn and
// SubnodeOut,
// are derived this one.
class Subnode : public Thread {
 public:
  Subnode()
      : Thread(true),
        _id(0),
        _type(DAGConfig::SUBNODE_NORMAL),
        _event_manager(NULL),
        _shared_data_manager(NULL),
        _stop(false),
        _inited(false),
        _total_count(0),
        _failed_count(0) {}

  virtual ~Subnode() {}

  // @brief Initial DataMgr, RosIO and EventMgr.
  //        It is same for all the subnodes in one stream;
  // @return  bool
  // @retval
  virtual bool init(const DAGConfig::Subnode& config,
                    EventManager* event_manager,
                    SharedDataManager* shared_data_manager,
                    const std::vector<EventID>& sub_events,
                    const std::vector<EventID>& pub_events);

  void stop() {
    _stop = true;
  }

  // @brief Subnode process interface, should be realized in derived class.
  // @return  bool
  // @retval
  virtual StatusCode proc_events() = 0;

  SubnodeID id() const {
    return _id;
  }

  std::string name() const {
    return _name;
  }

  std::string reserve() const {
    return _reserve;
  }

  virtual std::string debug_string() const;

 protected:
  //@brief init the inner members ( default do nothing )
  //@return true/false
  virtual bool init_internal() {
    // do nothing.
    return true;
  }

  //@brief inner run
  virtual void run() override;

  // following variable can be accessed by Derived Class.
  SubnodeID _id;
  std::string _name;
  std::string _reserve;
  DAGConfig::SubnodeType _type;
  EventManager* _event_manager;
  SharedDataManager* _shared_data_manager;

  std::vector<EventMeta> _sub_meta_events;
  std::vector<EventMeta> _pub_meta_events;

 private:
  DISALLOW_COPY_AND_ASSIGN(Subnode);
  volatile bool _stop;
  bool _inited;
  int _total_count;
  int _failed_count;
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

  virtual StatusCode proc_events();

 protected:
  // Derive class implement this api.
  virtual bool handle_event(const Event& sub_event, Event* pub_event) = 0;

 private:
  DISALLOW_COPY_AND_ASSIGN(CommonSubnode);
};

// Just a sample, showing how subnode works.
// class SubnodeSample : public Subnode {
// public:
//     virtual StatusCode proc_events() {
//         // SubnodeNormal
//         _event_mgr->sub(EVENT_TYPE_A, event_a);
//         _data_mgr->get_data(data)
//         do something.
//         _data_mgr->set_data(data)
//         _event_mgr->pub(event_b);
//
//         //SubnodeIn
//         _ros_io->sub(Topic, message_a);
//         do something.
//         _data_mgr->set_data(data)
//         _event_mgr->pub(event_c);
//
//         //SubnodeOut
//         _event_mgr->sub(EVENT_TYPE_D, event_d);
//         _data_mgr->get_data(data)
//         do something.
//         _ros_io->pub(message_e);
//
//
//         printf("Process one event.\n");
//         sleep(1);
//
//         return SUCC;
//     };
// };

}  // namespace perception
}  // namespace apollo

#endif  // MODEULES_PERCEPTION_ONBOARD_SUBNODE_H_
