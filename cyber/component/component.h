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

#ifndef CYBER_COMPONENT_COMPONENT_H_
#define CYBER_COMPONENT_COMPONENT_H_

#include <memory>
#include <utility>
#include <vector>

#include "cyber/base/macros.h"
#include "cyber/blocker/blocker_manager.h"
#include "cyber/common/global_data.h"
#include "cyber/common/types.h"
#include "cyber/common/util.h"
#include "cyber/component/component_base.h"
#include "cyber/croutine/routine_factory.h"
#include "cyber/data/data_visitor.h"
#include "cyber/scheduler/scheduler.h"

namespace apollo {
namespace cyber {

using apollo::cyber::common::GlobalData;
using apollo::cyber::proto::RoleAttributes;

/**
 * @brief .
 * The Component can process any number channels of messages. The message type
 * is specified when the component is created. The Component is inherited from
 * ComponentBase. Your component can inherit from Component, and implement
 * Init() & Proc(...), They are picked up by the CyberRT.
 * 
 * @tparam M0    first message type.
 * @tparam M...  other message types.
 * @warning The Init & Proc functions need to be overloaded, but don't want to
 * be called. They are called by the CyberRT Frame.
 *
 */
template <typename M0 = NullType, typename ...M>
class Component : public ComponentBase {
 public:
  Component() {}
  ~Component() override {}

  /**
   * @brief init the component by protobuf object.
   *
   * @param config which is defined in 'cyber/proto/component_conf.proto'
   *
   * @return returns true if successful, otherwise returns false
   */
  bool Initialize(const ComponentConfig& config) override {
    node_.reset(new Node(config.name()));
    LoadConfigFiles(config);

    if (config.readers_size() < (static_cast<int>(sizeof...(M))+1)) {
      AERROR << "Invalid config file: too few readers_." << std::endl;
      return false;
    }

    if (!Init()) {
      AERROR << "Component Init() failed." << std::endl;
      return false;
    }

    bool is_reality_mode = GlobalData::Instance()->IsRealityMode();


    if (cyber_likely(is_reality_mode)) {
      if (!CreateReaders(config, std::make_index_sequence<sizeof...(M)>{})) {
        AERROR << "Component create reader failed." << std::endl;
        return false;
      }
    } else {
      if (!CreateSimulatedReaders(config,
                                  std::make_index_sequence<sizeof...(M)>{})) {
        AERROR << "Component create reader failed." << std::endl;
        return false;
      }
      return true;
    }

    auto sched = scheduler::Instance();
    std::weak_ptr<Component<M0, M...>> self =
        std::dynamic_pointer_cast<Component<M0, M...>>(shared_from_this());
    auto func =
        [self](const std::shared_ptr<M0>& msg0,
               const std::shared_ptr<M>& ...msg) {
          auto ptr = self.lock();
          if (ptr) {
            ptr->Process(msg0, msg...);
          } else {
            AERROR << "Component object has been destroyed." << std::endl;
          }
        };

    std::vector<data::VisitorConfig> config_list;
    for (auto& reader : readers_) {
      config_list.emplace_back(reader->ChannelId(), reader->PendingQueueSize());
    }
    auto dv = std::make_shared<data::DataVisitor<M0, M...> >(config_list);
    croutine::RoutineFactory factory =
        croutine::CreateRoutineFactory(func, dv);
    return sched->CreateTask(factory, node_->Name());

    return true;
  }

  bool Process(const std::shared_ptr<M0> msg0,
               const std::shared_ptr<M> ...msg) {
    if (is_shutdown_.load()) {
      return true;
    }
    return Proc(msg0, msg...);
  }

 private:
  /**
   * @brief The process logical.
   *
   * @return returns true if successful, otherwise returns false
   */
  virtual bool Proc(const std::shared_ptr<M0>& msg0,
                    const std::shared_ptr<M>& ...msg) = 0;

  template<typename T>
  std::shared_ptr<ReaderBase> CreateReader(const ComponentConfig& config,
                                           size_t i) {
    ReaderConfig reader_cfg;
    reader_cfg.channel_name = config.readers(i).channel();
    reader_cfg.qos_profile.CopyFrom(config.readers(i).qos_profile());
    reader_cfg.pending_queue_size = config.readers(i).pending_queue_size();

    return node_->template CreateReader<T>(reader_cfg);
  }

  template<size_t ...N>
  bool CreateReaders(const ComponentConfig& config,
                     std::index_sequence<N...>) {
    readers_ = std::vector<std::shared_ptr<ReaderBase> >(
        {CreateReader<M0>(config, 0), CreateReader<M>(config, N+1)...});

    bool result = true;
    for (auto r : readers_) {
      result = result && r;
    }
    return result;
  }

  template<size_t ...N>
  bool CreateSimulatedReaders(const ComponentConfig& config,
                              std::index_sequence<N...>) {
    auto readers = std::vector<std::shared_ptr<ReaderBase> >(
          {CreateReader<M>(config, N+1)...});
    auto blockers = std::make_tuple(
          blocker::BlockerManager::Instance()->GetBlocker<M>(
              config.readers(N+1).channel())...);

    std::weak_ptr<Component<M0, M...>> self =
          std::dynamic_pointer_cast<Component<M0, M...>>(
              shared_from_this());
    auto func = [self, blockers](const std::shared_ptr<M0>& msg0) {
      auto ptr = self.lock();
      if (ptr) {
        std::vector<bool> isEmpty =
            {std::get<N>(blockers)->IsPublishedEmpty()...};
        bool hasEmpty =
            (isEmpty.end() != std::find(isEmpty.begin(), isEmpty.end(), true));
        if (!hasEmpty) {
          ptr->Process(msg0,
                       std::get<N>(blockers)->GetLatestPublishedPtr()...);
        }
      } else {
        AERROR << "Component object has been destroyed.";
      }
    };

    ReaderConfig reader_cfg;
    reader_cfg.channel_name = config.readers(0).channel();
    reader_cfg.qos_profile.CopyFrom(config.readers(0).qos_profile());
    reader_cfg.pending_queue_size = config.readers(0).pending_queue_size();
    auto reader0 = node_->template CreateReader<M0>(reader_cfg, func);

    readers_.push_back(reader0);
    readers_.insert(readers_.end(), readers.begin(), readers.end());


    bool result = true;
    for (auto r : readers_) {
      result = result && r;
    }
    return result;
  }
};

template <>
class Component<NullType> : public ComponentBase {
 public:
  Component() {}
  ~Component() override {}
  bool Initialize(const ComponentConfig& config) override;
};

inline bool Component<NullType>::Initialize(
    const ComponentConfig& config) {
  node_.reset(new Node(config.name()));
  LoadConfigFiles(config);
  if (!Init()) {
    AERROR << "Component Init() failed." << std::endl;
    return false;
  }
  return true;
}


#define CYBER_REGISTER_COMPONENT(name) \
  CLASS_LOADER_REGISTER_CLASS(name, apollo::cyber::ComponentBase)

}  // namespace cyber
}  // namespace apollo

#endif  // CYBER_COMPONENT_COMPONENT_H_
