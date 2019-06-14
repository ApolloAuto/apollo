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
 * The Component can process up to four channels of messages. The message type
 * is specified when the component is created. The Component is inherited from
 * ComponentBase. Your component can inherit from Component, and implement
 * Init() & Proc(...), They are picked up by the CyberRT. There are 4
 * specialization implementations.
 *
 * @tparam M0 the first message.
 * @tparam M1 the second message.
 * @tparam M2 the third message.
 * @tparam M3 the fourth message.
 * @warning The Init & Proc functions need to be overloaded, but don't want to
 * be called. They are called by the CyberRT Frame.
 *
 */
template <typename M0 = NullType, typename M1 = NullType,
          typename M2 = NullType, typename M3 = NullType>
class Component : public ComponentBase {
 public:
  Component() {}
  ~Component() override {}

  /**
   * @brief init the component by protobuf object.
   *
   * @param config which is define in 'cyber/proto/component_conf.proto'
   *
   * @return returns true if successful, otherwise returns false
   */
  bool Initialize(const ComponentConfig& config) override;
  bool Process(const std::shared_ptr<M0>& msg0, const std::shared_ptr<M1>& msg1,
               const std::shared_ptr<M2>& msg2,
               const std::shared_ptr<M3>& msg3);

 private:
  /**
   * @brief The process logical of yours.
   *
   * @param msg0 the first channel message.
   * @param msg1 the second channel message.
   * @param msg2 the third channel message.
   * @param msg3 the fourth channel message.
   *
   * @return returns true if successful, otherwise returns false
   */
  virtual bool Proc(const std::shared_ptr<M0>& msg0,
                    const std::shared_ptr<M1>& msg1,
                    const std::shared_ptr<M2>& msg2,
                    const std::shared_ptr<M3>& msg3) = 0;
};

template <>
class Component<NullType, NullType, NullType, NullType> : public ComponentBase {
 public:
  Component() {}
  ~Component() override {}
  bool Initialize(const ComponentConfig& config) override;
};

template <typename M0>
class Component<M0, NullType, NullType, NullType> : public ComponentBase {
 public:
  Component() {}
  ~Component() override {}
  bool Initialize(const ComponentConfig& config) override;
  bool Process(const std::shared_ptr<M0>& msg);

 private:
  virtual bool Proc(const std::shared_ptr<M0>& msg) = 0;
};

template <typename M0, typename M1>
class Component<M0, M1, NullType, NullType> : public ComponentBase {
 public:
  Component() {}
  ~Component() override {}
  bool Initialize(const ComponentConfig& config) override;
  bool Process(const std::shared_ptr<M0>& msg0,
               const std::shared_ptr<M1>& msg1);

 private:
  virtual bool Proc(const std::shared_ptr<M0>& msg,
                    const std::shared_ptr<M1>& msg1) = 0;
};

template <typename M0, typename M1, typename M2>
class Component<M0, M1, M2, NullType> : public ComponentBase {
 public:
  Component() {}
  ~Component() override {}
  bool Initialize(const ComponentConfig& config) override;
  bool Process(const std::shared_ptr<M0>& msg0, const std::shared_ptr<M1>& msg1,
               const std::shared_ptr<M2>& msg2);

 private:
  virtual bool Proc(const std::shared_ptr<M0>& msg,
                    const std::shared_ptr<M1>& msg1,
                    const std::shared_ptr<M2>& msg2) = 0;
};

template <typename M0>
bool Component<M0, NullType, NullType, NullType>::Process(
    const std::shared_ptr<M0>& msg) {
  if (is_shutdown_.load()) {
    return true;
  }
  return Proc(msg);
}

inline bool Component<NullType, NullType, NullType>::Initialize(
    const ComponentConfig& config) {
  node_.reset(new Node(config.name()));
  LoadConfigFiles(config);
  if (!Init()) {
    AERROR << "Component Init() failed." << std::endl;
    return false;
  }
  return true;
}

template <typename M0>
bool Component<M0, NullType, NullType, NullType>::Initialize(
    const ComponentConfig& config) {
  node_.reset(new Node(config.name()));
  LoadConfigFiles(config);

  if (config.readers_size() < 1) {
    AERROR << "Invalid config file: too few readers.";
    return false;
  }

  if (!Init()) {
    AERROR << "Component Init() failed.";
    return false;
  }

  bool is_reality_mode = GlobalData::Instance()->IsRealityMode();

  ReaderConfig reader_cfg;
  reader_cfg.channel_name = config.readers(0).channel();
  reader_cfg.qos_profile.CopyFrom(config.readers(0).qos_profile());
  reader_cfg.pending_queue_size = config.readers(0).pending_queue_size();

  std::weak_ptr<Component<M0>> self =
      std::dynamic_pointer_cast<Component<M0>>(shared_from_this());
  auto func = [self](const std::shared_ptr<M0>& msg) {
    auto ptr = self.lock();
    if (ptr) {
      ptr->Process(msg);
    } else {
      AERROR << "Component object has been destroyed.";
    }
  };

  std::shared_ptr<Reader<M0>> reader = nullptr;

  if (likely(is_reality_mode)) {
    reader = node_->CreateReader<M0>(reader_cfg);
  } else {
    reader = node_->CreateReader<M0>(reader_cfg, func);
  }

  if (reader == nullptr) {
    AERROR << "Component create reader failed.";
    return false;
  }
  readers_.emplace_back(std::move(reader));

  if (unlikely(!is_reality_mode)) {
    return true;
  }

  data::VisitorConfig conf = {readers_[0]->ChannelId(),
                              readers_[0]->PendingQueueSize()};
  auto dv = std::make_shared<data::DataVisitor<M0>>(conf);
  croutine::RoutineFactory factory =
      croutine::CreateRoutineFactory<M0>(func, dv);
  auto sched = scheduler::Instance();
  return sched->CreateTask(factory, node_->Name());
}

template <typename M0, typename M1>
bool Component<M0, M1, NullType, NullType>::Process(
    const std::shared_ptr<M0>& msg0, const std::shared_ptr<M1>& msg1) {
  if (is_shutdown_.load()) {
    return true;
  }
  return Proc(msg0, msg1);
}

template <typename M0, typename M1>
bool Component<M0, M1, NullType, NullType>::Initialize(
    const ComponentConfig& config) {
  node_.reset(new Node(config.name()));
  LoadConfigFiles(config);

  if (config.readers_size() < 2) {
    AERROR << "Invalid config file: too few readers.";
    return false;
  }

  if (!Init()) {
    AERROR << "Component Init() failed.";
    return false;
  }

  bool is_reality_mode = GlobalData::Instance()->IsRealityMode();

  ReaderConfig reader_cfg;
  reader_cfg.channel_name = config.readers(1).channel();
  reader_cfg.qos_profile.CopyFrom(config.readers(1).qos_profile());
  reader_cfg.pending_queue_size = config.readers(1).pending_queue_size();

  auto reader1 = node_->template CreateReader<M1>(reader_cfg);

  reader_cfg.channel_name = config.readers(0).channel();
  reader_cfg.qos_profile.CopyFrom(config.readers(0).qos_profile());
  reader_cfg.pending_queue_size = config.readers(0).pending_queue_size();

  std::shared_ptr<Reader<M0>> reader0 = nullptr;
  if (likely(is_reality_mode)) {
    reader0 = node_->template CreateReader<M0>(reader_cfg);
  } else {
    std::weak_ptr<Component<M0, M1>> self =
        std::dynamic_pointer_cast<Component<M0, M1>>(shared_from_this());

    auto blocker1 = blocker::BlockerManager::Instance()->GetBlocker<M1>(
        config.readers(1).channel());

    auto func = [self, blocker1](const std::shared_ptr<M0>& msg0) {
      auto ptr = self.lock();
      if (ptr) {
        if (!blocker1->IsPublishedEmpty()) {
          auto msg1 = blocker1->GetLatestPublishedPtr();
          ptr->Process(msg0, msg1);
        }
      } else {
        AERROR << "Component object has been destroyed.";
      }
    };

    reader0 = node_->template CreateReader<M0>(reader_cfg, func);
  }
  if (reader0 == nullptr || reader1 == nullptr) {
    AERROR << "Component create reader failed.";
    return false;
  }
  readers_.push_back(std::move(reader0));
  readers_.push_back(std::move(reader1));

  if (unlikely(!is_reality_mode)) {
    return true;
  }

  auto sched = scheduler::Instance();
  std::weak_ptr<Component<M0, M1>> self =
      std::dynamic_pointer_cast<Component<M0, M1>>(shared_from_this());
  auto func = [self](const std::shared_ptr<M0>& msg0,
                     const std::shared_ptr<M1>& msg1) {
    auto ptr = self.lock();
    if (ptr) {
      ptr->Process(msg0, msg1);
    } else {
      AERROR << "Component object has been destroyed.";
    }
  };

  std::vector<data::VisitorConfig> config_list;
  for (auto& reader : readers_) {
    config_list.emplace_back(reader->ChannelId(), reader->PendingQueueSize());
  }
  auto dv = std::make_shared<data::DataVisitor<M0, M1>>(config_list);
  croutine::RoutineFactory factory =
      croutine::CreateRoutineFactory<M0, M1>(func, dv);
  return sched->CreateTask(factory, node_->Name());
}

template <typename M0, typename M1, typename M2>
bool Component<M0, M1, M2, NullType>::Process(const std::shared_ptr<M0>& msg0,
                                              const std::shared_ptr<M1>& msg1,
                                              const std::shared_ptr<M2>& msg2) {
  if (is_shutdown_.load()) {
    return true;
  }
  return Proc(msg0, msg1, msg2);
}

template <typename M0, typename M1, typename M2>
bool Component<M0, M1, M2, NullType>::Initialize(
    const ComponentConfig& config) {
  node_.reset(new Node(config.name()));
  LoadConfigFiles(config);

  if (config.readers_size() < 3) {
    AERROR << "Invalid config file: too few readers.";
    return false;
  }

  if (!Init()) {
    AERROR << "Component Init() failed.";
    return false;
  }

  bool is_reality_mode = GlobalData::Instance()->IsRealityMode();

  ReaderConfig reader_cfg;
  reader_cfg.channel_name = config.readers(1).channel();
  reader_cfg.qos_profile.CopyFrom(config.readers(1).qos_profile());
  reader_cfg.pending_queue_size = config.readers(1).pending_queue_size();

  auto reader1 = node_->template CreateReader<M1>(reader_cfg);

  reader_cfg.channel_name = config.readers(2).channel();
  reader_cfg.qos_profile.CopyFrom(config.readers(2).qos_profile());
  reader_cfg.pending_queue_size = config.readers(2).pending_queue_size();

  auto reader2 = node_->template CreateReader<M2>(reader_cfg);

  reader_cfg.channel_name = config.readers(0).channel();
  reader_cfg.qos_profile.CopyFrom(config.readers(0).qos_profile());
  reader_cfg.pending_queue_size = config.readers(0).pending_queue_size();
  std::shared_ptr<Reader<M0>> reader0 = nullptr;
  if (likely(is_reality_mode)) {
    reader0 = node_->template CreateReader<M0>(reader_cfg);
  } else {
    std::weak_ptr<Component<M0, M1, M2, NullType>> self =
        std::dynamic_pointer_cast<Component<M0, M1, M2, NullType>>(
            shared_from_this());

    auto blocker1 = blocker::BlockerManager::Instance()->GetBlocker<M1>(
        config.readers(1).channel());
    auto blocker2 = blocker::BlockerManager::Instance()->GetBlocker<M2>(
        config.readers(2).channel());

    auto func = [self, blocker1, blocker2](const std::shared_ptr<M0>& msg0) {
      auto ptr = self.lock();
      if (ptr) {
        if (!blocker1->IsPublishedEmpty() && !blocker2->IsPublishedEmpty()) {
          auto msg1 = blocker1->GetLatestPublishedPtr();
          auto msg2 = blocker2->GetLatestPublishedPtr();
          ptr->Process(msg0, msg1, msg2);
        }
      } else {
        AERROR << "Component object has been destroyed.";
      }
    };

    reader0 = node_->template CreateReader<M0>(reader_cfg, func);
  }

  if (reader0 == nullptr || reader1 == nullptr || reader2 == nullptr) {
    AERROR << "Component create reader failed.";
    return false;
  }
  readers_.push_back(std::move(reader0));
  readers_.push_back(std::move(reader1));
  readers_.push_back(std::move(reader2));

  if (unlikely(!is_reality_mode)) {
    return true;
  }

  auto sched = scheduler::Instance();
  std::weak_ptr<Component<M0, M1, M2, NullType>> self =
      std::dynamic_pointer_cast<Component<M0, M1, M2, NullType>>(
          shared_from_this());
  auto func = [self](const std::shared_ptr<M0>& msg0,
                     const std::shared_ptr<M1>& msg1,
                     const std::shared_ptr<M2>& msg2) {
    auto ptr = self.lock();
    if (ptr) {
      ptr->Process(msg0, msg1, msg2);
    } else {
      AERROR << "Component object has been destroyed.";
    }
  };

  std::vector<data::VisitorConfig> config_list;
  for (auto& reader : readers_) {
    config_list.emplace_back(reader->ChannelId(), reader->PendingQueueSize());
  }
  auto dv = std::make_shared<data::DataVisitor<M0, M1, M2>>(config_list);
  croutine::RoutineFactory factory =
      croutine::CreateRoutineFactory<M0, M1, M2>(func, dv);
  return sched->CreateTask(factory, node_->Name());
}

template <typename M0, typename M1, typename M2, typename M3>
bool Component<M0, M1, M2, M3>::Process(const std::shared_ptr<M0>& msg0,
                                        const std::shared_ptr<M1>& msg1,
                                        const std::shared_ptr<M2>& msg2,
                                        const std::shared_ptr<M3>& msg3) {
  if (is_shutdown_.load()) {
    return true;
  }
  return Proc(msg0, msg1, msg2, msg3);
}

template <typename M0, typename M1, typename M2, typename M3>
bool Component<M0, M1, M2, M3>::Initialize(const ComponentConfig& config) {
  node_.reset(new Node(config.name()));
  LoadConfigFiles(config);

  if (config.readers_size() < 4) {
    AERROR << "Invalid config file: too few readers_." << std::endl;
    return false;
  }

  if (!Init()) {
    AERROR << "Component Init() failed." << std::endl;
    return false;
  }

  bool is_reality_mode = GlobalData::Instance()->IsRealityMode();

  ReaderConfig reader_cfg;
  reader_cfg.channel_name = config.readers(1).channel();
  reader_cfg.qos_profile.CopyFrom(config.readers(1).qos_profile());
  reader_cfg.pending_queue_size = config.readers(1).pending_queue_size();

  auto reader1 = node_->template CreateReader<M1>(reader_cfg);

  reader_cfg.channel_name = config.readers(2).channel();
  reader_cfg.qos_profile.CopyFrom(config.readers(2).qos_profile());
  reader_cfg.pending_queue_size = config.readers(2).pending_queue_size();

  auto reader2 = node_->template CreateReader<M2>(reader_cfg);

  reader_cfg.channel_name = config.readers(3).channel();
  reader_cfg.qos_profile.CopyFrom(config.readers(3).qos_profile());
  reader_cfg.pending_queue_size = config.readers(3).pending_queue_size();

  auto reader3 = node_->template CreateReader<M3>(reader_cfg);

  reader_cfg.channel_name = config.readers(0).channel();
  reader_cfg.qos_profile.CopyFrom(config.readers(0).qos_profile());
  reader_cfg.pending_queue_size = config.readers(0).pending_queue_size();

  std::shared_ptr<Reader<M0>> reader0 = nullptr;
  if (likely(is_reality_mode)) {
    reader0 = node_->template CreateReader<M0>(reader_cfg);
  } else {
    std::weak_ptr<Component<M0, M1, M2, M3>> self =
        std::dynamic_pointer_cast<Component<M0, M1, M2, M3>>(
            shared_from_this());

    auto blocker1 = blocker::BlockerManager::Instance()->GetBlocker<M1>(
        config.readers(1).channel());
    auto blocker2 = blocker::BlockerManager::Instance()->GetBlocker<M2>(
        config.readers(2).channel());
    auto blocker3 = blocker::BlockerManager::Instance()->GetBlocker<M3>(
        config.readers(3).channel());

    auto func = [self, blocker1, blocker2,
                 blocker3](const std::shared_ptr<M0>& msg0) {
      auto ptr = self.lock();
      if (ptr) {
        if (!blocker1->IsPublishedEmpty() && !blocker2->IsPublishedEmpty() &&
            !blocker3->IsPublishedEmpty()) {
          auto msg1 = blocker1->GetLatestPublishedPtr();
          auto msg2 = blocker2->GetLatestPublishedPtr();
          auto msg3 = blocker3->GetLatestPublishedPtr();
          ptr->Process(msg0, msg1, msg2, msg3);
        }
      } else {
        AERROR << "Component object has been destroyed.";
      }
    };

    reader0 = node_->template CreateReader<M0>(reader_cfg, func);
  }

  if (reader0 == nullptr || reader1 == nullptr || reader2 == nullptr ||
      reader3 == nullptr) {
    AERROR << "Component create reader failed." << std::endl;
    return false;
  }
  readers_.push_back(std::move(reader0));
  readers_.push_back(std::move(reader1));
  readers_.push_back(std::move(reader2));
  readers_.push_back(std::move(reader3));

  if (unlikely(!is_reality_mode)) {
    return true;
  }

  auto sched = scheduler::Instance();
  std::weak_ptr<Component<M0, M1, M2, M3>> self =
      std::dynamic_pointer_cast<Component<M0, M1, M2, M3>>(shared_from_this());
  auto func =
      [self](const std::shared_ptr<M0>& msg0, const std::shared_ptr<M1>& msg1,
             const std::shared_ptr<M2>& msg2, const std::shared_ptr<M3>& msg3) {
        auto ptr = self.lock();
        if (ptr) {
          ptr->Process(msg0, msg1, msg2, msg3);
        } else {
          AERROR << "Component object has been destroyed." << std::endl;
        }
      };

  std::vector<data::VisitorConfig> config_list;
  for (auto& reader : readers_) {
    config_list.emplace_back(reader->ChannelId(), reader->PendingQueueSize());
  }
  auto dv = std::make_shared<data::DataVisitor<M0, M1, M2, M3>>(config_list);
  croutine::RoutineFactory factory =
      croutine::CreateRoutineFactory<M0, M1, M2, M3>(func, dv);
  return sched->CreateTask(factory, node_->Name());
}

#define CYBER_REGISTER_COMPONENT(name) \
  CLASS_LOADER_REGISTER_CLASS(name, apollo::cyber::ComponentBase)

}  // namespace cyber
}  // namespace apollo

#endif  // CYBER_COMPONENT_COMPONENT_H_
