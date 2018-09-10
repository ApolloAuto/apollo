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

#ifndef CYBERTRON_COMPONENT_COMPONENT_H_
#define CYBERTRON_COMPONENT_COMPONENT_H_

#include <memory>
#include <vector>

#include "cybertron/common/global_data.h"
#include "cybertron/common/util.h"
#include "cybertron/component/component_base.h"
#include "cybertron/croutine/routine_factory.h"
#include "cybertron/data/data_visitor.h"
#include "cybertron/scheduler/scheduler.h"

namespace apollo {
namespace cybertron {

using apollo::cybertron::common::GlobalData;
using apollo::cybertron::proto::RoleAttributes;

struct NullType {};

template <typename M0 = NullType, typename M1 = NullType,
          typename M2 = NullType>
class Component : public ComponentBase {
 public:
  Component() {}
  ~Component() override {}
  // TODO(hewei03): Why do we need Initialize() function? RAII is always a good
  // choice.
  bool Initialize(const ComponentConfig& config) override;
  bool Process(const std::shared_ptr<M0>& msg0, const std::shared_ptr<M1>& msg1,
               const std::shared_ptr<M2>& msg2);

 protected:
  std::shared_ptr<Node> node_ = nullptr;

 private:
  // User interface.
  virtual bool Proc(const std::shared_ptr<M0>& msg0,
                    const std::shared_ptr<M1>& msg1,
                    const std::shared_ptr<M2>& msg2) = 0;
};

template <>
class Component<NullType, NullType, NullType> : public ComponentBase {
 public:
  Component() {}
  ~Component() override {}
  bool Initialize(const ComponentConfig& config) override;

 protected:
  std::shared_ptr<Node> node_ = nullptr;
};

template <typename M0>
class Component<M0, NullType, NullType> : public ComponentBase {
 public:
  Component() {}
  ~Component() override {}
  bool Initialize(const ComponentConfig& config) override;
  bool Process(const std::shared_ptr<M0>& msg);

 protected:
  std::shared_ptr<Node> node_ = nullptr;

 private:
  virtual bool Proc(const std::shared_ptr<M0>& msg) = 0;
};

template <typename M0, typename M1>
class Component<M0, M1, NullType> : public ComponentBase {
 public:
  Component() {}
  ~Component() override {}
  bool Initialize(const ComponentConfig& config) override;
  bool Process(const std::shared_ptr<M0>& msg0,
               const std::shared_ptr<M1>& msg1);

 protected:
  std::shared_ptr<Node> node_ = nullptr;
  /*
  std::shared_ptr<Reader<M0>> reader0_ = nullptr;
  std::shared_ptr<Reader<M1>> reader1_ = nullptr;
  */

 private:
  virtual bool Proc(const std::shared_ptr<M0>& msg,
                    const std::shared_ptr<M1>& msg1) = 0;
};

inline bool Component<NullType, NullType, NullType>::Initialize(
    const ComponentConfig& config) {
  node_.reset(new Node(config.name()));
  SetConfigFilePath(config);

  if (!Init()) {
    AERROR << "Component Init() failed." << std::endl;
    return false;
  }

  return true;
}

template <typename M0>
bool Component<M0, NullType, NullType>::Process(
    const std::shared_ptr<M0>& msg) {
  // TODO(hewei03): Add some protection here.
  return Proc(msg);
}

template <typename M0>
bool Component<M0, NullType, NullType>::Initialize(
    const ComponentConfig& config) {
  node_.reset(new Node(config.name()));
  SetConfigFilePath(config);

  if (config.readers_size() < 1) {
    AERROR << "Invalid config file: too few readers." << std::endl;
    return false;
  }

  if (!Init()) {
    AERROR << "Component Init() failed." << std::endl;
    return false;
  }
  RoleAttributes attr;
  attr.set_node_name(config.name());
  attr.set_channel_name(config.readers(0).channel());
  auto qos_profile = attr.mutable_qos_profile();
  *qos_profile = config.readers(0).qos_profile();
  auto reader = node_->CreateReader<M0>(attr);
  if (reader == nullptr) {
    AERROR << "Component create reader failed." << std::endl;
    return false;
  }
  readers.push_back(std::move(reader));

  auto sched = scheduler::Scheduler::Instance();
  std::weak_ptr<Component<M0>> self =
      std::dynamic_pointer_cast<Component<M0>>(shared_from_this());
  auto func = [self](const std::shared_ptr<M0>& msg) {
    auto ptr = self.lock();
    if (ptr) {
      ptr->Process(msg);
    } else {
      AERROR << "Component object has been destroyed." << std::endl;
    }
  };
  std::shared_ptr<data::DataVisitor> dv;
  std::vector<uint64_t> channel_vec;
  channel_vec.emplace_back(common::Hash(config.readers(0).channel()));
  dv = std::make_shared<data::DataVisitor>(
      std::move(channel_vec), config.readers(0).qos_profile().depth());
  croutine::RoutineFactory factory =
      croutine::CreateRoutineFactory<M0>(func, dv);
  return sched->CreateTask(factory, node_->Name());
}

template <typename M0, typename M1>
bool Component<M0, M1, NullType>::Process(const std::shared_ptr<M0>& msg0,
                                          const std::shared_ptr<M1>& msg1) {
  return Proc(msg0, msg1);
}

template <typename M0, typename M1>
bool Component<M0, M1, NullType>::Initialize(const ComponentConfig& config) {
  node_.reset(new Node(config.name()));
  SetConfigFilePath(config);

  if (config.readers_size() < 2) {
    AERROR << "Invalid config file: too few readers." << std::endl;
    return false;
  }

  if (!Init()) {
    AERROR << "Component Init() failed." << std::endl;
    return false;
  }

  RoleAttributes attr;
  attr.set_node_name(config.name());
  attr.set_channel_name(config.readers(0).channel());
  auto reader0 = node_->template CreateReader<M0>(attr);
  attr.set_channel_name(config.readers(1).channel());
  auto reader1 = node_->template CreateReader<M1>(attr);
  if (reader0 == nullptr || reader1 == nullptr) {
    AERROR << "Component create reader failed." << std::endl;
    return false;
  }
  readers.push_back(std::move(reader0));
  readers.push_back(std::move(reader1));

  auto sched = scheduler::Scheduler::Instance();
  std::weak_ptr<Component<M0, M1>> self =
      std::dynamic_pointer_cast<Component<M0, M1>>(shared_from_this());
  auto func = [self](const std::shared_ptr<M0>& msg0,
                     const std::shared_ptr<M1>& msg1) {
    auto ptr = self.lock();
    if (ptr) {
      ptr->Process(msg0, msg1);
    } else {
      AERROR << "Component object has been destroyed." << std::endl;
    }
  };

  std::shared_ptr<data::DataVisitor> dv;
  std::vector<uint64_t> channel_vec;
  channel_vec.emplace_back(common::Hash(config.readers(0).channel()));
  channel_vec.emplace_back(common::Hash(config.readers(1).channel()));
  dv = std::make_shared<data::DataVisitor>(
      std::move(channel_vec), config.readers(0).qos_profile().depth());
  croutine::RoutineFactory factory =
      croutine::CreateRoutineFactory<M0, M1>(func, dv);
  return sched->CreateTask(factory, node_->Name());
}

template <typename M0, typename M1, typename M2>
bool Component<M0, M1, M2>::Process(const std::shared_ptr<M0>& msg0,
                                    const std::shared_ptr<M1>& msg1,
                                    const std::shared_ptr<M2>& msg2) {
  return Proc(msg0, msg1, msg2);
}

template <typename M0, typename M1, typename M2>
bool Component<M0, M1, M2>::Initialize(const ComponentConfig& config) {
  node_.reset(new Node(config.name()));
  SetConfigFilePath(config);

  if (config.readers_size() < 3) {
    AERROR << "Invalid config file: too few readers." << std::endl;
    return false;
  }

  if (!Init()) {
    AERROR << "Component Init() failed." << std::endl;
    return false;
  }

  RoleAttributes attr;
  attr.set_node_name(config.name());
  attr.set_channel_name(config.readers(0).channel());
  auto reader0 = node_->template CreateReader<M0>(attr);
  attr.set_channel_name(config.readers(1).channel());
  auto reader1 = node_->template CreateReader<M1>(attr);
  attr.set_channel_name(config.readers(2).channel());
  auto reader2 = node_->template CreateReader<M2>(attr);
  if (reader0 == nullptr || reader1 == nullptr || reader2 == nullptr) {
    AERROR << "Component create reader failed." << std::endl;
    return false;
  }
  readers.push_back(std::move(reader0));
  readers.push_back(std::move(reader1));
  readers.push_back(std::move(reader2));

  auto sched = scheduler::Scheduler::Instance();
  std::weak_ptr<Component<M0, M1, M2>> self =
      std::dynamic_pointer_cast<Component<M0, M1, M2>>(shared_from_this());
  auto func = [self](const std::shared_ptr<M0>& msg0,
                     const std::shared_ptr<M1>& msg1,
                     const std::shared_ptr<M2>& msg2) {
    auto ptr = self.lock();
    if (ptr) {
      ptr->Process(msg0, msg1, msg2);
    } else {
      AERROR << "Component object has been destroyed." << std::endl;
    }
  };

  std::shared_ptr<data::DataVisitor> dv;
  std::vector<uint64_t> channel_vec;
  channel_vec.emplace_back(common::Hash(config.readers(0).channel()));
  channel_vec.emplace_back(common::Hash(config.readers(1).channel()));
  channel_vec.emplace_back(common::Hash(config.readers(2).channel()));
  dv = std::make_shared<data::DataVisitor>(
      std::move(channel_vec), config.readers(0).qos_profile().depth());
  croutine::RoutineFactory factory =
      croutine::CreateRoutineFactory<M0, M1, M2>(func, dv);
  return sched->CreateTask(factory, node_->Name());
}

#define CYBERTRON_REGISTER_COMPONENT(name) \
  CLASS_LOADER_REGISTER_CLASS(name, apollo::cybertron::ComponentBase)
}  // namespace cybertron
}  // namespace apollo

#endif  // CYBERTRON_COMPONENT_COMPONENT_H_
