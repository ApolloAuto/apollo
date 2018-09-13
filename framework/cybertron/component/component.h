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

#ifndef CYBERTRON_COMPONENT_COMPONENT_H_
#define CYBERTRON_COMPONENT_COMPONENT_H_

#include <memory>
#include <vector>

#include "cybertron/common/global_data.h"
#include "cybertron/common/types.h"
#include "cybertron/common/util.h"
#include "cybertron/component/component_base.h"
#include "cybertron/croutine/routine_factory.h"
#include "cybertron/data/data_visitor.h"
#include "cybertron/scheduler/scheduler.h"

namespace apollo {
namespace cybertron {

using apollo::cybertron::common::GlobalData;
using apollo::cybertron::proto::RoleAttributes;

template <typename M0 = NullType, typename M1 = NullType,
          typename M2 = NullType, typename M3 = NullType>
class Component : public ComponentBase {
 public:
  Component() {}
  ~Component() override {}
  bool Initialize(const ComponentConfig& config) override;
  bool Process(const std::shared_ptr<M0>& msg0, const std::shared_ptr<M1>& msg1,
               const std::shared_ptr<M2>& msg2,
               const std::shared_ptr<M3>& msg3);

 private:
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
  // TODO(hewei03): Add some protection here.
  return Proc(msg);
}

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
bool Component<M0, NullType, NullType, NullType>::Initialize(
    const ComponentConfig& config) {
  node_.reset(new Node(config.name()));
  SetConfigFilePath(config);

  if (config.readers_size() < 1) {
    AERROR << "Invalid config file: too few readers.";
    return false;
  }

  if (!Init()) {
    AERROR << "Component Init() failed.";
    return false;
  }
  RoleAttributes attr;
  attr.set_node_name(config.name());
  attr.set_channel_name(config.readers(0).channel());
  auto qos_profile = attr.mutable_qos_profile();
  *qos_profile = config.readers(0).qos_profile();
  auto reader = node_->CreateReader<M0>(attr);
  if (reader == nullptr) {
    AERROR << "Component create reader failed.";
    return false;
  }
  auto dv = std::make_shared<data::DataVisitor<M0>>(reader);
  readers_.emplace_back(std::move(reader));

  auto sched = scheduler::Scheduler::Instance();
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

  croutine::RoutineFactory factory =
      croutine::CreateRoutineFactory<M0>(func, dv);
  return sched->CreateTask(factory, node_->Name());
}

template <typename M0, typename M1>
bool Component<M0, M1, NullType, NullType>::Process(
    const std::shared_ptr<M0>& msg0, const std::shared_ptr<M1>& msg1) {
  return Proc(msg0, msg1);
}

template <typename M0, typename M1>
bool Component<M0, M1, NullType, NullType>::Initialize(
    const ComponentConfig& config) {
  node_.reset(new Node(config.name()));
  SetConfigFilePath(config);

  if (config.readers_size() < 2) {
    AERROR << "Invalid config file: too few readers.";
    return false;
  }

  if (!Init()) {
    AERROR << "Component Init() failed.";
    return false;
  }

  RoleAttributes attr;
  attr.set_node_name(config.name());
  attr.set_channel_name(config.readers(0).channel());
  auto reader0 = node_->template CreateReader<M0>(attr);
  attr.set_channel_name(config.readers(1).channel());
  auto reader1 = node_->template CreateReader<M1>(attr);
  if (reader0 == nullptr || reader1 == nullptr) {
    AERROR << "Component create reader failed.";
    return false;
  }
  readers_.push_back(std::move(reader0));
  readers_.push_back(std::move(reader1));

  auto sched = scheduler::Scheduler::Instance();
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

  auto dv = std::make_shared<data::DataVisitor<M0, M1>>(readers_);
  croutine::RoutineFactory factory =
      croutine::CreateRoutineFactory<M0, M1>(func, dv);
  return sched->CreateTask(factory, node_->Name());
}

template <typename M0, typename M1, typename M2>
bool Component<M0, M1, M2, NullType>::Process(const std::shared_ptr<M0>& msg0,
                                              const std::shared_ptr<M1>& msg1,
                                              const std::shared_ptr<M2>& msg2) {
  return Proc(msg0, msg1, msg2);
}

template <typename M0, typename M1, typename M2>
bool Component<M0, M1, M2, NullType>::Initialize(
    const ComponentConfig& config) {
  node_.reset(new Node(config.name()));
  SetConfigFilePath(config);

  if (config.readers_size() < 3) {
    AERROR << "Invalid config file: too few readers.";
    return false;
  }

  if (!Init()) {
    AERROR << "Component Init() failed.";
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
    AERROR << "Component create reader failed.";
    return false;
  }
  readers_.push_back(std::move(reader0));
  readers_.push_back(std::move(reader1));
  readers_.push_back(std::move(reader2));

  auto sched = scheduler::Scheduler::Instance();
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

  auto dv = std::make_shared<data::DataVisitor<M0, M1, M2>>(readers_);
  croutine::RoutineFactory factory =
      croutine::CreateRoutineFactory<M0, M1, M2>(func, dv);
  return sched->CreateTask(factory, node_->Name());
}

template <typename M0, typename M1, typename M2, typename M3>
bool Component<M0, M1, M2, M3>::Process(const std::shared_ptr<M0>& msg0,
                                        const std::shared_ptr<M1>& msg1,
                                        const std::shared_ptr<M2>& msg2,
                                        const std::shared_ptr<M3>& msg3) {
  return Proc(msg0, msg1, msg2, msg3);
}

template <typename M0, typename M1, typename M2, typename M3>
bool Component<M0, M1, M2, M3>::Initialize(const ComponentConfig& config) {
  node_.reset(new Node(config.name()));
  SetConfigFilePath(config);

  if (config.readers_size() < 4) {
    AERROR << "Invalid config file: too few readers_." << std::endl;
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
  attr.set_channel_name(config.readers(3).channel());
  auto reader3 = node_->template CreateReader<M3>(attr);
  if (reader0 == nullptr || reader1 == nullptr || reader2 == nullptr ||
      reader3 == nullptr) {
    AERROR << "Component create reader failed." << std::endl;
    return false;
  }
  readers_.push_back(std::move(reader0));
  readers_.push_back(std::move(reader1));
  readers_.push_back(std::move(reader2));
  readers_.push_back(std::move(reader3));

  auto sched = scheduler::Scheduler::Instance();
  std::weak_ptr<Component<M0, M1, M2, M3>> self =
      std::dynamic_pointer_cast<Component<M0, M1, M2, M3>>(shared_from_this());
  auto func = [self](
      const std::shared_ptr<M0>& msg0, const std::shared_ptr<M1>& msg1,
      const std::shared_ptr<M2>& msg2, const std::shared_ptr<M3>& msg3) {
    auto ptr = self.lock();
    if (ptr) {
      ptr->Process(msg0, msg1, msg2, msg3);
    } else {
      AERROR << "Component object has been destroyed." << std::endl;
    }
  };

  auto dv = std::make_shared<data::DataVisitor<M0, M1, M2, M3>>(readers_);
  croutine::RoutineFactory factory =
      croutine::CreateRoutineFactory<M0, M1, M2, M3>(func, dv);
  return sched->CreateTask(factory, node_->Name());
}

#define CYBERTRON_REGISTER_COMPONENT(name) \
  CLASS_LOADER_REGISTER_CLASS(name, apollo::cybertron::ComponentBase)

}  // namespace cybertron
}  // namespace apollo

#endif  // CYBERTRON_COMPONENT_COMPONENT_H_
