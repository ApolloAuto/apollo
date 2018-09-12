
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

#ifndef CYBERTRON_CROUTINE_ROUTINE_FACTORY_H_
#define CYBERTRON_CROUTINE_ROUTINE_FACTORY_H_

#include <memory>
#include <utility>

#include "cybertron/common/global_data.h"
#include "cybertron/common/log.h"
#include "cybertron/croutine/croutine.h"
#include "cybertron/data/data_visitor.h"
#include "cybertron/event/perf_event_cache.h"

namespace apollo {
namespace cybertron {
namespace croutine {

using common::GlobalData;
using apollo::cybertron::event::PerfEventCache;

class RoutineFactory {
 public:
  using VoidFunc = std::function<void()>;
  using CreateRoutineFunc = std::function<VoidFunc()>;
  // We can use routine_func directly.
  CreateRoutineFunc create_routine;
  inline std::shared_ptr<data::DataVisitorBase> GetDataVisitor() const {
    return data_visitor_;
  }
  inline void SetDataVisitor(const std::shared_ptr<data::DataVisitorBase>& dv) {
    data_visitor_ = dv;
  }

 private:
  std::shared_ptr<data::DataVisitorBase> data_visitor_ = nullptr;
};

template <typename M0, typename F>
RoutineFactory CreateRoutineFactory(
    F&& f, const std::shared_ptr<data::DataVisitor<M0>>& dv) {
  RoutineFactory factory;
  factory.SetDataVisitor(dv);
  factory.create_routine = [=]() {
    return [=]() {
      std::shared_ptr<M0> msg;
      for (;;) {
        if (dv->TryFetch(msg)) {
          PerfEventCache::Instance()->AddSchedEvent(
              3, CRoutine::GetCurrentRoutine()->Id(),
              CRoutine::GetCurrentRoutine()->ProcessorId(), 0, 0, 1);
          f(msg);
        } else {
          PerfEventCache::Instance()->AddSchedEvent(
              3, CRoutine::GetCurrentRoutine()->Id(),
              CRoutine::GetCurrentRoutine()->ProcessorId(), 0, 0, 0);
          CRoutine::GetCurrentRoutine()->SetState(RoutineState::WAITING_INPUT);
        }
        CRoutine::Yield();
      }
    };
  };
  return factory;
}

// TODO(hewei03): Use parameter pack here.
template <typename M0, typename M1, typename F>
RoutineFactory CreateRoutineFactory(
    F&& f, const std::shared_ptr<data::DataVisitor<M0, M1>>& dv) {
  RoutineFactory factory;
  factory.SetDataVisitor(dv);
  factory.create_routine = [=]() {
    return [=]() {
      std::shared_ptr<M0> msg0;
      std::shared_ptr<M1> msg1;
      for (;;) {
        if (dv->TryFetch(msg0, msg1)) {
          PerfEventCache::Instance()->AddSchedEvent(
              3, CRoutine::GetCurrentRoutine()->Id(),
              CRoutine::GetCurrentRoutine()->ProcessorId(), 0, 0, 1);
          f(msg0, msg1);
        } else {
          PerfEventCache::Instance()->AddSchedEvent(
              3, CRoutine::GetCurrentRoutine()->Id(),
              CRoutine::GetCurrentRoutine()->ProcessorId(), 0, 0, 0);
          CRoutine::GetCurrentRoutine()->SetState(RoutineState::WAITING_INPUT);
        }
        CRoutine::Yield();
      }
    };
  };
  return factory;
}

template <typename M0, typename M1, typename M2, typename F>
RoutineFactory CreateRoutineFactory(
    F&& f, const std::shared_ptr<data::DataVisitor<M0, M1, M2>>& dv) {
  RoutineFactory factory;
  factory.SetDataVisitor(dv);
  factory.create_routine = [=]() {
    return [=]() {
      std::shared_ptr<M0> msg0;
      std::shared_ptr<M1> msg1;
      std::shared_ptr<M2> msg2;
      for (;;) {
        if (dv->TryFetch(msg0, msg1, msg2)) {
          PerfEventCache::Instance()->AddSchedEvent(
              3, CRoutine::GetCurrentRoutine()->Id(),
              CRoutine::GetCurrentRoutine()->ProcessorId(), 0, 0, 1);
          f(msg0, msg1, msg2);
        } else {
          PerfEventCache::Instance()->AddSchedEvent(
              3, CRoutine::GetCurrentRoutine()->Id(),
              CRoutine::GetCurrentRoutine()->ProcessorId(), 0, 0, 0);
          CRoutine::GetCurrentRoutine()->SetState(RoutineState::WAITING_INPUT);
        }
        CRoutine::Yield();
      }
    };
  };
  return factory;
}

template <typename M0, typename M1, typename M2, typename M3, typename F>
RoutineFactory CreateRoutineFactory(
    F&& f, const std::shared_ptr<data::DataVisitor<M0, M1, M2, M3>>& dv) {
  RoutineFactory factory;
  factory.SetDataVisitor(dv);
  factory.create_routine = [=]() {
    return [=]() {
      std::shared_ptr<M0> msg0;
      std::shared_ptr<M1> msg1;
      std::shared_ptr<M2> msg2;
      std::shared_ptr<M3> msg3;
      for (;;) {
        if (dv->TryFetch(msg0, msg1, msg2, msg3)) {
          PerfEventCache::Instance()->AddSchedEvent(
              3, CRoutine::GetCurrentRoutine()->Id(),
              CRoutine::GetCurrentRoutine()->ProcessorId(), 0, 0, 1);
          f(msg0, msg1, msg2, msg3);
        } else {
          PerfEventCache::Instance()->AddSchedEvent(
              3, CRoutine::GetCurrentRoutine()->Id(),
              CRoutine::GetCurrentRoutine()->ProcessorId(), 0, 0, 0);
          CRoutine::GetCurrentRoutine()->SetState(RoutineState::WAITING_INPUT);
        }
        CRoutine::Yield();
      }
    };
  };
  return factory;
}

template <typename Function>
RoutineFactory CreateRoutineFactory(Function&& f) {
  RoutineFactory factory;
  factory.create_routine = [f = std::forward<Function&&>(f)]() { return f; };
  return factory;
}

}  // namespace croutine
}  // namespace cybertron
}  // namespace apollo

#endif  // CYBERTRON_CROUTINE_ROUTINE_FACTORY_H_
