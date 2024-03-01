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

#ifndef CYBER_CROUTINE_ROUTINE_FACTORY_H_
#define CYBER_CROUTINE_ROUTINE_FACTORY_H_

#include <memory>
#include <utility>

#include "cyber/common/global_data.h"
#include "cyber/common/log.h"
#include "cyber/croutine/croutine.h"
#include "cyber/data/data_visitor.h"
#include "cyber/event/perf_event_cache.h"

namespace apollo {
namespace cyber {
namespace croutine {

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

template <typename ...M, size_t ...N, typename F>
RoutineFactory CreateRoutineFactory(
    F&& f, const std::shared_ptr<data::DataVisitor<M...>>& dv,
    std::index_sequence<N...>) {
  RoutineFactory factory;
  factory.SetDataVisitor(dv);
  factory.create_routine = [=]() {
    return [=]() {
      auto msg = std::make_tuple(std::shared_ptr<M>()...);
      for (;;) {
        CRoutine::GetCurrentRoutine()->set_state(RoutineState::DATA_WAIT);
        if (dv->TryFetch(std::get<N>(msg)...)) {
          f(std::get<N>(msg)...);
          CRoutine::Yield(RoutineState::READY);
        } else {
          CRoutine::Yield();
        }
      }
    };
  };
  return factory;
}

template <typename ...M, typename F>
RoutineFactory CreateRoutineFactory(
    F&& f, const std::shared_ptr<data::DataVisitor<M...>>& dv) {
  return CreateRoutineFactory(f, dv, std::make_index_sequence<sizeof...(M)>{});
}

template <typename Function>
RoutineFactory CreateRoutineFactory(Function&& f) {
  RoutineFactory factory;
  factory.create_routine = [f = std::forward<Function&&>(f)]() { return f; };
  return factory;
}

}  // namespace croutine
}  // namespace cyber
}  // namespace apollo

#endif  // CYBER_CROUTINE_ROUTINE_FACTORY_H_
