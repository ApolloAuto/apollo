/******************************************************************************
 * Copyright 2024 The Apollo Authors. All Rights Reserved.
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
#ifndef APOLLO_LIVOX_OBSERABLE_BINDER_H
#define APOLLO_LIVOX_OBSERABLE_BINDER_H

#include <functional>
#include <memory>
#include <utility>

#include "cyber/cyber.h"

namespace apollo {
namespace drivers {
namespace lidar {

template <typename... FuncArgs>
class LivoxObserableBinder {
 public:
    using FuncType = std::shared_ptr<std::function<void(FuncArgs...)>>;
    static void Callback(FuncArgs... args);

    static void RegisterCallback(FuncType func);

    static FuncType func_cb_;
};

template <typename... FuncArgs>
void LivoxObserableBinder<FuncArgs...>::Callback(FuncArgs... args) {
    if (func_cb_) {
        (*func_cb_)(std::forward<FuncArgs>(args)...);
    } else {
        AERROR << "Binder callback is empty";
    }
}

template <typename... FuncArgs>
void LivoxObserableBinder<FuncArgs...>::RegisterCallback(
        LivoxObserableBinder::FuncType func) {
    func_cb_ = std::move(func);
}

template <typename... FuncArgs>
std::shared_ptr<std::function<void(FuncArgs...)>>
        LivoxObserableBinder<FuncArgs...>::func_cb_ = nullptr;

}  // namespace lidar
}  // namespace drivers
}  // namespace apollo
#endif  // APOLLO_LIVOX_OBSERABLE_BINDER_H
