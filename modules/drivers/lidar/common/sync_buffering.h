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
#pragma once

#include "deque"
#include "functional"
#include "memory"
#include "mutex"
#include "set"
#include "unordered_set"
#include "vector"

#include "cyber/cyber.h"

namespace apollo {
namespace drivers {
namespace lidar {
template <typename T>
class SyncBuffering {
 public:
    SyncBuffering() = default;
    SyncBuffering(
            std::function<std::shared_ptr<T>(void)> allocator_function,
            std::function<void(std::shared_ptr<T>&)> cleaner_function) :
            allocator_function_(allocator_function),
            cleaner_function_(cleaner_function) {}

    void SetBufferSize(int buffer_size);

    void SetAllocator(
            std::function<std::shared_ptr<T>(void)> allocator_function);

    void SetCleaner(std::function<void(std::shared_ptr<T>&)> cleaner_function);

    void Init();

    std::shared_ptr<T> GenerateNewObject();

    std::shared_ptr<T> AllocateElement();

 private:
    int buffer_size_ = 10;
    std::vector<std::shared_ptr<T>> buffer_;

    std::atomic_int buffer_index_{0};

    std::function<std::shared_ptr<T>(void)> allocator_function_ = nullptr;
    std::function<void(std::shared_ptr<T>&)> cleaner_function_ = nullptr;
};

template <typename T>
void SyncBuffering<T>::SetBufferSize(int buffer_size) {
    buffer_size_ = buffer_size;
}

template <typename T>
void SyncBuffering<T>::SetAllocator(
        std::function<std::shared_ptr<T>(void)> allocator_function) {
    allocator_function_ = allocator_function;
}

template <typename T>
void SyncBuffering<T>::SetCleaner(
        std::function<void(std::shared_ptr<T>&)> cleaner_function) {
    cleaner_function_ = cleaner_function;
}

template <typename T>
void SyncBuffering<T>::Init() {
    for (int i = 0; i < buffer_size_; ++i) {
        buffer_.push_back(GenerateNewObject());
    }
}

template <typename T>
std::shared_ptr<T> SyncBuffering<T>::GenerateNewObject() {
    std::shared_ptr<T> new_object;
    if (allocator_function_ == nullptr) {
        new_object = std::make_shared<T>();
    } else {
        new_object = allocator_function_();
    }
    return new_object;
}

template <typename T>
std::shared_ptr<T> SyncBuffering<T>::AllocateElement() {
    int element_idx = buffer_index_.fetch_add(1);
    std::shared_ptr<T> element = buffer_[element_idx % buffer_size_];
    if (cleaner_function_) {
        cleaner_function_(element);
    }
    return element;
}

}  // namespace lidar
}  // namespace drivers
}  // namespace apollo
