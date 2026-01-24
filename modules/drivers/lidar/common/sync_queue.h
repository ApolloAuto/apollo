/*********************************************************************************************************************
Copyright (c) 2020 RoboSense
All rights reserved

By downloading, copying, installing or using the software you agree to this
license. If you do not agree to this license, do not download, install, copy or
use the software.

License Agreement
For RoboSense LiDAR SDK Library
(3-clause BSD License)

Redistribution and use in source and binary forms, with or without modification,
are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this
list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice,
this list of conditions and the following disclaimer in the documentation and/or
other materials provided with the distribution.

3. Neither the names of the RoboSense, nor Suteng Innovation Technology, nor the
names of other contributors may be used to endorse or promote products derived
from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR
ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*********************************************************************************************************************/
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

#include <condition_variable>
#include <mutex>
#include <queue>
#include <thread>
#include <utility>

namespace apollo {
namespace lidar {
namespace drivers {

template <typename T>
class SyncQueue {
 public:
    inline size_t push(const T& value) {
        bool empty = false;
        size_t size = 0;

        {
            std::lock_guard<std::mutex> lg(mtx_);
            empty = queue_.empty();
            queue_.push(value);
            size = queue_.size();
        }

        if (empty)
            cv_.notify_one();
        return size;
    }

    inline T pop() {
        T value;

        std::lock_guard<std::mutex> lg(mtx_);
        if (!queue_.empty()) {
            value = queue_.front();
            queue_.pop();
        }

        return value;
    }

    inline bool popWait(T& ret_ele, unsigned int usec = 1000000) {
        {
            std::lock_guard<std::mutex> lg(mtx_);
            if (!queue_.empty()) {
                ret_ele = queue_.front();
                queue_.pop();
                return true;
            }
        }

        std::this_thread::sleep_for(std::chrono::microseconds(1000));
        return false;
    }

    inline void clear() {
        std::queue<T> empty;
        std::lock_guard<std::mutex> lg(mtx_);
        swap(empty, queue_);
    }

 private:
    std::queue<T> queue_;
    std::mutex mtx_;
    std::condition_variable cv_;
};

}  // namespace drivers
}  // namespace lidar
}  // namespace apollo
