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

#include <map>
#include <regex>
#include <string>

#include <livox_lidar_def.h>

#include "cyber/cyber.h"

namespace apollo {
namespace drivers {
namespace lidar {

class LivoxDispatcher {
 public:
    LivoxDispatcher() : handle_callback_functions(), mtx() {}

    using PointCloudCallbackType = std::function<void(
            uint32_t handle,
            const uint8_t dev_type,
            LivoxLidarEthernetPacket* data,
            void* client_data)>;

    bool GetHandleFromIP(const std::string& ip, uint32_t& handle);

    void RegisterHandleDispatchCallback(
            uint32_t handle,
            PointCloudCallbackType cb);

    void LivoxPointCloudCallback(
            uint32_t handle,
            const uint8_t dev_type,
            LivoxLidarEthernetPacket* data,
            void* client_data);

    static LivoxDispatcher& GetLivoxDispatcherInstance() {
        return dispatcher_instance_;
    }

 private:
    static LivoxDispatcher dispatcher_instance_;
    std::map<uint32_t, PointCloudCallbackType> handle_callback_functions;
    std::mutex mtx;
};

static void GlobalPointCloudCallback(
        uint32_t handle,
        const uint8_t dev_type,
        LivoxLidarEthernetPacket* data,
        void* client_data) {
    auto* dispatcher_pointer = reinterpret_cast<LivoxDispatcher*>(client_data);
    dispatcher_pointer->LivoxPointCloudCallback(
            handle, dev_type, data, client_data);
}

}  // namespace lidar
}  // namespace drivers
}  // namespace apollo
