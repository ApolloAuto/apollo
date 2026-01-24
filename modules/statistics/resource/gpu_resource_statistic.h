/******************************************************************************
 * Copyright 2023 The Apollo Authors. All Rights Reserved.
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

#ifdef __x86_64__
#include <nvml.h>
#endif

#ifdef __aarch64__
#include <regex>
#endif

#include "modules/statistics_msgs/resource_statistic.pb.h"

#include "cyber/cyber.h"
/**
 * @namespace apollo::resource_statistic
 * @brief apollo::resource_statistic
 */
namespace apollo {
namespace resource_statistic {

class GpuResourceStatistic {
public:
    /**
     * @brief Get GPU metric and write it into gpu_metric.
     * @param gpu_metric Gpu metric message
     * @return If the action is successful.
     */
    bool GetGpuMetric(GpuMetric* gpu_metric);

private:
    /**
     * @brief Init NVML.
     * @return Return true if successful.
     */
    bool Init();

#ifdef __x86_64__
    /**
     * @brief Get GPU devices Number and write into deviceCount.
     * @param deviceCount Gpu device number
     * @return If the action is successful.
     */
    bool GetGpuNumber(unsigned int* deviceCount);

    /**
     * @brief Get GPU device handle and write into device.
     * @param index The number of gpu device
     * @param device Gpu device handle
     * @return If the action is successful.
     */
    bool GetGpuDeviceHandle(const unsigned int index, nvmlDevice_t* device);

    /**
     * @brief Get GPU device name and write it into name.
     * @param device Gpu device handle
     * @param index The name of gpu device
     * @return If the action is successful.
     */
    bool GetGpuName(const nvmlDevice_t& device, char* name);

    /**
     * @brief Get GPU usage and write it into usage.
     * @param device Gpu device handle
     * @param usage The usage of gpu device
     * @return If the action is successful.
     */
    bool GetUsage(const nvmlDevice_t& device, float* usage);

    /**
     * @brief Get GPU temperature and write it into temp.
     * @param device Gpu device handle
     * @param usage The temperature of gpu device
     * @return If the action is successful.
     */
    bool GetTemperature(const nvmlDevice_t& device, float* temp);
#endif
};

}  // namespace resource_statistic
}  // namespace apollo
