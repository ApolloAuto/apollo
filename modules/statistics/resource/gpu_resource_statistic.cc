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
#include "modules/statistics/resource/gpu_resource_statistic.h"

#include <algorithm>

namespace apollo {
namespace resource_statistic {

#ifdef __x86_64__
bool GpuResourceStatistic::Init() {
    nvmlReturn_t ret = nvmlInit();
    if (ret != NVML_SUCCESS) {
        AERROR << "Failed to initialize NVML: " << nvmlErrorString(ret);
        return false;
    }
    return true;
}
// gpu
bool GpuResourceStatistic::GetGpuMetric(GpuMetric* gpu_metric) {
    // Init call once
    static bool nvml_has_init = Init();
    if (!nvml_has_init) {
        return false;
    }
    // Get devices count
    unsigned int deviceCount = 0;
    if (!GetGpuNumber(&deviceCount)) {
        return false;
    }
    float avg_usage = 0;
    float max_temp = 0;
    for (unsigned int index = 0; index < deviceCount; ++index) {
        nvmlDevice_t device;
        float temp = 0;
        float usage = 0;
        if (!GetGpuDeviceHandle(index, &device)) {
            continue;
        }
        if (GetUsage(device, &usage)) {
            avg_usage += usage;
        }
        if (GetTemperature(device, &temp)) {
            max_temp = std::max(max_temp, temp);
        }
    }
    gpu_metric->set_usage(avg_usage / deviceCount);
    gpu_metric->set_temperature(max_temp);
    return true;
}

bool GpuResourceStatistic::GetGpuNumber(unsigned int* deviceCount) {
    nvmlReturn_t ret = nvmlDeviceGetCount(deviceCount);
    if (ret != NVML_SUCCESS) {
        AERROR << "Failed to retrieve the number of compute devices: " << nvmlErrorString(ret);
        return false;
    }
    return true;
}

bool GpuResourceStatistic::GetGpuDeviceHandle(const unsigned int index, nvmlDevice_t* device) {
    nvmlReturn_t ret = nvmlDeviceGetHandleByIndex(index, device);
    if (ret != NVML_SUCCESS) {
        AWARN << "Failed to acquire the handle for device [" << index << "]: " << nvmlErrorString(ret);
        return false;
    }
    return true;
}

bool GpuResourceStatistic::GetGpuName(const nvmlDevice_t& device, char* name) {
    nvmlReturn_t ret = nvmlDeviceGetName(device, name, NVML_DEVICE_NAME_BUFFER_SIZE);
    if (ret != NVML_SUCCESS) {
        AWARN << "Failed to retrieve the name of this device: " << nvmlErrorString(ret);
        return false;
    }
    return true;
}

bool GpuResourceStatistic::GetTemperature(const nvmlDevice_t& device, float* temp) {
    // Get device's temperature
    unsigned int temperature = 0;
    nvmlReturn_t ret = nvmlDeviceGetTemperature(device, NVML_TEMPERATURE_GPU, &temperature);
    if (ret != NVML_SUCCESS) {
        AWARN << "Failed to retrieve the current temperature for device: " << nvmlErrorString(ret);
        return false;
    }
    *temp = static_cast<float>(temperature);
    return true;
}

bool GpuResourceStatistic::GetUsage(const nvmlDevice_t& device, float* usage) {
    nvmlUtilization_t utilization;
    nvmlReturn_t ret = nvmlDeviceGetUtilizationRates(device, &utilization);
    if (ret != NVML_SUCCESS) {
        AWARN << "Failed to retrieve the current utilization rates for device: " << nvmlErrorString(ret);
        return false;
    }
    *usage = static_cast<float>(utilization.gpu) / 100.0;
    return true;
}
#endif

#ifdef __aarch64__
bool GpuResourceStatistic::Init() {
    AINFO << "GpuResourceStatistic Init on Jetson.";
    return true;
}

bool GpuResourceStatistic::GetGpuMetric(GpuMetric* gpu_metric) {
    FILE* pipe = popen("tegrastats", "r");
    if (!pipe) {
        AERROR << "Failed to execute tegrastats command." << std::endl;
        return false;
    }

    std::string tegrastatsOutput;
    char buffer[1024];
    if (fgets(buffer, sizeof(buffer), pipe) != nullptr) {
        tegrastatsOutput = std::string(buffer);
    }

    pclose(pipe);

    std::regex usagePattern("GR3D_FREQ (\\d+,)?(\\d+)%");
    std::regex tempPattern("GPU( \\d+)?@(-?\\d+(\\.\\d+)?)C");

    std::smatch usageMatch, tempMatch;

    // query avg gpu usage
    float gpuUsageTotal = 0;
    float gpuUsageAvg = 0;
    int gpu_count = 0;
    auto usageBegin = tegrastatsOutput.cbegin();
    while (std::regex_search(usageBegin, tegrastatsOutput.cend(), usageMatch, usagePattern)) {
        if (usageMatch.size() == 3) {
            gpuUsageTotal += std::stof(usageMatch[2]);
            gpu_count += 1;
            usageBegin = usageMatch.suffix().first;
        }
    }
    if (gpu_count != 0) {
        gpuUsageAvg = gpuUsageTotal / gpu_count;
    }

    // query max gpu temperature
    float gpuTempMax = 0;
    auto tempBegin = tegrastatsOutput.cbegin();
    while (std::regex_search(tempBegin, tegrastatsOutput.cend(), tempMatch, tempPattern)) {
        if (tempMatch.size() == 4) {
            float gpuTemp = std::stof(tempMatch[2]);
            gpuTempMax = gpuTemp > gpuTempMax ? gpuTemp : gpuTempMax;
            tempBegin = tempMatch.suffix().first;
        }
    }

    gpu_metric->set_usage(gpuUsageAvg);
    gpu_metric->set_temperature(gpuTempMax);
    return true;
}

#endif

}  // namespace resource_statistic
}  // namespace apollo
