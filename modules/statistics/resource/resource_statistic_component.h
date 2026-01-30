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

#include <memory>

#include "modules/statistics_msgs/resource_statistic.pb.h"

#include "cyber/component/timer_component.h"
#include "cyber/cyber.h"
#include "modules/statistics/resource/cpu_resource_statistic.h"
#include "modules/statistics/resource/disk_resource_statistic.h"
#include "modules/statistics/resource/gpu_resource_statistic.h"
#include "modules/statistics/resource/memory_resource_statistic.h"
/**
 * @namespace apollo::resource_statistic
 * @brief apollo::resource_statistic
 */
namespace apollo {
namespace resource_statistic {

class ResourceStatisticComponent : public apollo::cyber::TimerComponent {
public:
    bool Init() override;
    bool Proc() override;

private:
    /**
     * @brief Load conf.
     */
    void LoadConf(const ResourceStatisticConf& conf);

    CpuResourceStatistic cpu_statistic_;
    GpuResourceStatistic gpu_statistic_;
    DiskResourceStatistic disk_statistic_;
    MemoryResourceStatistic mem_statistic_;
    std::shared_ptr<cyber::Writer<ResourceMetrics>> metric_writer_;
};

CYBER_REGISTER_COMPONENT(ResourceStatisticComponent)

}  // namespace resource_statistic
}  // namespace apollo
