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
#include "modules/statistics/resource/resource_statistic_component.h"

#include "modules/common/util/message_util.h"
#include "modules/statistics/resource/resource_statistic_gflags.h"

namespace apollo {
namespace resource_statistic {

bool ResourceStatisticComponent::Init() {
    // hook: Apollo License Verification: v_apollo_park
    ResourceStatisticConf conf;
    if (!ComponentBase::GetProtoConfig(&conf)) {
        AERROR << "Unable to load resource statistic conf file: " << ComponentBase::ConfigFilePath();
        return false;
    }
    LoadConf(conf);
    disk_statistic_.SetInterval(GetInterval());
    // create metric writer
    metric_writer_ = node_->CreateWriter<ResourceMetrics>(FLAGS_resource_statistic_topic);
    return true;
}

void ResourceStatisticComponent::LoadConf(const ResourceStatisticConf& conf) {
    if (conf.has_disk_conf()) {
        disk_statistic_.LoadDiskConf(conf.disk_conf());
    }
}

bool ResourceStatisticComponent::Proc() {
    auto metrics = std::make_shared<ResourceMetrics>();
    apollo::common::util::FillHeader(node_->Name(), metrics.get());

    auto cpu_metric = metrics->mutable_cpu();
    cpu_statistic_.GetCpuMetric(cpu_metric);

    auto gpu_metric = metrics->mutable_gpu();
    gpu_statistic_.GetGpuMetric(gpu_metric);

    auto mem_metric = metrics->mutable_memory();
    mem_statistic_.GetMemoryMetric(mem_metric);

    auto disk_metric = metrics->mutable_disk();
    disk_statistic_.GetDiskMetric(disk_metric);

    metric_writer_->Write(metrics);
    return true;
}

}  // namespace resource_statistic
}  // namespace apollo
