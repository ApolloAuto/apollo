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

#include <atomic>
#include <limits>
#include <memory>
#include <string>

#include "cyber/profiler/profiler.h"
#include "cyber/component/component.h"
#include "modules/transform/buffer.h"
#include "modules/common_msgs/sensor_msgs/pointcloud.pb.h"
#include "modules/common_msgs/perception_msgs/perception_barrier_gate.pb.h"
#include "modules/perception/common/hdmap/hdmap_input.h"
#include "modules/perception/common/onboard/inner_component_messages/lidar_inner_component_messages.h"
#include "modules/perception/barrier_recognition/proto/barrier_recognition_component_config.pb.h"
#include "modules/perception/barrier_recognition/interface/base_barrier_recognizer.h"
#include "modules/perception/barrier_recognition/detector/barrier_recognition_bank.h"
#include "modules/perception/barrier_recognition/tracker/barrier_status_tracker.h"

namespace apollo {
namespace perception {
namespace lidar {

using onboard::LidarFrameMessage;
using apollo::transform::Buffer;

class BarrierRecognitionComponent final
    : public cyber::Component<LidarFrameMessage> {
public:
    using BarrierGate = apollo::perception::BarrierGate;
    using PerceptionBarrierGate = apollo::perception::PerceptionBarrierGate;
public:
    BarrierRecognitionComponent() = default;
    virtual ~BarrierRecognitionComponent() = default;
    /**
     * @brief Init barrier recognition component
     *
     * @return true
     * @return false
     */
    bool Init() override;
    /**
     * @brief Process of barrier recognition
     *
     * @param message PointCloud message from driver
     * @return true
     * @return false
     */
    bool Proc(const std::shared_ptr<LidarFrameMessage>& message) override;

private:
    bool InternalProc(
            const std::shared_ptr<LidarFrameMessage>& message,
            std::vector<BarrieGate>& gates);

    bool QueryPoseAndGates(const double ts, const Eigen::Affine3d& pose,
                           std::vector<apollo::hdmap::BarrierGate>* gates);

    void GetGatesPolygon(const apollo::hdmap::BarrierGate& barrier_gate,
                         std::vector<double>& world_polygon);

    bool MakeProtobufMsg(double msg_timestamp, int seq_num,
                         std::vector<BarrieGate> gates,
                         PerceptionBarrierGate *msg);

private:
    uint32_t seq_num_ = 0;
    double forward_distance_to_queries_ = 20;
    double last_gates_ts_ = -1.0;
    double valid_hdmap_interval_ = 1.5;
    double tf2_timeout_second_ = 0.01;

    std::string tf2_frame_id_;
    std::string tf2_child_frame_id_;

    // Barrier recognizer bank
    BarrierRecognizerBank barrier_bank_;
    // Barrier tracker
    BarrierStatusTracker status_tracker_;
    
    std::vector<apollo::hdmap::BarrierGate> last_gates_;
    Buffer* tf2_buffer_ = Buffer::Instance();
    apollo::perception::map::HDMapInput* hd_map_ = nullptr;

    std::shared_ptr<cyber::Writer<PerceptionBarrierGate>> writer_;
};

CYBER_REGISTER_COMPONENT(BarrierRecognitionComponent);

}  // namespace lidar
}  // namespace perception
}  // namespace apollo
