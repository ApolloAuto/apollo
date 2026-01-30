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

#include "modules/perception/barrier_recognition/barrier_recognition_component.h"

#include "cyber/time/clock.h"
#include "modules/perception/common/lidar/common/lidar_frame_pool.h"

namespace apollo {
namespace perception {
namespace lidar {

using apollo::cyber::common::GetAbsolutePath;

const std::string kBarrierId2Type[] = {
    "UNKOWN", "ROD", "FENCE", "ADVERTISING", "TELESCOPIC", "OTHER"
};

bool BarrierRecognitionComponent::Init() {
    BarrierRecognitionComponentConfig barrier_comp_config;
    if (!GetProtoConfig(&barrier_comp_config)) {
        AERROR << "Get BarrierRecognitionComponent file failed";
        return false;
    }
    AINFO << "BarrierRecognitionComponent Component Configs: " << barrier_comp_config.DebugString();

    tf2_frame_id_ = barrier_comp_config.frame_id();
    tf2_child_frame_id_ = barrier_comp_config.child_frame_id();

    auto recognizer_param = barrier_comp_config.recognizer_param();
    BarrierRecognizerInitOptions barrier_bank_init_options;
    barrier_bank_init_options.config_path = recognizer_param.config_path();
    barrier_bank_init_options.config_file = recognizer_param.config_file();
    ACHECK(barrier_bank_.Init(barrier_bank_init_options));

    auto tracker_param = barrier_comp_config.tracker_param();
    StatusTrackerInitOptions status_tracker_init_options;
    status_tracker_init_options.config_path = tracker_param.config_path();
    status_tracker_init_options.config_file = tracker_param.config_file();
    ACHECK(status_tracker_.Init(status_tracker_init_options));
    
    // init hdmaps
    hd_map_ = map::HDMapInput::Instance();
    if (hd_map_ == nullptr) {
        AERROR << "BarrierRecognitionComponent get hdmap failed.";
        return false;
    }

    if (!hd_map_->Init()) {
        AERROR << "BarrierRecognitionComponent init hd-map failed.";
        return false;
    }
    AINFO << "Successfully init barrier recognition component.";

    writer_ = node_->CreateWriter<PerceptionBarrierGate>(
      barrier_comp_config.output_channel_name());

    return true;
}

bool BarrierRecognitionComponent::Proc(const std::shared_ptr<LidarFrameMessage>& message) {
    PERF_FUNCTION()
    // internal proc
    std::vector<BarrieGate> gates;
    bool status = InternalProc(message, gates);
    if (status) {
        std::shared_ptr<PerceptionBarrierGate> out_message(new (std::nothrow)
                                                           PerceptionBarrierGate);
        if (!MakeProtobufMsg(message->timestamp_, seq_num_, gates,
                            out_message.get())) {
            AERROR << "MakeProtobufMsg failed ts: " << message->timestamp_;
            return false;
        }
        seq_num_++;
        writer_->Write(out_message);
        AINFO << "Send peception barrier gate message.";
     }
    return status;
}

bool BarrierRecognitionComponent::InternalProc(
                const std::shared_ptr<LidarFrameMessage>& message, 
                std::vector<BarrieGate>& gates) {
    const double timestamp = message->timestamp_;
    auto frame = message->lidar_frame_.get();

    std::vector<apollo::hdmap::BarrierGate> hdmap_gates;
    // query pose and signals, if far, return false
    QueryPoseAndGates(timestamp, frame->lidar2world_pose, &hdmap_gates);

    PERF_BLOCK("barrier_recognition")
    for (auto& hdmap_gate : hdmap_gates) {
        std::vector<double> world_polygon;
        GetGatesPolygon(hdmap_gate, world_polygon);
        BarrierRecognizerOptions detector_options;
        detector_options.name = "StraightPoleRecognizer";
        detector_options.world_roi_polygon = world_polygon;

        float open_percent = 0.;
        if (!barrier_bank_.Recognize(detector_options, frame, open_percent)) {
            AINFO << "barrier recognize error.";
            return false;
        }
        int status_id = 0;
        if (!status_tracker_.Track(timestamp, open_percent, status_id)) {
            AINFO << "barrier track error.";
            return false;
        }

        BarrieGate obj = {status_id, hdmap_gate.id().id(), 
                          kBarrierId2Type[static_cast<int>(hdmap_gate.type())],
                          open_percent};
        gates.emplace_back(obj);
    }
    PERF_BLOCK_END

    return true;
}

bool BarrierRecognitionComponent::QueryPoseAndGates(
              const double ts, const Eigen::Affine3d& pose,
              std::vector<apollo::hdmap::BarrierGate>* gates) {
    if (!hd_map_) {
        AERROR << "hd_map_ not init.";
        return false;
    }

    // get gates
    Eigen::Vector3d global_tranlation = pose.translation();
    if (!hd_map_->GetBarrierGates(global_tranlation, 
                                  forward_distance_to_queries_,
                                  gates)) {
        if (ts - last_gates_ts_ < valid_hdmap_interval_) {
            *gates = last_gates_;
            AWARN << "query pose and gates failed to get gates info. "
                  << "Now use last info. ts:" << ts 
                  << " pose:" << global_tranlation
                  << " gates.size(): " << gates->size();
        } else {
            AERROR << "query pose and gates failed to get gates info. "
                  << "ts:" << ts << " pose:" << global_tranlation;
            return true;
        }
    } else {
        AINFO << "query pose and gates succeeded, gates.size(): "
              << gates->size();
        last_gates_ts_ = ts;
        last_gates_ = *gates;
    }
    return true;
}

void BarrierRecognitionComponent::GetGatesPolygon(
    const apollo::hdmap::BarrierGate& barrier_gate,
    std::vector<double>& world_polygon) {
    world_polygon.clear();
    for (int i = 0; i < barrier_gate.polygon().point_size(); i+=2) {
        world_polygon.push_back(barrier_gate.polygon().point(i).x());
        world_polygon.push_back(barrier_gate.polygon().point(i).y());
    }
}

bool BarrierRecognitionComponent::MakeProtobufMsg(
    double msg_timestamp, int seq_num,
    std::vector<BarrieGate> gates,
    PerceptionBarrierGate *msg) {
    double publish_time = apollo::cyber::Clock::NowInSeconds();
    auto header = msg->mutable_header();
    header->set_timestamp_sec(publish_time);
    header->set_module_name("perception_lidar");
    header->set_sequence_num(seq_num);
    // in nanosecond
    // PnC would use lidar timestamp to predict
    header->set_lidar_timestamp(static_cast<uint64_t>(msg_timestamp * 1e9));
    header->set_camera_timestamp(static_cast<uint64_t>(msg_timestamp * 1e9));

    for (const auto &obj : gates) {
        BarrierGate *gate = msg->add_barrier_gates();
        gate->set_status(static_cast<BarrierGate::Status>(obj.status_id));
        gate->set_id(obj.id);
        gate->set_type(obj.type);
        gate->set_is_useable(true);
        gate->set_open_percent(obj.open_percent);
    }

    return true;
}


}  // namespace lidar
}  // namespace perception
}  // namespace apollo
