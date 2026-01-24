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

#include "modules/drivers/lidar_fusion_and_compensator/fusion_and_compensator_component.h"

#include <climits>
#include <cstdlib>
#include <memory>
#include <thread>

namespace apollo {
namespace drivers {
namespace lidar {

using apollo::cyber::Time;

bool FusionAndCompensatorComponent::Init() {
    if (!GetProtoConfig(&conf_)) {
        AWARN << "Load config failed, config file" << ConfigFilePath();
        return false;
    }
    tf2_buffer_ptr_ = apollo::transform::Buffer::Instance();

    writer_ = node_->CreateWriter<PointCloud>(conf_.output_channel());

    for (const auto& channel : conf_.input_channel()) {
        auto reader = node_->CreateReader<PointCloud>(channel);

        readers_.emplace_back(reader);
    }
    for (const auto& filter_config : conf_.filter_config()) {
        filter_map_.emplace(filter_config.frame_id(), &filter_config);
    }
    return true;
}

bool FusionAndCompensatorComponent::Proc(const std::shared_ptr<PointCloud>& point_cloud) {
    auto target = writer_->AcquireMessage();
    target->Clear();
    target->mutable_point()->Reserve(reserved_point_size_);
    target->mutable_header()->set_lidar_timestamp(point_cloud->header().lidar_timestamp());
    if (conf_.has_target_frame_id()) {
        target->mutable_header()->set_frame_id(conf_.target_frame_id());
    } else {
        target->mutable_header()->set_frame_id(point_cloud->header().frame_id());
    }

    auto start_time = Time::Now().ToSecond();
    std::vector<std::shared_ptr<PointCloud>> point_clouds = {point_cloud};
    auto fusion_readers = readers_;
    while ((Time::Now().ToSecond() - start_time) < conf_.wait_time_s() && fusion_readers.size() > 0) {
        for (auto itr = fusion_readers.begin(); itr != fusion_readers.end();) {
            (*itr)->Observe();
            if (!(*itr)->Empty()) {
                auto source = (*itr)->GetLatestObserved();
                if (conf_.drop_expired_data() && IsExpired(target, source)) {
                    ++itr;
                } else {
                    point_clouds.emplace_back(source);
                    itr = fusion_readers.erase(itr);
                }
            } else {
                ++itr;
            }
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }

    auto diff = Time::Now().ToNanosecond() - target->header().lidar_timestamp();
    AINFO << "Pointcloud fusion diff: " << diff / 1000000 << "ms";

    uint64_t timestamp_min = 0;
    uint64_t timestamp_max = 0;
    GetTimestampInterval(point_clouds, &timestamp_min, &timestamp_max);

    for (const auto& pc : point_clouds) {
        if (!FusionAndCompensator(pc, timestamp_min, timestamp_max, target)) {
            return false;
        }
    }

    target->mutable_header()->set_sequence_num(seq_num_++);
    target->mutable_header()->set_timestamp_sec(cyber::Time::Now().ToSecond());
    target->set_height(point_cloud->height());
    target->set_width(target->point_size() / target->height());
    target->set_is_dense(point_cloud->is_dense());
    target->set_measurement_time(point_cloud->measurement_time());

    writer_->Write(target);
    return true;
}

void FusionAndCompensatorComponent::GetTimestampInterval(
        const std::vector<std::shared_ptr<PointCloud>>& point_clouds,
        uint64_t* timestamp_min,
        uint64_t* timestamp_max) {
    *timestamp_max = 0;
    *timestamp_min = std::numeric_limits<uint64_t>::max();
    for (const auto& pc : point_clouds) {
        for (const auto& point : pc->point()) {
            if (point.timestamp() < *timestamp_min) {
                *timestamp_min = point.timestamp();
            }

            if (point.timestamp() > *timestamp_max) {
                *timestamp_max = point.timestamp();
            }
        }
    }
}

bool FusionAndCompensatorComponent::IsExpired(
        const std::shared_ptr<PointCloud>& target,
        const std::shared_ptr<PointCloud>& source) {
    auto diff = target->measurement_time() - source->measurement_time();
    return diff * 1000 > conf_.max_interval_ms();
}

bool FusionAndCompensatorComponent::QueryPoseAffineFromTF2(
        const uint64_t& timestamp,
        const std::string& target_frame_id,
        const std::string& source_frame_id,
        Eigen::Affine3d* pose) {
    cyber::Time query_time(timestamp);
    std::string err_string;
    if (!tf2_buffer_ptr_->canTransform(
                target_frame_id, source_frame_id, query_time, conf_.transform_query_timeout(), &err_string)) {
        AERROR << "Can not find transform, target_frame_id: " << target_frame_id
               << ", source_frame_id: " << source_frame_id << ", Error info: " << err_string;
        return false;
    }

    apollo::transform::TransformStamped stamped_transform;
    try {
        stamped_transform = tf2_buffer_ptr_->lookupTransform(target_frame_id, source_frame_id, query_time);
    } catch (tf2::TransformException& ex) {
        AERROR << ex.what();
        return false;
    }

    *pose = Eigen::Translation3d(
                    stamped_transform.transform().translation().x(),
                    stamped_transform.transform().translation().y(),
                    stamped_transform.transform().translation().z())
            * Eigen::Quaterniond(
                    stamped_transform.transform().rotation().qw(),
                    stamped_transform.transform().rotation().qx(),
                    stamped_transform.transform().rotation().qy(),
                    stamped_transform.transform().rotation().qz());
    return true;
}

bool FusionAndCompensatorComponent::FusionAndCompensator(
        const std::shared_ptr<PointCloud> source,
        const uint64_t& timestamp_min,
        const uint64_t& timestamp_max,
        std::shared_ptr<PointCloud> target) {
    auto& target_frame_id = target->header().frame_id();
    auto& source_frame_id = source->header().frame_id();
    // get static tf
    auto lidar_tf_f = static_tf_map_.find(source_frame_id);
    if (lidar_tf_f == static_tf_map_.end()) {
        Eigen::Affine3d lidar_tf_d;
        if (!QueryPoseAffineFromTF2(0, target_frame_id, source_frame_id, &lidar_tf_d)) {
            return false;
        }
        static_tf_map_[source_frame_id] = lidar_tf_d.cast<float>();
    }

    // compensator
    Eigen::Affine3d world_tf_min_time;
    Eigen::Affine3d world_tf_max_time;
    if (!QueryPoseAffineFromTF2(timestamp_min, conf_.world_frame_id(), target_frame_id, &world_tf_min_time)
        || !QueryPoseAffineFromTF2(timestamp_max, conf_.world_frame_id(), target_frame_id, &world_tf_max_time)) {
        AERROR << "compensator transform query error.";
        return false;
    }
    MotionCompensation(
            source,
            timestamp_min,
            timestamp_max,
            world_tf_min_time,
            world_tf_max_time,
            static_tf_map_[source_frame_id],
            target);

    return true;
}

bool FusionAndCompensatorComponent::IsFilteredPoint(const Eigen::Vector3f& point, const std::string& frame_id) const {
    if (filter_map_.find(frame_id) == filter_map_.end()) {
        return false;
    }
    const auto& x = point.x();
    const auto& y = point.y();
    const auto& z = point.z();
    const auto& filter = filter_map_.at(frame_id);
    if (filter->has_min_x() && x < filter->min_x()) {
        return false;
    }
    if (filter->has_max_x() && x > filter->max_x()) {
        return false;
    }
    if (filter->has_min_y() && y < filter->min_y()) {
        return false;
    }
    if (filter->has_max_y() && y > filter->max_y()) {
        return false;
    }
    if (filter->has_min_z() && z < filter->min_z()) {
        return false;
    }
    if (filter->has_max_z() && z > filter->max_z()) {
        return false;
    }
    return true;
}

void FusionAndCompensatorComponent::MotionCompensation(
        const std::shared_ptr<PointCloud>& source,
        const uint64_t& timestamp_min,
        const uint64_t& timestamp_max,
        const Eigen::Affine3d& world_tf_min_time,
        const Eigen::Affine3d& world_tf_max_time,
        const Eigen::Affine3f& lidar_tf,
        std::shared_ptr<PointCloud> target) {
    Eigen::Vector3d translation_d = world_tf_min_time.translation() - world_tf_max_time.translation();
    Eigen::Quaterniond q_max(world_tf_max_time.linear());
    Eigen::Quaterniond q_min(world_tf_min_time.linear());
    Eigen::Quaterniond q1_d(q_max.conjugate() * q_min);
    Eigen::Quaterniond q0_d(Eigen::Quaterniond::Identity());
    Eigen::Quaternionf q1 = q1.cast<float>();
    Eigen::Quaternionf q0 = q0_d.cast<float>();
    q1.normalize();
    Eigen::Vector3f translation = (q_max.conjugate() * translation_d).cast<float>();

    double d = q0.dot(q1);
    double abs_d = std::abs(d);
    float f = 1.0 / static_cast<float>(timestamp_max - timestamp_min);

    // Threshold for a "significant" rotation from min_time to max_time:
    // The LiDAR range accuracy is ~2 cm. Over 70 meters range, it means an angle
    // of 0.02 / 70 =
    // 0.0003 rad. So, we consider a rotation "significant" only if the scalar
    // part of quaternion is
    // less than cos(0.0003 / 2) = 1 - 1e-8.
    if (conf_.rotation_compensation() && abs_d < 1.0 - 1.0e-8) {
        float theta = static_cast<float>(std::acos(abs_d));
        float sin_theta = std::sin(theta);
        float c1_sign = (d > 0) ? 1 : -1;
        for (const auto& point : source->point()) {
            float x_scalar = point.x();
            float y_scalar = point.y();
            float z_scalar = point.z();
            if (std::isnan(x_scalar) || std::isnan(y_scalar) || std::isnan(z_scalar)) {
                // ignore nan point
                continue;
            }
            Eigen::Vector3f p(x_scalar, y_scalar, z_scalar);

            auto fusion_p = lidar_tf * p;
            if (IsFilteredPoint(fusion_p, source->header().frame_id())) {
                continue;
            }

            uint64_t tp = point.timestamp();
            float t = static_cast<float>(timestamp_max - tp) * f;

            Eigen::Translation3f ti(t * translation);

            float c0 = std::sin((1 - t) * theta) / sin_theta;
            float c1 = std::sin(t * theta) / sin_theta * c1_sign;
            Eigen::Quaternionf qi(c0 * q0.coeffs() + c1 * q1.coeffs());

            Eigen::Affine3f trans = ti * qi;
            p = trans * fusion_p;

            auto* point_new = target->add_point();
            point_new->set_intensity(point.intensity());
            point_new->set_timestamp(point.timestamp());
            point_new->set_x(p.x());
            point_new->set_y(p.y());
            point_new->set_z(p.z());
        }
    } else {
        // Not a "significant" rotation. Do translation only.
        for (auto& point : source->point()) {
            float x_scalar = point.x();
            float y_scalar = point.y();
            float z_scalar = point.z();
            if (std::isnan(x_scalar) || std::isnan(y_scalar) || std::isnan(z_scalar)) {
                // ignore nan point
                continue;
            }

            Eigen::Vector3f p(x_scalar, y_scalar, z_scalar);

            auto fusion_p = lidar_tf * p;
            if (IsFilteredPoint(fusion_p, source->header().frame_id())) {
                continue;
            }

            if (conf_.translation_compensation()) {
                uint64_t tp = point.timestamp();
                float t = static_cast<float>(timestamp_max - tp) * f;
                Eigen::Translation3f ti(t * translation);
                p = ti * fusion_p;
            } else {
                p = fusion_p;
            }

            auto* point_new = target->add_point();
            point_new->set_intensity(point.intensity());
            point_new->set_timestamp(point.timestamp());
            point_new->set_x(p.x());
            point_new->set_y(p.y());
            point_new->set_z(p.z());
        }
    }
}

}  // namespace lidar
}  // namespace drivers
}  // namespace apollo
