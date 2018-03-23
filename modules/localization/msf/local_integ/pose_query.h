/******************************************************************************
 * Copyright 2017 The Apollo Authors. All Rights Reserved.
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

/**
 * @file pose_query.h
 * @brief The class of PoseQuery
 */

#ifndef MODULES_LOCALIZATION_MSF_POSE_QUERY_H_
#define MODULES_LOCALIZATION_MSF_POSE_QUERY_H_

#include <Eigen/Core>
#include <Eigen/Geometry>
// #include "modules/localization/proto/localization.pb.h"
// #include "modules/localization/proto/sins_pva.pb.h"
#include <mutex>
#include <iostream>
#include "glog/logging.h"
#include "glog/raw_logging.h"

namespace apollo {
namespace localization {

typedef Eigen::Affine3d TransformD;
typedef Eigen::Quaterniond QuaternionD;
typedef Eigen::Vector3d Vector3D;
typedef Eigen::Matrix3d MatrixDcm;

struct PoseForQuery {
public:
    PoseForQuery() : measure_time(0), translation(Vector3D::Zero()), quaternion(1, 0, 0, 0) {}
    PoseForQuery(double time, Vector3D trans, QuaternionD quat)
            : measure_time(time), translation(trans), quaternion(quat) {}
public:
    double measure_time;
    Vector3D translation;
    QuaternionD quaternion;
};

struct PoseQuery {
public:
    PoseQuery() {
        buffer_max_size = 400;
        buffer_size = 0;
    }

    ~PoseQuery() {}

    void add_pose(double time, Vector3D trans, QuaternionD quat) {
        std::unique_lock<std::mutex> lock(pose_mutex);

        if (time < pose_buffer.back().measure_time + 1e-6) {
            return;
        }

        PoseForQuery pose(time, trans, quat);
        pose_buffer.push_back(pose);
        ++buffer_size;
        if (buffer_size > buffer_max_size) {
            pose_buffer.pop_front();
            --buffer_size;
        }
        return;
    }

    bool query_quaternion(double time, QuaternionD& quat) {
        std::unique_lock<std::mutex> lock(pose_mutex);

        if (buffer_size < 2) {
            return false;
        }

        auto itr_last = pose_buffer.begin();
        auto itr = itr_last;
        ++itr;
        auto itr_end = pose_buffer.end();

        // query time is too old
        if (time < itr_last->measure_time) {
            LOG(WARNING) << std::setprecision(20) 
                    << "query time is too old, query time: " << time;
            return false;
        }

        // query time is OK
        bool found_poses = false;
        for (; itr != itr_end; ++itr, ++itr_last) {
            double time0 = itr_last->measure_time;
            double time1 = itr->measure_time;
            if (time0 <= time && time <= time1) {
                double ratio = (time - time0) / (time1 - time0); // add_pose avoid /0
                quat = itr_last->quaternion.slerp(ratio, itr->quaternion);
                return true;
            }
        }

        // query time is too new
        if (time < itr_last->measure_time + 0.02) {
            quat = itr_last->quaternion;
        }
        
        LOG(WARNING) << std::setprecision(20) 
                << "query time is too new, query time: " << time;
        return false;
    }

protected:
    std::list<PoseForQuery> pose_buffer;
    int buffer_size;
    int buffer_max_size;
    std::mutex pose_mutex;
};

}  // namespace localization
}  // namespace apollo

#endif  // MODULES_LOCALIZATION_MSF_POSE_QUERY_H_
