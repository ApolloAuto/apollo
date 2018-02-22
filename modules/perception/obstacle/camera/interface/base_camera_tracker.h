/******************************************************************************
 * Copyright 2018 The Apollo Authors. All Rights Reserved.
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

// The base class of Id association for camera objects between frames
 
// #ifndef ADU_PERCEPTION_OBSTACLE_CAMERA_INTERFACE_BASE_CAMERA_TRACKER_H
// #define ADU_PERCEPTION_OBSTACLE_CAMERA_INTERFACE_BASE_CAMERA_TRACKER_H
//
//
// #include <string>
// #include <vector>
//
// #include <Eigen/Core>
// #include <opencv2/opencv.hpp>
// #include <boost/circular_buffer.hpp>
//
// #include "lib/base/noncopyable.h"
// #include "lib/base/registerer.h"
// #include "obstacle/camera/common/visual_object.h"
//
// namespace apollo {
// namespace perception {
//
// struct CameraTrackerOptions {
//     CameraTrackerOptions() = default;
//
//     explicit CameraTrackerOptions(Eigen::Matrix4d* pose) :
//         camera2world_pose(pose) {}
//
//     Eigen::Matrix4d* camera2world_pose = nullptr;
// };
//
// class BaseCameraTracker {
// public:
//     BaseCameraTracker() {}
//     virtual ~BaseCameraTracker() {}
//
//     virtual bool init() = 0;
//
//     // @brief: tracking objects.
//     // @param [in]: raw frame from camera.
//     // @param [in]: current frame object list.
//     // @param [in]: timestamp.
//     // @param [in]: options.
//     // @param [out]: current tracked objects.
//     virtual bool predict_velocity(const cv::Mat &frame,
//                                   const std::vector<VisualObjectPtr> &objects,
//                                   double timestamp,
//                                   const CameraTrackerOptions &options,
//                                   std::vector<VisualObjectPtr> *tracked_objects) = 0;
//
//     virtual bool predict_shape(const cv::Mat &frame,
//                                const std::vector<VisualObjectPtr> &objects,
//                                double timestamp,
//                                const CameraTrackerOptions &options,
//                                std::vector<VisualObjectPtr> *tracked_objects) = 0;
//
//     virtual bool associate(const cv::Mat &frame,
//                            const std::vector<VisualObjectPtr> &objects,
//                            double timestamp,
//                            const CameraTrackerOptions &options,
//                            std::vector<VisualObjectPtr> *tracked_objects) = 0;
//
//     void trans_object_to_world(const CameraTrackerOptions& options,
//                                const std::vector<VisualObjectPtr> *visual_objects) {
//         if (options.camera2world_pose == nullptr) {
//             LOG(INFO) << "can not trans object from camera to world";
//             return;
//         }
//         for (size_t i = 0; i < visual_objects->size(); ++i) {
//
//             float theta = (*visual_objects)[i]->theta;
//             Eigen::Vector4d dir(cos(theta), 0, -sin(theta), 0.0);
//
//             dir = (*options.camera2world_pose) * dir;
//             (*visual_objects)[i]->direction = dir.head(3);
//             (*visual_objects)[i]->theta = atan2(dir[1],dir[0]);
//
//             Eigen::Vector3d center = (*visual_objects)[i]->center;
//             Eigen::Vector4d ctr(center.x(), center.y(), center.z(), 1.0);
//             (*visual_objects)[i]->center = ((*options.camera2world_pose) * ctr).head(3);
//
//             XLOG(DEBUG) << "world center" << (*visual_objects)[i]->center;
//         }
//     }
//
//     bool smooth_center(std::vector<VisualObjectPtr> *visual_objects) {
//         for (auto objectptr : *visual_objects) {
//             int trackid = objectptr->track_id;
//             double min_depth = std::numeric_limits<double>::max();
//             if (_tracked_center_buffer.find(trackid) != _tracked_center_buffer.end()) {
//                 CenterBuffer& buffer = _tracked_center_buffer[trackid];
//                 buffer.push_back(objectptr->center[2]);
//                 int start = 0;
//                 for (CenterBuffer::iterator iter=buffer.begin(); iter!=buffer.end(); ++iter) {
//                      if (min_depth > (*iter)) {
//                         min_depth = (*iter);
//                      }
//                      ++start;
//                      if (start > LOOK_BACK_WINDOW) {
//                         break;
//                      }
//                 }
//                 objectptr->center[2] = min_depth;
//             }
//             else {
//                 CenterBuffer& buffer = _tracked_center_buffer[trackid];
//                 buffer.set_capacity(CENTER_BUFFER_SIZE);
//                 buffer.push_back((float)objectptr->center[2]);
//             }
//         }
//         return true;
//     }
//
//     virtual std::string name() const = 0;
//
// private:
//     DISALLOW_COPY_AND_ASSIGN(BaseCameraTracker);
//     typedef boost::circular_buffer<float> CenterBuffer;
//     std::unordered_map<int, CenterBuffer>  _tracked_center_buffer;
//     static int CenterBufferSize;
//     const static int CENTER_BUFFER_SIZE=10;
//     const static int LOOK_BACK_WINDOW=3;
// };
//
// REGISTER_REGISTERER(BaseCameraTracker);
// #define REGISTER_CAMERA_TRACKER(name) REGISTER_CLASS(BaseCameraTracker, name)
//
// }  // namespace obstacle
// }  // namespace perception
// }  // namespace adu
//
// #endif  // ADU_PERCEPTION_OBSTACLE_CAMERA_INTERFACE_BASE_CAMERA_TRACKER_H
