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

// The base class of transforming camera space objects into other defined
// 3D spaces, like world space or ego-car space

// void trans_object_to_world(const CameraTrackerOptions& options,
//                            const std::vector<VisualObjectPtr> *visual_objects) {
//     if (options.camera2world_pose == nullptr) {
//         LOG(INFO) << "can not trans object from camera to world";
//         return;
//     }
//     for (size_t i = 0; i < visual_objects->size(); ++i) {
//
//         float theta = (*visual_objects)[i]->theta;
//         Eigen::Vector4d dir(cos(theta), 0, -sin(theta), 0.0);
//
//         dir = (*options.camera2world_pose) * dir;
//         (*visual_objects)[i]->direction = dir.head(3);
//         (*visual_objects)[i]->theta = atan2(dir[1],dir[0]);
//
//         Eigen::Vector3d center = (*visual_objects)[i]->center;
//         Eigen::Vector4d ctr(center.x(), center.y(), center.z(), 1.0);
//         (*visual_objects)[i]->center = ((*options.camera2world_pose) * ctr).head(3);
//
//         XLOG(DEBUG) << "world center" << (*visual_objects)[i]->center;
//     }
// }
