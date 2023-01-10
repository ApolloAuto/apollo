/******************************************************************************
 * Copyright 2020 The Apollo Authors. All Rights Reserved.
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

#include <deque>
#include <functional>
#include <map>
#include <utility>
#include <vector>

#include "Eigen/Geometry"

namespace apollo {
namespace common {

// Using STL Containers with Eigen:
// https://eigen.tuxfamily.org/dox/group__TopicStlContainers.html
template <class EigenType>
using EigenVector = std::vector<EigenType, Eigen::aligned_allocator<EigenType>>;

template <class EigenType>
using EigenDeque = std::deque<EigenType, Eigen::aligned_allocator<EigenType>>;

template <typename T, class EigenType>
using EigenMap = std::map<T, EigenType, std::less<T>,
    Eigen::aligned_allocator<std::pair<const T, EigenType>>>;

template <typename T, class EigenType>
using EigenMultiMap = std::multimap<T, EigenType, std::less<T>,
    Eigen::aligned_allocator<std::pair<const T, EigenType>>>;

using EigenVector3dVec = EigenVector<Eigen::Vector3d>;
using EigenAffine3dVec = EigenVector<Eigen::Affine3d>;

}  // namespace common
}  // namespace apollo
