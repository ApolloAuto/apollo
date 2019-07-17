/******************************************************************************
 * Copyright 2019 The Apollo Authors. All Rights Reserved.
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

#include <Eigen/Core>
#include <Eigen/Geometry>

namespace apollo {
namespace localization {
namespace msf {

typedef Eigen::Matrix4f Matrix4f;
typedef Eigen::Matrix3f Matrix3f;
typedef Eigen::Affine3f Affine3f;
typedef Eigen::Vector3f Vector3f;
typedef Eigen::Vector2f Vector2f;

typedef Eigen::Matrix4d Matrix4d;
typedef Eigen::Matrix3d Matrix3d;
typedef Eigen::Affine3d Affine3d;
typedef Eigen::Vector3d Vector3d;
typedef Eigen::Vector2d Vector2d;

/**@brief The options of the reflectance map. */
class BaseMapConfig;

/**@brief The data structure of the base map. */
class BaseMap;

/**@brief The data structure of the map cells in a map node. */
class BaseMapMatrix;

/**@brief The data structure of a Node in the map. */
class BaseMapNode;

class MapNodeIndex;

/**@brief The memory pool for the data structure of BaseMapNode. */
class BaseMapNodePool;

}  // namespace msf
}  // namespace localization
}  // namespace apollo
