/*********************************************************************************************************************
Copyright (c) 2020 RoboSense
All rights reserved

By downloading, copying, installing or using the software you agree to this license. If you do not agree to this
license, do not download, install, copy or use the software.

License Agreement
For RoboSense LiDAR SDK Library
(3-clause BSD License)

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the
following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following
disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following
disclaimer in the documentation and/or other materials provided with the distribution.

3. Neither the names of the RoboSense, nor Suteng Innovation Technology, nor the names of other contributors may be used
to endorse or promote products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES,
INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*********************************************************************************************************************/

#pragma once
#include "modules/drivers/lidar/rslidar/common/common_header.h"

namespace apollo {
namespace drivers {
namespace robosense {

template <typename PointT>
#ifdef _MSC_VER
struct __declspec(align(16)) PointCloudMsg
#elif __GNUC__
struct __attribute__((aligned(16))) PointCloudMsg
#endif
{
  typedef std::vector<PointT> PointCloud;
  typedef std::shared_ptr<PointCloud> PointCloudPtr;
  typedef std::shared_ptr<const PointCloud> PointCloudConstPtr;
  double timestamp = 0.0;
  std::string frame_id = "";      ///< Point cloud frame id
  uint32_t seq = 0;               ///< Sequence number of message
  uint32_t height = 0;            ///< Height of point cloud
  uint32_t width = 0;             ///< Width of point cloud
  bool is_dense = false;          ///< If is_dense=true, the point cloud does not contain NAN points
  PointCloudPtr point_cloud_ptr;  ///< Point cloud pointer
  PointCloudMsg() = default;
  explicit PointCloudMsg(const PointCloudPtr& ptr) : point_cloud_ptr(ptr)
  {
  }
  typedef std::shared_ptr<PointCloudMsg> Ptr;
  typedef std::shared_ptr<const PointCloudMsg> ConstPtr;
};

}  // namespace robosense
}  // namespace drivers
}  // namespace apollo