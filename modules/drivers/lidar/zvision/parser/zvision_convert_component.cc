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

#include "modules/drivers/lidar/zvision/parser/zvision_convert_component.h"

namespace apollo
{
  namespace drivers
  {
    namespace zvision
    {

      using apollo::cyber::Component;
      using apollo::cyber::Reader;
      using apollo::cyber::Writer;
      using apollo::cyber::base::CCObjectPool;
      using apollo::drivers::PointCloud;
      using apollo::drivers::zvision::ZvisionScan;

      bool ZvisionConvertComponent::Init()
      {
        Config zvision_config;
        if (!GetProtoConfig(&zvision_config))
        {
          AWARN << "Load config failed, config file" << config_file_path_;
          return false;
        }

        conv_.reset(new Convert());
        conv_->init(zvision_config);
        writer_ =
            node_->CreateWriter<PointCloud>(zvision_config.pointcloud_channel());
        // printf("zvision_config.pointcloud_channel() =  %d \n", zvision_config.pointcloud_channel());
        point_cloud_pool_.reset(new CCObjectPool<PointCloud>(pool_size_));
        point_cloud_pool_->ConstructAll();
        for (int i = 0; i < pool_size_; i++)
        {
          auto point_cloud = point_cloud_pool_->GetObject();
          if (point_cloud == nullptr)
          {
            AERROR << "fail to getobject, i: " << i;
            return false;
          }
          point_cloud->mutable_point()->Reserve(140000);
        }
        AINFO << "Point cloud comp convert init success";
        return true;
      }

      bool ZvisionConvertComponent::Proc(const std::shared_ptr<ZvisionScan> &scan_msg)
      {
        std::shared_ptr<PointCloud> point_cloud_out = point_cloud_pool_->GetObject();
        // printf("point_cloud_out =  %u \n", point_cloud_out);
        if (point_cloud_out == nullptr)
        {
          AWARN << "poin cloud pool return nullptr, will be create new.";
          point_cloud_out = std::make_shared<PointCloud>();
          point_cloud_out->mutable_point()->Reserve(140000);
        }
        if (point_cloud_out == nullptr)
        {
          AWARN << "point cloud out is nullptr";
          return false;
        }
        // if (!conv_->CalibrationInitOk())
        // {
        //   AERROR << "calibration is not available.";
        //   return true;
        // }

        point_cloud_out->Clear();
        conv_->ConvertPacketsToPointcloud(scan_msg, point_cloud_out);
      
        if (point_cloud_out == nullptr || point_cloud_out->point().empty())
        {
          if (conv_->CalibrationInitOk())
            AWARN << "point_cloud_out convert is empty.";
          return false;
        }
        // printf("publish zvision lidar points.\n");
        writer_->Write(point_cloud_out);
        return true;
      }

    } // namespace zvision
  } // namespace drivers
} // namespace apollo
