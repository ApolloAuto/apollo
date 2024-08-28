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

#include "modules/drivers/lidar/zvision/parser/parser.h"

namespace apollo
{
  namespace drivers
  {
    namespace zvision
    {

      void Ml30Parser::GeneratePointcloud(
          const std::shared_ptr<ZvisionScan> &scan_msg,
          std::shared_ptr<PointCloud> out_msg)
      {
        // allocate a point cloud with same time and frame ID as raw data
        out_msg->mutable_header()->set_frame_id(scan_msg->header().frame_id());
        out_msg->set_height(1);
        out_msg->mutable_header()->set_sequence_num(
            scan_msg->header().sequence_num());

        size_t packets_size = scan_msg->firing_pkts_size();
        for (size_t i = 0; i < packets_size; ++i)
        {
          Unpack(scan_msg->firing_pkts(static_cast<int>(i)), out_msg);
        }

        if (out_msg->point().empty())
        {
          // we discard this pointcloud if empty
          // AERROR << "All points is NAN!Please check zvision:" << config_.model();
        }

        // set default width
        out_msg->set_width(out_msg->point_size());
      }

      /** @brief convert raw packet to point cloud
       *
       *  @param pkt raw packet to Unpack
       *  @param pc shared pointer to point cloud (points are appended)
       */
      void Ml30Parser::Unpack(const ZvisionPacket &pkt,
                              std::shared_ptr<PointCloud> pc)
      {
        // const RawPacket* raw = (const RawPacket*)&pkt.data[0];
        const unsigned char *data = (const unsigned char *)pkt.data().c_str();

        unsigned char value = data[1303];
        Model model = UNKNOWN;
        if (0xFF == value)
        {
          model = ML30B1;
        }
        else if (0x04 == value)
        {
          model = ML30SA1;
        }
        else if (0x05 == value)
        {
          model = MLX;
        }
        else
        {
          AERROR << "Unknown device type, flag is " << value;
          return;
        }

        // find lidar type
        if (model != cal_lut_.model)
        {
          AERROR << "Device type and calibration data is not matched.";
          return;
        }

        uint32_t seq = (unsigned char)(data[3]) + (((unsigned char)data[2] & 0xF) << 8);
        // UDP packet parameters
        unsigned int groups_in_one_udp = 0;
        unsigned int points_in_one_group = 0;
        unsigned int point_position_in_group = 0;
        unsigned int group_len = 0;
        unsigned int point_len = 4;
        unsigned int group_position_in_packet = 4;

        if (ML30B1 == model)
        {
          groups_in_one_udp = 80;
          points_in_one_group = 3;
          point_position_in_group = 4;
          group_len = 16;
          // if (30000 != cloud.points.size())
          //     cloud.points.resize(30000);
        }
        else if (ML30SA1 == model)
        {
          groups_in_one_udp = 40;
          points_in_one_group = 8;
          point_position_in_group = 0;
          group_len = 32;
          // if (51200 != cloud.points.size())
          //     cloud.points.resize(51200);
        }
        else if (MLX == model)
        {
          groups_in_one_udp = 40;
          points_in_one_group = 8;
          point_position_in_group = 0;
          group_len = 16;
          // if (96000 != cloud.points.size())
          //     cloud.points.resize(96000);
        }

        uint64_t udp_time_stamp_ns = GetUdpSensorTimestamp(data);
        uint64_t fire_time_interval_ns = 1.67 * 1000;       // 1.67 us
        for (uint32_t gp = 0; gp < groups_in_one_udp; ++gp) /*every groups*/
        {
          unsigned char *first_point_pos_in_group = (unsigned char *)data + group_position_in_packet + group_len * gp + point_position_in_group;
          uint32_t dis_low, dis_high, int_low, int_high; /*dis*/
          float distance = 0.0;
          int reflectivity = 0;
          for (uint32_t pt = 0; pt < points_in_one_group; ++pt)
          {
            unsigned char *point_pos = first_point_pos_in_group + point_len * pt;
            dis_high = point_pos[0];
            dis_low = point_pos[1];
            int_high = point_pos[2];
            int_low = point_pos[3];

            distance = static_cast<int>((((dis_high << 8) + dis_low) << 3) + (int)((int_high & 0xE0) >> 5));
            distance = distance * 0.0015f;
            reflectivity = (((int_high & 0x1F) << 8) + (int_low));
            reflectivity = (int)reflectivity & 0xFF;

            int point_number = seq * points_in_one_group * groups_in_one_udp + gp * points_in_one_group + pt;
            PointCalibrationData &point_cal = cal_lut_.data[point_number];
            PointXYZIT *point = pc->add_point();
            float x = distance * point_cal.cos_ele * point_cal.sin_azi;
            float y = distance * point_cal.cos_ele * point_cal.cos_azi;
            float z = distance * point_cal.sin_ele;
            Eigen::Vector3d tp(x, y, z);
            if (need_transform_)
            {
              tp = transform_ * tp;
            }

#if 1
            point->set_x(static_cast<float>(tp.x()));
            point->set_y(static_cast<float>(tp.y()));
            point->set_z(static_cast<float>(tp.z()));
#else
            point->set_x(x);
            point->set_y(y);
            point->set_z(z);
#endif

            point->set_intensity(reflectivity);
            point->set_timestamp(udp_time_stamp_ns + (gp * points_in_one_group + pt) * fire_time_interval_ns);
            // point->set_z(nan);

            if (point_number == 29999)
            {
              // ADEBUG << "point x:" << point->x() << "  y:" << point->y()
              //        << "  z:" << point->z()
              //        << "  intensity:" << int(point->intensity());
              // printf("point number[%6d], x = %.3f y = %.3f z = %.3f dis = % .3f ref = %d timestamp = %lu\n",point_number,
              //        point->x(), point->y(), point->z(), distance, (int)point->intensity(), point->timestamp());
              // printf("azi= %.3f ele=%.3f.\n", point_cal.azi / M_PI * 180.0, point_cal.ele / M_PI * 180.0);
            }
          }
        }

        return;
      }
    } // namespace velodyne
  } // namespace drivers
} // namespace apollo
