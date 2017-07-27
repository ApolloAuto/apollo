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

#ifndef CAR_ROS_DRIVERS_VELODYNE_VELODYNE_POINTCLOUD_TESTS_VELODYNE_UNIT_TEST_H
#define CAR_ROS_DRIVERS_VELODYNE_VELODYNE_POINTCLOUD_TESTS_VELODYNE_UNIT_TEST_H

#include <gtest/gtest.h>
#include <pcl/PCLPointField.h>
#include <sensor_msgs/PointCloud2.h>
#include <stdint.h>
#include <boost/filesystem.hpp>
#include <boost/filesystem/operations.hpp>

#include "velodyne_pointcloud/util.h"
#include "velodyne_pointcloud/velodyne_parser.h"

namespace apollo {
namespace drivers {
namespace velodyne {

void compare_out_msg(const VPointCloud& m1, const VPointCloud& m2) {
  EXPECT_EQ(m1.size(), m2.size()) << "size of messages";
  EXPECT_STREQ(m1.header.frame_id.c_str(), m2.header.frame_id.c_str());
  EXPECT_EQ(m1.header.seq, m2.header.seq);
  //    EXPECT_EQ(m1.header.stamp, m2.header.stamp);
  EXPECT_EQ(m1.height, m2.height);
  EXPECT_EQ(m1.width, m2.width);
  EXPECT_EQ(m1.is_dense, m2.is_dense);
  EXPECT_EQ(m1.isOrganized(), m2.isOrganized());

  int index = 0;
  pcl::PointCloud<VPoint>::const_iterator it1;
  pcl::PointCloud<VPoint>::const_iterator it2;

  for (it1 = m1.begin(), it2 = m2.begin(); it1 != m1.end(), it2 != m2.end();
       ++it1, ++it2, ++index) {
    // EXPECT_DOUBLE_EQ(it1->timestamp, it2->timestamp) << "The iterator index
    // is " << index;
    EXPECT_EQ(it1->intensity, it2->intensity) << "The iterator index is "
                                              << index;

    if (std::isnan(it1->x)) {
      // if x is nan then we can know this point is a NaNPoint
      EXPECT_TRUE(std::isnan(it2->x)) << "The iterator index is " << index;
    } else {
      EXPECT_NEAR(it1->x, it2->x, 0.001) << "The iterator index is " << index;
      EXPECT_NEAR(it1->y, it2->y, 0.001) << "The iterator index is " << index;
      EXPECT_NEAR(it1->z, it2->z, 0.001) << "The iterator index is " << index;
    }
  }
}

void compare_pcl_pointnormal_msg(const pcl::PointCloud<pcl::PointNormal>& m1,
                                 const pcl::PointCloud<pcl::PointNormal>& m2) {
  EXPECT_EQ(m1.points.size(), m2.points.size());
  EXPECT_EQ(m1.height, m2.height);
  EXPECT_EQ(m1.width, m2.width);
  EXPECT_EQ(m1.is_dense, m2.is_dense);

  // TODO: will use a new datastruct with interface to get offset,
  // datatype,datasize...
  float abs_error = 0.1;
  for (int i = 0; i < m1.points.size(); ++i) {
    pcl::PointNormal point1 = m1.points.at(i);
    pcl::PointNormal point2 = m2.points.at(i);
    EXPECT_EQ(point1.x, point2.x);
    EXPECT_EQ(point1.y, point2.y);
    EXPECT_EQ(point1.z, point2.z);
    EXPECT_NEAR(point1.normal_x, point2.normal_x, abs_error);
    EXPECT_NEAR(point1.normal_y, point2.normal_y, abs_error);
    EXPECT_NEAR(point1.normal_z, point2.normal_z, abs_error);
    EXPECT_NEAR(point1.curvature, point2.curvature, abs_error);
  }
}

template <typename PointT>
void compare_pcl_pointcloud_msg(typename pcl::PointCloud<PointT>::Ptr& m1,
                                typename pcl::PointCloud<PointT>::Ptr& m2) {
  EXPECT_EQ(m1->points.size(), m2->points.size());
  EXPECT_EQ(m1->height, m2->height);
  EXPECT_EQ(m1->width, m2->width);
  EXPECT_EQ(m1->is_dense, m2->is_dense);

  //    int total = m1.width * m1.height;
  //
  //    int x_offset = 0;
  //    int y_offset = 0;
  //    int z_offset = 0;

  // TODO: will use a new datastruct with interface to get offset,
  // datatype,datasize...
  for (int i = 0; i < m1->points.size(); ++i) {
    PointT point1 = m1->points.at(i);
    PointT point2 = m2->points.at(i);
    EXPECT_EQ(point1.x, point2.x);
    EXPECT_EQ(point1.y, point2.y);
    EXPECT_EQ(point1.z, point2.z);
  }
}

int get_time() {
  struct timeval tp;
  gettimeofday(&tp, NULL);
  long int ms = tp.tv_sec * 1000 + tp.tv_usec / 1000;
  return ms;
}

void compare_pcl_pclpointcloud2_msg(pcl::PCLPointCloud2::Ptr& m1,
                                    pcl::PCLPointCloud2::Ptr& m2,
                                    bool compute_normal) {
  EXPECT_EQ(m1->fields.size(), m2->fields.size());
  EXPECT_EQ(m1->data.size(), m2->data.size());
  EXPECT_EQ(m1->height, m2->height);
  EXPECT_EQ(m1->width, m2->width);
  EXPECT_EQ(m1->is_dense, m2->is_dense);
  EXPECT_EQ(m1->point_step, m2->point_step);
  EXPECT_EQ(m1->row_step, m2->row_step);
  EXPECT_EQ(m1->is_bigendian, m2->is_bigendian);

  for (int i = 0; i < m1->fields.size(); ++i) {
    pcl::PCLPointField point1 = m1->fields.at(i);
    pcl::PCLPointField point2 = m2->fields.at(i);
    EXPECT_EQ(point1.offset, point2.offset);
    EXPECT_EQ(point1.datatype, point2.datatype);
    EXPECT_EQ(point1.count, point2.count);
  }

  if (compute_normal) {
    pcl::PointCloud<pcl::PointNormal>::Ptr cloud1(
        new pcl::PointCloud<pcl::PointNormal>);

    pcl::PointCloud<pcl::PointNormal>::Ptr cloud2(
        new pcl::PointCloud<pcl::PointNormal>);

    pcl::fromPCLPointCloud2(*m1, *cloud1);
    pcl::fromPCLPointCloud2(*m2, *cloud2);
    for (int i = 0; i < cloud1->points.size(); ++i) {
      pcl::PointNormal point1 = cloud1->points.at(i);
      pcl::PointNormal point2 = cloud2->points.at(i);
      EXPECT_EQ(point1.x, point2.x);
      EXPECT_EQ(point1.y, point2.y);
      EXPECT_EQ(point1.z, point2.z);
      EXPECT_EQ(point1.normal_x, point2.normal_x);
      EXPECT_EQ(point1.normal_y, point2.normal_y);
      EXPECT_EQ(point1.normal_z, point2.normal_z);
      EXPECT_EQ(point1.curvature, point2.curvature);
    }
  } else {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud1(
        new pcl::PointCloud<pcl::PointXYZ>);

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud2(
        new pcl::PointCloud<pcl::PointXYZ>);

    pcl::fromPCLPointCloud2(*m1, *cloud1);
    pcl::fromPCLPointCloud2(*m2, *cloud2);
    for (int i = 0; i < cloud1->points.size(); ++i) {
      pcl::PointXYZ point1 = cloud1->points.at(i);
      pcl::PointXYZ point2 = cloud2->points.at(i);
      EXPECT_EQ(point1.x, point2.x);
      EXPECT_EQ(point1.y, point2.y);
      EXPECT_EQ(point1.z, point2.z);
    }
  }
}

void compare_pointcloud2_msg(const sensor_msgs::PointCloud2& m1,
                             const sensor_msgs::PointCloud2& m2) {
  EXPECT_EQ(m1.header.seq, m2.header.seq);
  EXPECT_EQ(m1.header.stamp, m2.header.stamp);
  EXPECT_EQ(m1.height, m2.height);
  EXPECT_EQ(m1.width, m2.width);
  EXPECT_EQ(m1.is_dense, m2.is_dense);
  EXPECT_EQ(m1.point_step, m2.point_step);
  EXPECT_EQ(m1.row_step, m2.row_step);

  int total = m1.width * m1.height;

  int x_offset = 0;
  int y_offset = 0;
  int z_offset = 0;

  // TODO: will use a new datastruct with interface to get offset,
  // datatype,datasize...
  for (size_t i = 0; i < m1.fields.size(); ++i) {
    const sensor_msgs::PointField& f = m1.fields[i];

    if (f.name == "x") {
      x_offset = f.offset;
    }

    if (f.name == "y") {
      y_offset = f.offset;
    }

    if (f.name == "z") {
      z_offset = f.offset;
    }
  }

  int point_step = m1.point_step;

  for (int index = 0; index < total; ++index) {
    float m1x = *(float*)&m1.data[index * point_step + x_offset];
    float m2x = *(float*)&m2.data[index * point_step + x_offset];
    float m1y = *(float*)&m1.data[index * point_step + y_offset];
    float m2y = *(float*)&m2.data[index * point_step + y_offset];
    float m1z = *(float*)&m1.data[index * point_step + z_offset];
    float m2z = *(float*)&m2.data[index * point_step + z_offset];
    if (std::isnan(m1x)) {
      EXPECT_TRUE(std::isnan(m2x)) << "The iterator index is " << index;
      continue;
    }
    EXPECT_NEAR(m1x, m2x, 1e-5);
    EXPECT_NEAR(m1y, m2y, 1e-5);
    EXPECT_NEAR(m1z, m2z, 1e-5);
    /*EXPECT_EQ(m1.data[index * point_step + x_offset],
              m2.data[index * point_step + x_offset]) << "The index is " <<
    index;
    EXPECT_EQ(m1.data[index * point_step + y_offset],
              m2.data[index * point_step + y_offset]) << "The index is " <<
    index;
    EXPECT_EQ(m1.data[index * point_step + z_offset],
              m2.data[index * point_step + z_offset]) << "The index is " <<
    index;
              */
  }
}

void remove_tmp_files(const std::string& tmp_file_folder,
                      const std::string& prefix) {
  std::cout << tmp_file_folder << std::endl;
  boost::filesystem::path path(tmp_file_folder);
  if (boost::filesystem::exists(path)) {
    boost::filesystem::directory_iterator it(path);
    boost::filesystem::directory_iterator it_end;
    int prefix_len = prefix.size();
    for (; it != it_end; ++it) {
      if (it->path().leaf().string().compare(0, prefix_len, prefix, 0,
                                             prefix_len) == 0) {
        std::cout << "remove tmp msg file: " << it->path().leaf().string()
                  << std::endl;
        boost::filesystem::remove(it->path());
      }
    }
  }
}

}  // namespace velodyne
}  // namespace drivers
}  // namespace apollo

#endif /* CAR_ROS_DRIVERS_VELODYNE_VELODYNE_POINTCLOUD_TESTS_VELODYNE_UNIT_TEST_H \
          */
