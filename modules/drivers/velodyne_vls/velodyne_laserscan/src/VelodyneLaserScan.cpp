#include "VelodyneLaserScan.h"
#include <sensor_msgs/point_cloud2_iterator.h>

namespace velodyne_laserscan {
  
VelodyneLaserScan::VelodyneLaserScan(ros::NodeHandle &nh, ros::NodeHandle &nh_priv) :
    ring_count_(0), nh_(nh), srv_(nh_priv)
{
  ros::SubscriberStatusCallback connect_cb = boost::bind(&VelodyneLaserScan::connectCb, this);
  pub_ = nh.advertise<sensor_msgs::LaserScan>("scan", 10, connect_cb, connect_cb);
  
  srv_.setCallback(boost::bind(&VelodyneLaserScan::reconfig, this, _1, _2));
}

void VelodyneLaserScan::connectCb()
{
  boost::lock_guard<boost::mutex> lock(connect_mutex_);
  if (!pub_.getNumSubscribers()) {
    sub_.shutdown();
  } else if (!sub_) {
    sub_ = nh_.subscribe("velodyne_points", 10, &VelodyneLaserScan::recvCallback, this);
  }
}

void VelodyneLaserScan::recvCallback(const sensor_msgs::PointCloud2ConstPtr& msg)
{
  // Latch ring count
  if (!ring_count_) {
    // Check for PointCloud2 field 'ring'
    bool found = false;
    for (size_t i = 0; i < msg->fields.size(); i++) {
      if (msg->fields[i].datatype == sensor_msgs::PointField::UINT16) {
        if (msg->fields[i].name == "ring") {
          found = true;
          break;
        }
      }
    }
    if (!found) {
      ROS_ERROR("VelodyneLaserScan: Field 'ring' of type 'UINT16' not present in PointCloud2");
      return;
    }
    for (sensor_msgs::PointCloud2ConstIterator<uint16_t> it(*msg, "ring"); it != it.end(); ++it) {
      const uint16_t ring = *it;
      if (ring + 1 > ring_count_) {
        ring_count_ = ring + 1;
      }
    }
    if (ring_count_) {
      ROS_INFO("VelodyneLaserScan: Latched ring count of %u", ring_count_);
    } else {
      ROS_ERROR("VelodyneLaserScan: Field 'ring' of type 'UINT16' not present in PointCloud2");
      return;
    }
  }

  // Select ring to use
  uint16_t ring;
  if ((cfg_.ring < 0) || (cfg_.ring >= ring_count_)) {
    // Default to ring closest to being level for each known sensor
    if (ring_count_ > 32) {
      ring = 57; // HDL-64E
    } else if (ring_count_ > 16) {
      ring = 23; // HDL-32E
    } else {
      ring = 8; // VLP-16
    }
  } else {
    ring = cfg_.ring;
  }
  ROS_INFO_ONCE("VelodyneLaserScan: Extracting ring %u", ring);

  // Load structure of PointCloud2
  int offset_x = -1;
  int offset_y = -1;
  int offset_z = -1;
  int offset_i = -1;
  int offset_r = -1;
  for (size_t i = 0; i < msg->fields.size(); i++) {
    if (msg->fields[i].datatype == sensor_msgs::PointField::FLOAT32) {
      if (msg->fields[i].name == "x") {
        offset_x = msg->fields[i].offset;
      } else if (msg->fields[i].name == "y") {
        offset_y = msg->fields[i].offset;
      } else if (msg->fields[i].name == "z") {
        offset_z = msg->fields[i].offset;
      } else if (msg->fields[i].name == "intensity") {
        offset_i = msg->fields[i].offset;
      }
    } else if (msg->fields[i].datatype == sensor_msgs::PointField::UINT16) {
      if (msg->fields[i].name == "ring") {
        offset_r = msg->fields[i].offset;
      }
    }
  }

  // Construct LaserScan message
  if ((offset_x >= 0) && (offset_y >= 0) && (offset_r >= 0)) {
    const float RESOLUTION = fabsf(cfg_.resolution);
    const size_t SIZE = 2.0 * M_PI / RESOLUTION;
    sensor_msgs::LaserScanPtr scan(new sensor_msgs::LaserScan());
    scan->header = msg->header;
    scan->angle_increment = RESOLUTION;
    scan->angle_min = -M_PI;
    scan->angle_max = M_PI;
    scan->range_min = 0.0;
    scan->range_max = 200.0;
    scan->time_increment = 0.0;
    scan->ranges.resize(SIZE, INFINITY);
    if ((offset_x == 0) && (offset_y == 4) && (offset_z == 8) && (offset_i == 16) && (offset_r == 20)) {
      scan->intensities.resize(SIZE);
      for (sensor_msgs::PointCloud2ConstIterator<float> it(*msg, "x"); it != it.end(); ++it) {
        const uint16_t r = *((const uint16_t*)(&it[5])); // ring
        if (r == ring) {
          const float x = it[0]; // x
          const float y = it[1]; // y
          const float i = it[4]; // intensity
          const int bin = (atan2f(y, x) + (float)M_PI) / RESOLUTION;
          if ((bin >= 0) && (bin < (int)SIZE)) {
            scan->ranges[bin] = sqrtf(x * x + y * y);
            scan->intensities[bin] = i;
          }
        }
      }
    } else {
      ROS_WARN_ONCE("VelodyneLaserScan: PointCloud2 fields in unexpected order. Using slower generic method.");
      if (offset_i >= 0) {
        scan->intensities.resize(SIZE);
        sensor_msgs::PointCloud2ConstIterator<uint16_t> iter_r(*msg, "ring");
        sensor_msgs::PointCloud2ConstIterator<float> iter_x(*msg, "x");
        sensor_msgs::PointCloud2ConstIterator<float> iter_y(*msg, "y");
        sensor_msgs::PointCloud2ConstIterator<float> iter_i(*msg, "intensity");
        for ( ; iter_r != iter_r.end(); ++iter_x, ++iter_y, ++iter_r, ++iter_i) {
          const uint16_t r = *iter_r; // ring
          if (r == ring) {
            const float x = *iter_x; // x
            const float y = *iter_y; // y
            const float i = *iter_i; // intensity
            const int bin = (atan2f(y, x) + (float)M_PI) / RESOLUTION;
            if ((bin >= 0) && (bin < (int)SIZE)) {
              scan->ranges[bin] = sqrtf(x * x + y * y);
              scan->intensities[bin] = i;
            }
          }
        }
      } else {
        sensor_msgs::PointCloud2ConstIterator<uint16_t> iter_r(*msg, "ring");
        sensor_msgs::PointCloud2ConstIterator<float> iter_x(*msg, "x");
        sensor_msgs::PointCloud2ConstIterator<float> iter_y(*msg, "y");
        for ( ; iter_r != iter_r.end(); ++iter_x, ++iter_y, ++iter_r) {
          const uint16_t r = *iter_r; // ring
          if (r == ring) {
            const float x = *iter_x; // x
            const float y = *iter_y; // y
            const int bin = (atan2f(y, x) + (float)M_PI) / RESOLUTION;
            if ((bin >= 0) && (bin < (int)SIZE)) {
              scan->ranges[bin] = sqrtf(x * x + y * y);
            }
          }
        }
      }
    }
    pub_.publish(scan);
  } else {
    ROS_ERROR("VelodyneLaserScan: PointCloud2 missing one or more required fields! (x,y,ring)");
  }
}

void VelodyneLaserScan::reconfig(VelodyneLaserScanConfig& config, uint32_t level)
{
  cfg_ = config;
}

}
