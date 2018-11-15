#ifndef VELODYNELASERSCAN_H
#define VELODYNELASERSCAN_H

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/LaserScan.h>

#include <boost/thread/mutex.hpp>
#include <boost/thread/lock_guard.hpp>

#include <dynamic_reconfigure/server.h>
#include <velodyne_laserscan/VelodyneLaserScanConfig.h>

namespace velodyne_laserscan {

class VelodyneLaserScan
{
public:
  VelodyneLaserScan(ros::NodeHandle &nh, ros::NodeHandle &nh_priv);

private:
  boost::mutex connect_mutex_;
  void connectCb();
  void recvCallback(const sensor_msgs::PointCloud2ConstPtr& msg);

  ros::NodeHandle nh_;
  ros::Subscriber sub_;
  ros::Publisher pub_;

  VelodyneLaserScanConfig cfg_;
  dynamic_reconfigure::Server<VelodyneLaserScanConfig> srv_;
  void reconfig(VelodyneLaserScanConfig& config, uint32_t level);

  unsigned int ring_count_;
};

}

#endif // VELODYNELASERSCAN_H
