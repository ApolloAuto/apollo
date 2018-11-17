#include <ros/ros.h>
#include <pluginlib/class_list_macros.h>
#include <nodelet/nodelet.h>
#include "VelodyneLaserScan.h"

namespace velodyne_laserscan
{

class LaserScanNodelet: public nodelet::Nodelet
{
public:
  LaserScanNodelet() {}
  ~LaserScanNodelet() {}

private:
  virtual void onInit() {
    node_.reset(new VelodyneLaserScan(getNodeHandle(), getPrivateNodeHandle()));
  }
  boost::shared_ptr<VelodyneLaserScan> node_;
};

}

PLUGINLIB_DECLARE_CLASS(velodyne_laserscan, LaserScanNodelet, velodyne_laserscan::LaserScanNodelet, nodelet::Nodelet);
