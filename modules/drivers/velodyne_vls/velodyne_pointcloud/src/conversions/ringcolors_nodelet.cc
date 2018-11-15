/*
 *  Copyright (C) 2012 Austin Robot Technology, Jack O'Quin
 *  License: Modified BSD Software License Agreement
 *
 *  $Id$
 */

/** @file

    This ROS nodelet converts a Velodyne 3D LIDAR PointXYZIR cloud to
    PointXYZRGB, assigning colors for visualization of the laser
    rings.

*/

#include <ros/ros.h>
#include <pluginlib/class_list_macros.h>
#include <nodelet/nodelet.h>

#include "colors.h"

namespace velodyne_pointcloud
{
  class RingColorsNodelet: public nodelet::Nodelet
  {
  public:

    RingColorsNodelet() {}
    ~RingColorsNodelet() {}

  private:

    virtual void onInit();
    boost::shared_ptr<RingColors> colors_;
  };

  /** @brief Nodelet initialization. */
  void RingColorsNodelet::onInit()
  {
    colors_.reset(new RingColors(getNodeHandle(), getPrivateNodeHandle()));
  }

} // namespace velodyne_pointcloud


// Register this plugin with pluginlib.  Names must match nodelets.xml.
//
// parameters: class type, base class type
PLUGINLIB_EXPORT_CLASS(velodyne_pointcloud::RingColorsNodelet, nodelet::Nodelet)
