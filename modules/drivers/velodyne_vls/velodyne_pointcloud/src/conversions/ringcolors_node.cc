/*
 *  Copyright (C) 2012 Austin Robot Technology, Jack O'Quin
 *  License: Modified BSD Software License Agreement
 *
 *  $Id$
 */

/** @file

    This ROS node converts a Velodyne 3D LIDAR PointXYZIR cloud to
    PointXYZRGB, assigning colors for visualization of the laser
    rings.

*/

#include <ros/ros.h>
#include "colors.h"

/** Main node entry point. */
int main(int argc, char **argv)
{
  ros::init(argc, argv, "colors_node");
  ros::NodeHandle node;
  ros::NodeHandle priv_nh("~");

  // create conversion class, which subscribes to input messages
  velodyne_pointcloud::RingColors colors(node, priv_nh);

  // handle callbacks until shut down
  ros::spin();

  return 0;
}
