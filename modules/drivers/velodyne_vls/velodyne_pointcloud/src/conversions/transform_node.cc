/*
 *  Copyright (C) 2012 Austin Robot Technology, Jack O'Quin
 *  License: Modified BSD Software License Agreement
 *
 *  $Id$
 */

/** \file

    This ROS node transforms raw Velodyne LIDAR packets to PointCloud2
    in the /odom frame of reference.

*/

#include <ros/ros.h>
#include "transform.h"

/** Main node entry point. */
int main(int argc, char **argv)
{
  ros::init(argc, argv, "transform_node");

  // create conversion class, which subscribes to raw data
  velodyne_pointcloud::Transform transform(ros::NodeHandle(),
                                           ros::NodeHandle("~"));

  // handle callbacks until shut down
  ros::spin();

  return 0;
}
