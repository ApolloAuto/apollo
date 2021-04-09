/*
 *  Copyright (C) 2012 Austin Robot Technology, Jack O'Quin
 * 
 *  License: Modified BSD Software License Agreement
 *
 *  $Id$
 */

/** \file
 *
 *  ROS driver node for the Velodyne 3D LIDARs.
 */

#include <ros/ros.h>
#include "driver.h"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "velodyne_node");
  ros::NodeHandle node;
  ros::NodeHandle private_nh("~");

  // start the driver
  velodyne_driver::VelodyneDriver dvr(node, private_nh);

  // loop until shut down or end of file
  while(ros::ok() && dvr.poll())
    {
      ros::spinOnce();
    }

  return 0;
}
