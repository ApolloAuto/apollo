/*
 *  Copyright (C) 2009, 2010 Austin Robot Technology, Jack O'Quin
 *  Copyright (C) 2011 Jesse Vera
 *  Copyright (C) 2012 Austin Robot Technology, Jack O'Quin
 *  License: Modified BSD Software License Agreement
 *
 *  $Id$
 */

/** @file

    This class transforms raw Velodyne 3D LIDAR packets to PointCloud2
    in the /odom frame of reference.

    @author Jack O'Quin
    @author Jesse Vera

*/

#include "transform.h"

#include <pcl_conversions/pcl_conversions.h>

namespace velodyne_pointcloud
{
  /** @brief Constructor. */
  Transform::Transform(ros::NodeHandle node, ros::NodeHandle private_nh):
    tf_prefix_(tf::getPrefixParam(private_nh)),
    data_(new velodyne_rawdata::RawData())
  {
    // Read calibration.
    data_->setup(private_nh);

    // advertise output point cloud (before subscribing to input data)
    output_ =
      node.advertise<sensor_msgs::PointCloud2>("velodyne_points", 10);

    srv_ = boost::make_shared <dynamic_reconfigure::Server<velodyne_pointcloud::
      TransformNodeConfig> > (private_nh);
    dynamic_reconfigure::Server<velodyne_pointcloud::TransformNodeConfig>::
      CallbackType f;
    f = boost::bind (&Transform::reconfigure_callback, this, _1, _2);
    srv_->setCallback (f);
    
    // subscribe to VelodyneScan packets using transform filter
    velodyne_scan_.subscribe(node, "velodyne_packets", 10);
    tf_filter_ =
      new tf::MessageFilter<velodyne_msgs::VelodyneScan>(velodyne_scan_,
                                                         listener_,
                                                         config_.frame_id, 10);
    tf_filter_->registerCallback(boost::bind(&Transform::processScan, this, _1));
  }
  
  void Transform::reconfigure_callback(
      velodyne_pointcloud::TransformNodeConfig &config, uint32_t level)
  {
    ROS_INFO_STREAM("Reconfigure request.");
    data_->setParameters(config.min_range, config.max_range, 
                         config.view_direction, config.view_width);
    config_.frame_id = tf::resolve(tf_prefix_, config.frame_id);
    ROS_INFO_STREAM("Target frame ID: " << config_.frame_id);
  }

  /** @brief Callback for raw scan messages.
   *
   *  @pre TF message filter has already waited until the transform to
   *       the configured @c frame_id can succeed.
   */
  void
    Transform::processScan(const velodyne_msgs::VelodyneScan::ConstPtr &scanMsg)
  {
    if (output_.getNumSubscribers() == 0)         // no one listening?
      return;                                     // avoid much work

    // allocate an output point cloud with same time as raw data
    VPointCloud::Ptr outMsg(new VPointCloud());
    outMsg->header.stamp = pcl_conversions::toPCL(scanMsg->header).stamp;
    outMsg->header.frame_id = config_.frame_id;
    outMsg->height = 1;

    // process each packet provided by the driver
    for (size_t next = 0; next < scanMsg->packets.size(); ++next)
      {
        // clear input point cloud to handle this packet
        inPc_.points.clear();
        inPc_.width = 0;
        inPc_.height = 1;
        std_msgs::Header header;
        header.stamp = scanMsg->packets[next].stamp;
        header.frame_id = scanMsg->header.frame_id;
        pcl_conversions::toPCL(header, inPc_.header);

        // unpack the raw data
        data_->unpack(scanMsg->packets[next], inPc_);

        // clear transform point cloud for this packet
        tfPc_.points.clear();           // is this needed?
        tfPc_.width = 0;
        tfPc_.height = 1;
        header.stamp = scanMsg->packets[next].stamp;
        pcl_conversions::toPCL(header, tfPc_.header);
        tfPc_.header.frame_id = config_.frame_id;

        // transform the packet point cloud into the target frame
        try
          {
            ROS_DEBUG_STREAM("transforming from " << inPc_.header.frame_id
                             << " to " << config_.frame_id);
	    VPointCloud inPc_V;
	    pcl::copyPointCloud(inPc_, inPc_V);
            //pcl_ros::transformPointCloud(config_.frame_id, inPc_, tfPc_,
            pcl_ros::transformPointCloud(config_.frame_id, inPc_V, tfPc_,
                                         listener_);
#if 0       // use the latest transform available, should usually work fine
            pcl_ros::transformPointCloud(inPc_.header.frame_id,
                                         ros::Time(0), inPc_,
                                         config_.frame_id,
                                         tfPc_, listener_);
#endif
          }
        catch (tf::TransformException &ex)
          {
            // only log tf error once every 100 times
            ROS_WARN_THROTTLE(100, "%s", ex.what());
            continue;                   // skip this packet
          }

        // append transformed packet data to end of output message
        outMsg->points.insert(outMsg->points.end(),
                             tfPc_.points.begin(),
                             tfPc_.points.end());
        outMsg->width += tfPc_.points.size();
      }

    // publish the accumulated cloud message
    ROS_DEBUG_STREAM("Publishing " << outMsg->height * outMsg->width
                     << " Velodyne points, time: " << outMsg->header.stamp);
    output_.publish(outMsg);
  }

} // namespace velodyne_pointcloud
