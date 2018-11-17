/******************************************************************************
 * Copyright 2018 The Apollo Authors. All Rights Reserved.
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

#include "rslidar_pointcloud/convert.h"

#include <pcl/common/time.h>
#include <pcl_conversions/pcl_conversions.h>
#include <ros/advertise_options.h>

namespace apollo {
namespace drivers {
namespace rslidar {
	

 void Convert::init(ros::NodeHandle& node, ros::NodeHandle& private_nh) {
  	Config_p config_switch_;
	private_nh.param("max_range", config_switch_.max_range, 150.0);
	private_nh.param("min_range", config_switch_.min_range, 0.2);
	private_nh.param("queue_size", queue_size_, 10);
  	private_nh.param("model", config_switch_.model, std::string("RS16"));
	private_nh.param("topic_packets", topic_packets_, std::string("rslidar_packets"));
	private_nh.param("topic_pointcloud", topic_pointcloud_, std::string("rslidar_points"));

   	data_ = RslidarParserFactory::create_parser(config_switch_);

	calibration_ = new calibration_parse();
	if (config_switch_.model  == "RS16") {
		TEMPERATURE_RANGE = 40;
	} else if (config_switch_.model  == "RS32") {
		TEMPERATURE_RANGE = 50;
	}
	
  	data_->loadConfigFile(private_nh);            //load lidar parameters
  	data_->init_setup();
  
  	pointcloud_pub_ =
      node.advertise<sensor_msgs::PointCloud2>(topic_pointcloud_, queue_size_);

  	// subscribe to rslidarScan packets
  	rslidar_scan_ = node.subscribe(
		topic_packets_, queue_size_, &Convert::convert_packets_to_pointcloud,
      (Convert*)this, ros::TransportHints().tcpNoDelay(true));
 
}


Convert::~Convert() {
  	if (data_ != nullptr) {
    	delete data_;
  	}
}

/** @brief Callback for raw scan messages. */
void Convert::convert_packets_to_pointcloud(
    const rslidar_msgs::rslidarScan::ConstPtr& scan_msg) {
  ROS_INFO_ONCE("********************************************************");
  ROS_INFO_ONCE("Start convert rslidar packets to pointcloud");
  ROS_INFO_ONCE("********************************************************");
  ROS_DEBUG_STREAM(scan_msg->header.seq);

  
  pcl::PointCloud<pcl::PointXYZI>::Ptr pointcloud(new pcl::PointCloud<pcl::PointXYZI>);
  pointcloud->header.frame_id = scan_msg->header.frame_id;
  pointcloud->header.stamp = pcl_conversions::toPCL(scan_msg->header).stamp;
  
  //sensor_msgs::PointCloud2 outMsg;
  //use rslidar method
  bool finish_packets_parse = false;
   for (size_t i = 0; i < scan_msg->packets.size(); ++i) {
     if (i == (scan_msg->packets.size() - 1)) {
         // ROS_INFO_STREAM("Packets per scan: "<< scanMsg->packets.size());
         finish_packets_parse = true;
     	}
     	data_->unpack(scan_msg->packets[i], pointcloud, finish_packets_parse);  //wait
   }
  
  if (pointcloud->empty()) {
	ROS_INFO_ONCE("pointcloud->empty()");
    return;
  }

  // publish the accumulated cloud message
  pointcloud_pub_.publish(pointcloud);
}

}  // namespace rslidar
}  // namespace drivers
}  // namespace apollo
