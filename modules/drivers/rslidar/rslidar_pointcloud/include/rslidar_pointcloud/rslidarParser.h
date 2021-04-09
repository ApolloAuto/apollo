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

/* -*- mode: C++ -*-
 *
 *  Copyright (C) 2007 Austin Robot Technology, Yaxin Liu, Patrick Beeson
 *  Copyright (C) 2009 Austin Robot Technology, Jack O'Quin
 *
 *  License: Modified BSD Software License Agreement
 *
 *  $Id$
 */

/** \file
 *
 *  Robosense 3D LIDAR data accessors
 *
 *  \ingroup rslidar
 *
 *  These classes unpack raw Robosense LIDAR packets into several
 *  useful formats.
 *
 *     rslidar::Data -- virtual base class for unpacking data into
 *                      various formats
 *
 *     rslidar::DataScans -- derived class, unpacks into vector of
 *                      individual laser scans
 *
 *     rslidar::DataXYZ -- derived class, unpacks into XYZ format
 *
 *  \todo make a separate header for each class?
 *
 *  \author Yaxin Liu
 *  \author Patrick Beeson
 *  \author Jack O'Quin
 */

#ifndef MODULES_DRIVERS_ROBOSENSE_RSLIDAR_POINTCLOUD_RSLIDAR_PARSER_H_
#define MODULES_DRIVERS_ROBOSENSE_RSLIDAR_POINTCLOUD_RSLIDAR_PARSER_H_

#include <errno.h>
#include <math.h>
#include <stdint.h>
#include <boost/format.hpp>
#include <string>

#include <nav_msgs/Odometry.h>
#include <pcl_ros/point_cloud.h>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/Time.h>
#include "rslidar_pointcloud/point_types.h"
#include "rslidar_msgs/rslidarScan.h"
#include "rslidar_msgs/rslidarPic.h"


namespace apollo {
namespace drivers {
namespace rslidar {

	typedef PointXYZIT VPoint;
	typedef pcl::PointCloud<VPoint> VPointCloud;
	class calibration_parse;
	struct Config_p {
  		double max_range;  ///< maximum range to publish
  		double min_range;  ///< minimum range to publish
  		double max_angle;
  		double min_angle;
  		double view_direction;
  		double view_width;
  		bool calibration_online;
  		std::string calibration_file;
  		std::string model;  // RS16,RS32
  		bool organized;     // is point cloud order
	};
    
	static const int SIZE_BLOCK = 100;
	static const int RAW_SCAN_SIZE = 3;
	static const int SCANS_PER_BLOCK = 32;
	static const int BLOCK_DATA_SIZE = (SCANS_PER_BLOCK * RAW_SCAN_SIZE); //96
	
	static const float ROTATION_RESOLUTION = 0.01f; /**< degrees 旋转角分辨率*/
	static const uint16_t ROTATION_MAX_UNITS = 36000; /**< hundredths of degrees */
	
	static const float DISTANCE_MAX = 200.0f;		 /**< meters */
	static const float DISTANCE_MIN = 0.2f; 	   /**< meters */
	static const float DISTANCE_RESOLUTION = 0.01f; /**< meters */
	static const float DISTANCE_MAX_UNITS = (DISTANCE_MAX
												 / DISTANCE_RESOLUTION + 1.0f);
	/** @todo make this work for both big and little-endian machines */
	static const uint16_t UPPER_BANK = 0xeeff; //
	static const uint16_t LOWER_BANK = 0xddff;
	
	/** Special Defines for RS16 support **/
	static const int RS16_FIRINGS_PER_BLOCK = 2;
	static const int RS16_SCANS_PER_FIRING = 16;
	static const float RS16_BLOCK_TDURATION = 100.0f;	// [µs]
	static const float RS16_DSR_TOFFSET = 3.0f;   // [µs]
	static const float RS16_FIRING_TOFFSET = 50.0f;   // [µs]
	static const int RS16_DATA_NUMBER_PER_SCAN = 40000; //Set 40000 to be large enough
	
	/** Special Defines for RS32 support **/
	static const int RS32_FIRINGS_PER_BLOCK = 1;
	static const int RS32_SCANS_PER_FIRING = 32;
	static const float RS32_BLOCK_TDURATION = 50.0f;   // [µs]
	static const float RS32_DSR_TOFFSET = 3.0f;   // [µs]
	static const float RL32_FIRING_TOFFSET = 50.0f;   // [µs]
	static const int RS32_DATA_NUMBER_PER_SCAN = 70000; //Set 70000 to be large enough
	
	static const int TEMPERATURE_MIN = 31;
	
	
	static calibration_parse* calibration_;
	typedef struct raw_block {
		uint16_t header;		///< UPPER_BANK or LOWER_BANK
		uint8_t rotation_1;
		uint8_t rotation_2; 	///combine rotation1 and rotation2 together to get 0-35999, divide by 100 to get degrees
		uint8_t data[BLOCK_DATA_SIZE]; //96
	} raw_block_t;
	
	
	union two_bytes {
		uint16_t uint;
		uint8_t bytes[2];
	};
	
	static const int PACKET_SIZE = 1248;
	static const int BLOCKS_PER_PACKET = 12;
	static const int PACKET_STATUS_SIZE = 4;
	static const int SCANS_PER_PACKET = (SCANS_PER_BLOCK * BLOCKS_PER_PACKET);
	
	typedef struct raw_packet {
		raw_block_t blocks[BLOCKS_PER_PACKET];
		uint16_t revolution;
		uint8_t status[PACKET_STATUS_SIZE];
	} raw_packet_t;
	
		/** \brief RSLIDAR data conversion class */
class rslidarParser {
	public:
		rslidarParser() {}
	
		virtual ~rslidarParser() {}

		/*init the size of the scan point size */
		virtual void init_setup() = 0;
	
		/*load the cablibrated files: angle, distance, intensity*/
		virtual void loadConfigFile(ros::NodeHandle private_nh) = 0;

		/*unpack the RS16 UDP packet and opuput PCL PointXYZI type*/
		virtual void unpack(const rslidar_msgs::rslidarPacket &pkt, pcl::PointCloud<pcl::PointXYZI>::Ptr pointcloud,
						bool finish_packets_parse) = 0;

	
	
		/*calibrated the azimuth*/
		virtual int correctAzimuth(float azimuth_f, int passageway)= 0;

		void sed_out_passage();
		
	};
	
	static float VERT_ANGLE[32];
    static float HORI_ANGLE[32];
    static float aIntensityCal[7][32];
    static int g_ChannelNum[32][51];
    static float CurvesRate[32];

    static float temper = 31.0;
    static int tempPacketNum = 0;
    static int numOfLasers = 16;
    static int TEMPERATURE_RANGE = 40;

    static rslidar_msgs::rslidarPic pic;
	

class rslidar16Parser : public rslidarParser {
	public:
	
	  	rslidar16Parser();
	  	~rslidar16Parser() {}
	
	 	void init_setup();
	
		/*load the cablibrated files: angle, distance, intensity*/
		void loadConfigFile(ros::NodeHandle private_nh);
	
		/*unpack the RS16 UDP packet and opuput PCL PointXYZI type*/
		void unpack(const rslidar_msgs::rslidarPacket &pkt, pcl::PointCloud<pcl::PointXYZI>::Ptr pointcloud,
						bool finish_packets_parse);

	
	
		/*calibrated the azimuth*/
		int correctAzimuth(float azimuth_f, int passageway);
	

};

class rslidar32Parser : public rslidarParser {
	public:
	
	  	rslidar32Parser(){}
	  	~rslidar32Parser() {}
	 
	 	void init_setup();
	
		/*load the cablibrated files: angle, distance, intensity*/
		void loadConfigFile(ros::NodeHandle private_nh);
	
		/*unpack the RS16 UDP packet and opuput PCL PointXYZI type*/
		void unpack(const rslidar_msgs::rslidarPacket &pkt, pcl::PointCloud<pcl::PointXYZI>::Ptr pointcloud,
						bool finish_packets_parse);
	
	
		/*calibrated the azimuth*/
		int correctAzimuth(float azimuth_f, int passageway);
	
		
};


class RslidarParserFactory {
 public:
   static rslidarParser *create_parser(Config_p config);
};

class calibration_parse
{
	public:
 	  calibration_parse() {}
	  ~calibration_parse() {}
	  float calibrateIntensity(float intensity, int calIdx, int distance);
	  float pixelToDistance(int pixelValue, int passageway);
	  int isABPacket(int distance);
	  float computeTemperature(unsigned char bit1, unsigned char bit2);
	  int estimateTemperature(float Temper);
};

}  // namespacejiexi rslidar//our suanfa
}  // namespace drivers
}  // namespace apollo

#endif  
