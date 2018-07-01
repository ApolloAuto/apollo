/***************************************************************************
Copyright 2018 The Apollo Authors. All Rights Reserved                     /
                                                                            /
Licensed under the Apache License, Version 2.0 (the "License");             /
you may not use this file except in compliance with the License.            /
You may obtain a copy of the License at                                     /
                                                                            /
    http://www.apache.org/licenses/LICENSE-2.0                              /
                                                                            /
Unless required by applicable law or agreed to in writing, software         /
distributed under the License is distributed on an "AS IS" BASIS,           /
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.    /
See the License for the specific language governing permissions and         /
limitations under the License.                                              /
****************************************************************************/

#ifndef LSLIDAR_DECODER_H
#define LSLIDAR_DECODER_H

#define DEG_TO_RAD 0.017453292
#define RAD_TO_DEG 57.29577951

#include <cmath>
#include <vector>
#include <string>
#include <boost/shared_ptr.hpp>

#include <ros/ros.h>
#include <std_msgs/UInt64.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/LaserScan.h>
#include <std_msgs/Int8.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>

#include <lslidar_msgs/LslidarPacket.h>
#include <lslidar_msgs/LslidarPoint.h>
#include <lslidar_msgs/LslidarScan.h>
#include <lslidar_msgs/LslidarSweep.h>
#include <lslidar_msgs/LslidarLayer.h>

namespace apollo {
namespace drivers {
namespace lslidar_decoder {

// Raw lslidar packet constants and structures.
static const int SIZE_BLOCK      = 100;
static const int RAW_SCAN_SIZE   = 3;
static const int SCANS_PER_BLOCK = 32;
static const int BLOCK_DATA_SIZE =
        (SCANS_PER_BLOCK * RAW_SCAN_SIZE);

// According to Bruce Hall DISTANCE_MAX is 65.0, but we noticed
// valid packets with readings up to 130.0.
static const double DISTANCE_MAX        = 130.0;        /**< meters */
static const double DISTANCE_RESOLUTION = 0.01; /**< meters */
static const double DISTANCE_MAX_UNITS  =
        (DISTANCE_MAX / DISTANCE_RESOLUTION + 1.0);

/** @todo make this work for both big and little-endian machines */
static const uint16_t UPPER_BANK = 0xeeff;
static const uint16_t LOWER_BANK = 0xddff;

/** Special Defines for VLP16 support **/
static const int     FIRINGS_PER_BLOCK = 2;
static const int     SCANS_PER_FIRING  = 16;
static const double  BLOCK_TDURATION   = 110.592; // [µs]
static const double  DSR_TOFFSET       = 2.304;   // [µs]
static const double  FIRING_TOFFSET    = 55.296;  // [µs]

static const int PACKET_SIZE        = 1206;
static const int BLOCKS_PER_PACKET  = 12;
static const int PACKET_STATUS_SIZE = 4;
static const int SCANS_PER_PACKET =
        (SCANS_PER_BLOCK * BLOCKS_PER_PACKET);
static const int FIRINGS_PER_PACKET =
        FIRINGS_PER_BLOCK * BLOCKS_PER_PACKET;

// Pre-compute the sine and cosine for the altitude angles.
static const double SCAN_ALTITUDE[16] = {
    -0.2617993877991494,   0.017453292519943295,
    -0.22689280275926285,  0.05235987755982989,
    -0.19198621771937624,  0.08726646259971647,
    -0.15707963267948966,  0.12217304763960307,
    -0.12217304763960307,  0.15707963267948966,
    -0.08726646259971647,  0.19198621771937624,
    -0.05235987755982989,  0.22689280275926285,
    -0.017453292519943295, 0.2617993877991494
};

static const double COS_SCAN_ALTITUDE[16] = {
    std::cos(SCAN_ALTITUDE[ 0]), std::cos(SCAN_ALTITUDE[ 1]),
    std::cos(SCAN_ALTITUDE[ 2]), std::cos(SCAN_ALTITUDE[ 3]),
    std::cos(SCAN_ALTITUDE[ 4]), std::cos(SCAN_ALTITUDE[ 5]),
    std::cos(SCAN_ALTITUDE[ 6]), std::cos(SCAN_ALTITUDE[ 7]),
    std::cos(SCAN_ALTITUDE[ 8]), std::cos(SCAN_ALTITUDE[ 9]),
    std::cos(SCAN_ALTITUDE[10]), std::cos(SCAN_ALTITUDE[11]),
    std::cos(SCAN_ALTITUDE[12]), std::cos(SCAN_ALTITUDE[13]),
    std::cos(SCAN_ALTITUDE[14]), std::cos(SCAN_ALTITUDE[15]),
};

static const double SIN_SCAN_ALTITUDE[16] = {
    std::sin(SCAN_ALTITUDE[ 0]), std::sin(SCAN_ALTITUDE[ 1]),
    std::sin(SCAN_ALTITUDE[ 2]), std::sin(SCAN_ALTITUDE[ 3]),
    std::sin(SCAN_ALTITUDE[ 4]), std::sin(SCAN_ALTITUDE[ 5]),
    std::sin(SCAN_ALTITUDE[ 6]), std::sin(SCAN_ALTITUDE[ 7]),
    std::sin(SCAN_ALTITUDE[ 8]), std::sin(SCAN_ALTITUDE[ 9]),
    std::sin(SCAN_ALTITUDE[10]), std::sin(SCAN_ALTITUDE[11]),
    std::sin(SCAN_ALTITUDE[12]), std::sin(SCAN_ALTITUDE[13]),
    std::sin(SCAN_ALTITUDE[14]), std::sin(SCAN_ALTITUDE[15]),
};

struct point_struct{
    double distance;
    double intensity;
};

struct PointXYZIT {
  PCL_ADD_POINT4D
  uint8_t intensity;
  double timestamp;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW  // make sure our new allocators are aligned
} EIGEN_ALIGN16;
// enforce SSE padding for correct memory alignment

class LslidarDecoder {
public:

    LslidarDecoder(ros::NodeHandle& n, ros::NodeHandle& pn);
    LslidarDecoder(const LslidarDecoder&) = delete;
    LslidarDecoder operator=(const LslidarDecoder&) = delete;
    ~LslidarDecoder() {return;}

    bool initialize();

    typedef boost::shared_ptr<LslidarDecoder> LslidarDecoderPtr;
    typedef boost::shared_ptr<const LslidarDecoder> LslidarDecoderConstPtr;

private:

    union TwoBytes {
        uint16_t distance;
        uint8_t  bytes[2];
    };

    struct RawBlock {
        uint16_t header;        ///< UPPER_BANK or LOWER_BANK
        uint16_t rotation;      ///< 0-35999, divide by 100 to get degrees
        uint8_t  data[BLOCK_DATA_SIZE];
    };

    struct RawPacket {
        RawBlock blocks[BLOCKS_PER_PACKET];
        uint32_t time_stamp;
        uint8_t factory[2];
        //uint16_t revolution;
        //uint8_t status[PACKET_STATUS_SIZE];
    };

    struct Firing {
        // Azimuth associated with the first shot within this firing.
        double firing_azimuth;
        double azimuth[SCANS_PER_FIRING];
        double distance[SCANS_PER_FIRING];
        double intensity[SCANS_PER_FIRING];
    };

    // Intialization sequence
    bool loadParameters();
    bool createRosIO();


    // Callback function for a single lslidar packet.
    bool checkPacketValidity(const RawPacket* packet);
    void decodePacket(const RawPacket* packet);
    void layerCallback(const std_msgs::Int8Ptr& msg);
    void packetCallback(const lslidar_msgs::LslidarPacketConstPtr& msg);
    // Publish data
    void publishPointCloud();
    void publishChannelScan();
    // Publish scan Data
    void publishScan();

    // Check if a point is in the required range.
    bool isPointInRange(const double& distance) {
        return (distance >= min_range && distance <= max_range);
    }

    double rawAzimuthToDouble(const uint16_t& raw_azimuth) {
        // According to the user manual,
        // azimuth = raw_azimuth / 100.0;
        return static_cast<double>(raw_azimuth) / 100.0 * DEG_TO_RAD;
    }

    // calc the means_point
    point_struct getMeans(std::vector<point_struct> clusters);

    // configuration degree base
    int point_num;
    double angle_base;

    // Configuration parameters
    double min_range;
    double max_range;
    double angle_disable_min;
    double angle_disable_max;
    double frequency;
    bool publish_point_cloud;
    bool publish_channels;
    bool apollo_interface;
    double cos_azimuth_table[6300];
    double sin_azimuth_table[6300];

    bool is_first_sweep;
    double last_azimuth;
    double sweep_start_time;
    double packet_start_time;
    double point_time;
    int layer_num;
    Firing firings[FIRINGS_PER_PACKET];

    // ROS related parameters
    ros::NodeHandle nh;
    ros::NodeHandle pnh;

    std::string fixed_frame_id;
    std::string child_frame_id;

    lslidar_msgs::LslidarSweepPtr sweep_data;
    lslidar_msgs::LslidarLayerPtr multi_scan;
    sensor_msgs::PointCloud2 point_cloud_data;

    ros::Subscriber packet_sub;
    ros::Subscriber layer_sub;
    ros::Publisher sweep_pub;
    ros::Publisher point_cloud_pub;
    ros::Publisher scan_pub;
    ros::Publisher channel_scan_pub;

};

typedef LslidarDecoder::LslidarDecoderPtr LslidarDecoderPtr;
typedef LslidarDecoder::LslidarDecoderConstPtr LslidarDecoderConstPtr;
typedef PointXYZIT VPoint;
typedef pcl::PointCloud<VPoint> VPointCloud;

} // end namespace lslidar_decoder
}
}

POINT_CLOUD_REGISTER_POINT_STRUCT(apollo::drivers::lslidar_decoder::PointXYZIT,
                                  (float, x, x)(float, y, y)(float, z, z)(
                                      uint8_t, intensity,
                                      intensity)(double, timestamp, timestamp))
#endif
