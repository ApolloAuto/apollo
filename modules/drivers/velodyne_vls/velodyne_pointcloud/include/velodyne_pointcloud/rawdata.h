/* -*- mode: C++ -*-
 *
 *  Copyright (C) 2007 Austin Robot Technology, Yaxin Liu, Patrick Beeson
 *  Copyright (C) 2009, 2010, 2012 Austin Robot Technology, Jack O'Quin
 *  Copyright (C) 2017, Velodyne LiDAR INC., Algorithms and Signal Processing Group
 *
 *  License: Modified BSD Software License Agreement
 *
 *  $Id$
 */

/** @file
 *
 *  @brief Interfaces for interpreting raw packets from the Velodyne 3D LIDAR.
 *
 *  @author Yaxin Liu
 *  @author Patrick Beeson
 *  @author Jack O'Quin
 *  @author Velodyne LiDAR, Algorithms and Signal Processing Group
 */

#ifndef __VELODYNE_RAWDATA_H
#define __VELODYNE_RAWDATA_H

#include <errno.h>
#include <stdint.h>
#include <string>
#include <boost/format.hpp>
#include <math.h>

#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <velodyne_msgs/VelodyneScan.h>
#include <velodyne_pointcloud/point_types.h>
#include <velodyne_pointcloud/calibration.h>

namespace velodyne_rawdata
{
  // Shorthand typedefs for point cloud representations
  struct PointXYZIRB
   {
       PCL_ADD_POINT4D;                    // quad-word XYZ
       float    intensity;                 ///< laser intensity reading
       uint16_t ring;                      ///< laser ring number
       uint16_t block;                     ///< packet datablock number
       EIGEN_MAKE_ALIGNED_OPERATOR_NEW     // ensure proper alignment
   } EIGEN_ALIGN16;

  typedef velodyne_pointcloud::PointXYZIR VPoint;
  typedef pcl::PointCloud<VPoint> VPointCloud;
  // typedef pcl::PointCloud<PointXYZIRB> XYZIRBPointCloud;

  /**
   * Raw Velodyne packet constants and structures.
   */
  static const int BLOCK_SIZE = 100; // [bytes]
  static const int CHANNEL_SIZE = 3; // [bytes]
  static const int NUM_CHANS_PER_BLOCK = 32;
  static const int BLOCK_DATA_SIZE = (NUM_CHANS_PER_BLOCK * CHANNEL_SIZE);
  static const float  CHANNEL_TDURATION  =   2.304f;   // [µs] Channels corresponds to one laser firing
  static const float  SEQ_TDURATION      =  55.296f;   // [µs] Sequence is a set of laser firings including recharging

  static const float    ROTATION_RESOLUTION   =     0.01f;  // [deg]
  static const uint16_t ROTATION_MAX_UNITS    =    36000u;  // [deg/100]
  static const float    DISTANCE_RESOLUTION   =    0.002f;  // [m]
  static const float    VLP32_DISTANCE_RESOLUTION   =    0.004f;  // [m]

  /** @todo make this work for both big and little-endian machines */
  static const uint16_t UPPER_BANK = 0xeeff;
  static const uint16_t LOWER_BANK = 0xddff;
  
  /** Special Definitions for VLP16 support **/
  static const int    VLP16_NUM_SEQS_PER_BLOCK  = 2;
  static const int    VLP16_NUM_CHANS_PER_SEQ   = 16;
  static const float  VLP16_BLOCK_TDURATION     = (VLP16_NUM_SEQS_PER_BLOCK * SEQ_TDURATION);
 
  /** Special Definitions for HDL32 support **/
  static const float  HDL32_CHANNEL_TDURATION  =   1.152f;   // [µs] From Application Note: HDL-32E Packet Structure and Timing Defition
  static const float  HDL32_SEQ_TDURATION      =  46.080f;   // [µs] Sequence is a set of laser firings including recharging
  
  /** Special Definition for VLS128 support **/
  static const int VLS128_NUM_CHANS_PER_BLOCK = 128;
  static const int VLS128_BLOCK_DATA_SIZE = VLS128_NUM_CHANS_PER_BLOCK * CHANNEL_SIZE;
  
  /** \brief Raw Velodyne data block.
   *
   *  Each block contains data from either the upper or lower laser
   *  bank.  The device returns three times as many upper bank blocks.
   *
   *  use stdint.h types, so things work with both 64 and 32-bit machines
   */
  typedef struct raw_block
  {
    uint16_t header;        ///< UPPER_BANK or LOWER_BANK
    uint16_t rotation;      ///< 0-35999, divide by 100 to get degrees
    uint8_t  data[BLOCK_DATA_SIZE];
  } raw_block_t;

  /** used for unpacking the first two data bytes in a block
   *
   *  They are packed into the actual data stream misaligned.  I doubt
   *  this works on big endian machines.
   */
  union two_bytes
  {
    uint16_t uint;
    uint8_t  bytes[2];
  };

  static const int PACKET_SIZE = 1206; // [bytes]
  static const int NUM_BLOCKS_PER_PACKET = 12;
  static const int PACKET_STATUS_SIZE = 4;
  static const int NUM_CHANS_PER_PACKET = (NUM_CHANS_PER_BLOCK * NUM_BLOCKS_PER_PACKET);

  /** \brief Raw Velodyne packet.
   *
   *  revolution is described in the device manual as incrementing
   *    (mod 65536) for each physical turn of the device.  Our device
   *    seems to alternate between two different values every third
   *    packet.  One value increases, the other decreases.
   *
   *  \todo figure out if revolution is only present for one of the
   *  two types of status fields
   *
   *  status has either a temperature encoding or the microcode level
   */
  typedef struct raw_packet
  {
    raw_block_t blocks[NUM_BLOCKS_PER_PACKET];
    uint16_t revolution;
    uint8_t status[PACKET_STATUS_SIZE]; 
  } raw_packet_t;

  /** \brief Velodyne data conversion class */
  class RawData
  {
  public:

    RawData();
    ~RawData() {}

    /** \brief Set up for data processing.
     *
     *  Perform initializations needed before data processing can
     *  begin:
     *
     *    - read device-specific angles calibration
     *
     *  @param private_nh private node handle for ROS parameters
     *  @returns 0 if successful;
     *           errno value for failure
     */
    int setup(ros::NodeHandle private_nh);

    /** \brief Set up for data processing offline. 
      * Performs the same initialization as in setup, in the abscence of a ros::NodeHandle.
      * this method is useful if unpacking data directly from bag files, without passing 
      * through a communication overhead.
      * 
      * @param calibration_file path to the calibration file
      * @param max_range_ cutoff for maximum range
      * @param min_range_ cutoff for minimum range
      * @returns 0 if successful;
      *           errno value for failure
     */
    int setupOffline(std::string calibration_file, double max_range_, double min_range_);

    void unpack(const velodyne_msgs::VelodynePacket &pkt, VPointCloud &pc);
//    void unpack(const velodyne_msgs::VelodynePacket &pkt, XYZIRBPointCloud &pc);
    
    void setParameters(double min_range, double max_range, double view_direction,
                       double view_width);

  private:

    /** configuration parameters */
    typedef struct {
      std::string calibrationFile;     ///< calibration file name
      double max_range;                ///< maximum range to publish
      double min_range;                ///< minimum range to publish
      int min_angle;                   ///< minimum angle to publish
      int max_angle;                   ///< maximum angle to publish
      
      double tmp_min_angle;
      double tmp_max_angle;
    } Config;
    Config config_;
    float previous_block_corrected_azimuth;    // for support of different scan patterns
    float previous_block_corrected_elevation;  // record the scan profile endpoint in previous packet 

    /** 
     * Calibration file
     */
    velodyne_pointcloud::Calibration calibration_;
    float sin_rot_table_[ROTATION_MAX_UNITS];
    float cos_rot_table_[ROTATION_MAX_UNITS];
    /** add private function to handle each sensor **/ 
    void unpack_vlp16(const velodyne_msgs::VelodynePacket &pkt, VPointCloud &pc);
    void unpack_vlp32(const velodyne_msgs::VelodynePacket &pkt, VPointCloud &pc);
    void unpack_hdl32(const velodyne_msgs::VelodynePacket &pkt, VPointCloud &pc);
    void unpack_hdl64(const velodyne_msgs::VelodynePacket &pkt, VPointCloud &pc);
    void unpack_vls128(const velodyne_msgs::VelodynePacket &pkt, VPointCloud &pc);
    void compute_xyzi( const uint8_t chan_id
                     , const uint16_t azimuth_uint
                     , const float distance
                     , float &intensity
                     , float &x_coord
                     , float &y_coord
                     , float &z_coord
                     ); 

    /** in-line test whether a point is in range */
    bool pointInRange(float range)
    {
      return (range >= config_.min_range
              && range <= config_.max_range);
    }
  };

} // namespace velodyne_rawdata
  POINT_CLOUD_REGISTER_POINT_STRUCT(velodyne_rawdata::PointXYZIRB,
		(float, x, x)
                (float, y, y)
                (float, z, z)
                (float, intensity, intensity)
                (uint16_t, ring,  ring)
                (uint16_t, block, block))


#endif // __VELODYNE_RAWDATA_H
