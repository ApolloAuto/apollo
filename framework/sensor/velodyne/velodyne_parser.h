
#ifndef SENSOR_VELODYNE_VELODYNE_PARSER_H
#define SENSOR_VELODYNE_VELODYNE_PARSER_H

#include <errno.h>
#include <stdint.h>
#include <string>
#include <boost/format.hpp>

#include "cybertron/time/time.h"
#include "sensor/proto/sensor_velodyne.pb.h"
#include "sensor/proto/sensor_pointcloud.pb.h"
#include "sensor/proto/velodyne_config.pb.h"

#include "sensor/velodyne/calibration.h"
#include "sensor/velodyne/const_variables.h"
#include "sensor/velodyne/data_type.h"

namespace apollo {
namespace sensor {
namespace velodyne {

/** \brief Velodyne data conversion class */
class VelodyneParser {
 public:
  VelodyneParser() {}
  VelodyneParser(proto::VelodyneConfig& config);
  virtual ~VelodyneParser() {}

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
  virtual void generate_pointcloud(
      const std::shared_ptr<proto::VelodyneScan const>& scan_msg,
      std::shared_ptr<proto::PointCloud>& out_msg) = 0;
  virtual void setup();
  // order point cloud fod IDL by velodyne model
  virtual void order(std::shared_ptr<proto::PointCloud>& cloud) = 0;

  const Calibration& get_calibration() { return _calibration; }
  double get_last_timestamp() { return _last_time_stamp; }

 protected:
  const float (*_inner_time)[12][32];
  /**
   * \brief Calibration file
   */
  Calibration _calibration;
  float _sin_rot_table[ROTATION_MAX_UNITS];
  float _cos_rot_table[ROTATION_MAX_UNITS];
  proto::VelodyneConfig _config;
  // Last Velodyne packet time stamp. (Full time)
  double _last_time_stamp;
  bool _need_two_pt_correction;
  Mode _mode;

  proto::PointXYZIT get_nan_point(uint64_t timestamp);
  void init_angle_params(double view_direction, double view_width);
  /**
   * \brief Compute coords with the data in block
   *
   * @param tmp A two bytes union store the value of laser distance infomation
   * @param index The index of block
   */
  void compute_coords(const union RawDistance& raw_distance,
                      const LaserCorrection& corrections,
                      const uint16_t& rotation,
                      proto::PointXYZIT* point);

  bool is_scan_valid(int rotation, float distance);

  /**
   * \brief Unpack velodyne packet
   *
   */
  virtual void unpack(const proto::VelodynePacket& pkt,
                      std::shared_ptr<proto::PointCloud>& pc) = 0;

  uint64_t get_gps_stamp(double current_stamp, double& previous_stamp,
                         uint64_t& gps_base_usec);

  virtual uint64_t get_timestamp(double base_time, float time_offset,
                                 uint16_t laser_block_id) = 0;

  void timestamp_check(double timestamp);

  void init_sin_cos_rot_table(float* sin_rot_table, float* cos_rot_table,
                              uint16_t rotation, float rotation_resolution);

};  // class VelodyneParser

class Velodyne64Parser : public VelodyneParser {
 public:
  Velodyne64Parser(proto::VelodyneConfig& config);
  ~Velodyne64Parser() {}

  void generate_pointcloud(
      const std::shared_ptr<proto::VelodyneScan const>& scan_msg,
      std::shared_ptr<proto::PointCloud>& out_msg);
  void check_gps_status(const proto::VelodynePacket& pkt);
  void order(std::shared_ptr<proto::PointCloud>& cloud);
  void setup();

 private:
  void set_base_time_from_packets(const proto::VelodynePacket& pkt);
  uint64_t get_timestamp(double base_time, float time_offset,
                         uint16_t laser_block_id);
  void unpack(const proto::VelodynePacket& pkt,
              std::shared_ptr<proto::PointCloud>& pc);
  void init_offsets();
  int intensity_compensate(const LaserCorrection& corrections,
                           const uint16_t& raw_distance, int intensity);
  // Previous Velodyne packet time stamp. (offset to the top hour)
  double _previous_packet_stamp[4];
  uint64_t _gps_base_usec[4];  // full time
  bool _is_s2;
  int _offsets[64];

};  // class Velodyne64Parser

class Velodyne32Parser : public VelodyneParser {
 public:
  Velodyne32Parser(proto::VelodyneConfig& config);
  ~Velodyne32Parser() {}

  void generate_pointcloud(
      const std::shared_ptr<proto::VelodyneScan const>& scan_msg,
      std::shared_ptr<proto::PointCloud>& out_msg);
  void order(std::shared_ptr<proto::PointCloud>& cloud);

 private:
  uint64_t get_timestamp(double base_time, float time_offset,
                         uint16_t laser_block_id);
  void unpack(const proto::VelodynePacket& pkt,
              std::shared_ptr<proto::PointCloud>& pc);
  // Previous Velodyne packet time stamp. (offset to the top hour)
  double _previous_packet_stamp;
  uint64_t _gps_base_usec;  // full time

};  // class Velodyne32Parser

class Velodyne16Parser : public VelodyneParser {
 public:
  Velodyne16Parser(proto::VelodyneConfig& config);
  ~Velodyne16Parser() {}

  void generate_pointcloud(
      const std::shared_ptr<proto::VelodyneScan const>& scan_msg,
      std::shared_ptr<proto::PointCloud>& out_msg);
  void order(std::shared_ptr<proto::PointCloud>& cloud);

 private:
  uint64_t get_timestamp(double base_time, float time_offset,
                         uint16_t laser_block_id);
  void unpack(const proto::VelodynePacket& pkt,
              std::shared_ptr<proto::PointCloud>& pc);
  // Previous Velodyne packet time stamp. (offset to the top hour)
  double _previous_packet_stamp;
  uint64_t _gps_base_usec;  // full time

};  // class Velodyne32Parser

class VelodyneParserFactory {
 public:
  static VelodyneParser* create_parser(
      proto::VelodyneConfig& config);
};

}  // namespace velodyne
}
}

#endif  // SENSOR_VELODYNE_VELODYNE_PARSER_H
