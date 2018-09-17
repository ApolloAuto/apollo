
#ifndef SENSOR_VELODYNE_CALIBRATION_H
#define SENSOR_VELODYNE_CALIBRATION_H

#include <memory>
#include <map>
#include <string>

namespace apollo {
namespace sensor {
namespace velodyne {

struct LaserCorrection {

  /** parameters in db.xml */
  float rot_correction;
  float vert_correction;
  float dist_correction;
  float dist_correction_x;
  float dist_correction_y;
  float vert_offset_correction;
  float horiz_offset_correction;
  int max_intensity;
  int min_intensity;
  float focal_distance;
  float focal_slope;
  float focal_offset;

  float cos_rot_correction;   ///< cached cosine of rot_correction
  float sin_rot_correction;   ///< cached sine of rot_correction
  float cos_vert_correction;  ///< cached cosine of vert_correction
  float sin_vert_correction;  ///< cached sine of vert_correction

  int laser_ring;  ///< ring number for this laser
};

class Calibration {

 public:
  std::map<int, LaserCorrection> _laser_corrections;
  int _num_lasers;
  bool _initialized;

 public:
  Calibration() : _initialized(false) {}
  Calibration(const std::string& calibration_file) { read(calibration_file); }

  void read(const std::string& calibration_file);
  void write(const std::string& calibration_file);
};

} /* velodyne_pointcloud */
}
}

#endif  // SENSOR_VELODYNE_CALIBRATION_H
