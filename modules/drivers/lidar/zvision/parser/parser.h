
/******************************************************************************
 * Copyright 2017 The Apollo Authors. All Rights Reserved.
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
 *  Zvision 3D LIDAR data accessors
 *
 *  \ingroup Zvision
 *
 *  These classes Unpack raw Zvision LIDAR packets into several
 *  useful formats.
 *
 *     Zvision::Data -- virtual base class for unpacking data into
 *                      various formats
 *
 *     Zvision::DataScans -- derived class, unpacks into vector of
 *                      individual laser scans
 *
 *     Zvision::DataXYZ -- derived class, unpacks into XYZ format
 *
 *  \todo make a separate header for each class?
 *
 *  \author Yaxin Liu
 *  \author Patrick Beeson
 *  \author Jack O'Quin
 */

#pragma once

#include <cerrno>
#include <cmath>
#include <cstdint>
#include <limits>
#include <memory>
#include <string>
#include <vector>

#include <boost/format.hpp>
#include "Eigen/Eigen"
// Eigen 3.3.7: #define ALIVE (0)
// fastrtps: enum ChangeKind_t { ALIVE, ... };
#if defined(ALIVE)
#undef ALIVE
#endif
// #include "modules/drivers/proto/pointcloud.pb.h"
#include "modules/common_msgs/sensor_msgs/pointcloud.pb.h"
#include "modules/drivers/lidar/zvision/tools/tools.h"
#include "modules/drivers/lidar/proto/zvision_config.pb.h"
#include "modules/drivers/lidar/proto/zvision.pb.h"

namespace apollo {
namespace drivers {
namespace zvision {

using apollo::drivers::PointCloud;
using apollo::drivers::PointXYZIT;

static const int PACKET_SIZE = 1304;
static const float nan = std::numeric_limits<float>::signaling_NaN();

/** \brief Zvision data conversion class */
class ZvisionParser {
 public:
  ZvisionParser() {}
  explicit ZvisionParser(const Config& config);
  virtual ~ZvisionParser();

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
  virtual void GeneratePointcloud(const std::shared_ptr<ZvisionScan>& scan_msg,
                                  std::shared_ptr<PointCloud> out_msg) = 0;
  virtual void setup();

  virtual bool CalibrationInitOk();

 protected:

  int LoadOfflineCalibration(std::string filename);

  int LoadOnlineCalibration(std::string ip);

  virtual void PollCalibrationData();

  bool need_transform_ = false;
  Eigen::Affine3d transform_;
  std::thread online_calibration_thread_;
  PointCalibrationTable cal_lut_;
  Config config_;
  volatile bool is_calibration_ = false;

  std::atomic<bool> running_ = {true};

  PointXYZIT get_nan_point(uint64_t timestamp);
  /**
   * \brief Compute coords with the data in block
   *
   * @param tmp A two bytes union store the value of laser distance information
   * @param index The index of block
   */
  void ComputeCoords(PointXYZIT* point);

  /**
   * \brief Unpack Zvision packet
   *
   */
  virtual void Unpack(const ZvisionPacket& pkt,
                      std::shared_ptr<PointCloud> pc) = 0;

  virtual uint64_t GetUdpSensorTimestamp(const unsigned char * data);
  virtual double GetExcitonTimestampNSSampleA(const unsigned char * data);
  virtual double GetExcitonTimestampNSSampleB(const unsigned char * data);
};  // class ZvisionParser

class Ml30Parser : public ZvisionParser {
 public:
  explicit Ml30Parser(const Config& config) : ZvisionParser(config) {}
  ~Ml30Parser() {}

  void GeneratePointcloud(const std::shared_ptr<ZvisionScan>& scan_msg,
                          std::shared_ptr<PointCloud> out_msg);
  //void setup() override;

 private:

  void Unpack(const ZvisionPacket& pkt, std::shared_ptr<PointCloud> pc);

};  // class Ml30Parser

class Ml30sa1Parser : public ZvisionParser {
 public:
  explicit Ml30sa1Parser(const Config& config) : ZvisionParser(config) {}
  ~Ml30sa1Parser() {}

  void GeneratePointcloud(const std::shared_ptr<ZvisionScan>& scan_msg,
                          std::shared_ptr<PointCloud> out_msg);
  //void setup() override;

 private:

  void Unpack(const ZvisionPacket& pkt, std::shared_ptr<PointCloud> pc);

};  // class Ml30sa1Parser

class MlxParser : public ZvisionParser {
 public:
  explicit MlxParser(const Config& config) : ZvisionParser(config) {}
  ~MlxParser() {}

  void GeneratePointcloud(const std::shared_ptr<ZvisionScan>& scan_msg,
                          std::shared_ptr<PointCloud> out_msg);
  //void setup() override;

 private:

  void Unpack(const ZvisionPacket& pkt, std::shared_ptr<PointCloud> pc);

};  // class MlxParser

class EZ6Parser : public ZvisionParser {
 public:
  explicit EZ6Parser(const Config& config) : ZvisionParser(config), 
        azi_comp_96_(azi_comp_96_init),
        ele_comp_96_(ele_comp_96_init),
        azi_comp_192_(azi_comp_192_init),
        ele_comp_192_(ele_comp_192_init) {}
  ~EZ6Parser() {}



  void GeneratePointcloud(const std::shared_ptr<ZvisionScan>& scan_msg,
                          std::shared_ptr<PointCloud> out_msg);
  //void setup() override;

 private:
  static const std::vector<float> azi_comp_96_init;
  static const std::vector<float> ele_comp_96_init;
  static const std::vector<float> azi_comp_192_init;
  static const std::vector<float> ele_comp_192_init;

  std::vector<float> azi_comp_96_;
  std::vector<float> ele_comp_96_;
  std::vector<float> azi_comp_192_;
  std::vector<float> ele_comp_192_;

  void Unpack(const ZvisionPacket& pkt, std::shared_ptr<PointCloud> pc);
};  // class EZ6Parser

class ZvisionParserFactory {
 public:
  static ZvisionParser* CreateParser(Config config);
};

}  // namespace Zvision
}  // namespace drivers
}  // namespace apollo
