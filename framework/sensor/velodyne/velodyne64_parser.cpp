
#include <fstream>

#include "cybertron/common/log.h"
#include "sensor/velodyne/velodyne_parser.h"

namespace apollo {
namespace sensor {
namespace velodyne {

Velodyne64Parser::Velodyne64Parser(proto::VelodyneConfig& config)
    : VelodyneParser(config) {

  for (int i = 0; i < 4; i++) {
    _gps_base_usec[i] = 0;
    _previous_packet_stamp[i] = 0;
  }
  _need_two_pt_correction = true;
  // init unpack function and order function by model.
  // default 64E_S3
  _inner_time = &velodyne::INNER_TIME_64E_S3;
  _is_s2 = false;
  if (_config.model() == proto::HDL64E_S2) {
    _inner_time = &velodyne::INNER_TIME_64;
    _is_s2 = true;
  }

  if (_config.mode() == proto::LAST) {
    _mode = LAST;
  } else if (_config.mode() == proto::DUAL) {
    _mode = DUAL;
  }

}

void Velodyne64Parser::setup() {
  VelodyneParser::setup();
  if (_config.organized()) {
    init_offsets();
  }
}

void Velodyne64Parser::set_base_time_from_packets(
    const proto::VelodynePacket& pkt) {
  const RawPacket* raw = (const RawPacket*)pkt.data().c_str();
  StatusType status_type = StatusType(raw->status_type);
  char status_value = raw->status_value;

  static int year = -1, month = -1, day = -1, hour = -1, minute = -1,
             second = -1;
  static int gps_status = 0;
  static tm time;
  // static int error_flag = 1;

  switch (status_type) {
    case YEAR:
      year = status_value + 2000;
      break;
    case MONTH:
      month = status_value;
      break;
    case DATE:
      day = status_value;
      break;
    case HOURS:
      hour = status_value;
      break;
    case MINUTES:
      minute = status_value;
      break;
    case SECONDS:
      second = status_value;
      break;
    case GPS_STATUS:
      gps_status = status_value;
      break;
    default:
      break;
  }

  AINFO << "Get base time from packets. Obtained "
            << ", year:" << year << ", month:" << month << ", day:" << day
            << ", hour:" << hour << ", minute:" << minute << ", second:" << second;

  if (status_type == GPS_STATUS && year > 0 && month > 0 && day > 0 &&
      hour >= 0 && minute >= 0 && second >= 0) {
    if (gps_status != 65) {
      AERROR << "Sync failed because Velodyne-GPS Sync is NOT good! "
                    "Status: " << (int)gps_status
                 << " (65 = both; 86 = gps only; 80 = PPS only; 0 = GPS not "
                    "connected)";
    }

    time.tm_year = year - 1900;
    time.tm_mon = month - 1;
    time.tm_mday = day;
    time.tm_hour = hour + _config.time_zone();
    time.tm_min = 0;
    time.tm_sec = 0;

    uint64_t unix_base = static_cast<uint64_t>(mktime(&time));
    for (int i = 0; i < 4; i++) {
      _gps_base_usec[i] =
          unix_base * 1e6;  // unix_to_gps_microseconds(unix_base * 1e6);
    }
  }
}

void Velodyne64Parser::check_gps_status(
    const proto::VelodynePacket& pkt) {
  const RawPacket* raw = (const RawPacket*)pkt.data().c_str();
  StatusType status_type = StatusType(raw->status_type);
  char status_value = raw->status_value;

  if (status_type == StatusType::GPS_STATUS) {
    if (status_value != 65) {
      AERROR << "Sync failed because Velodyne-GPS Sync is NOT good! "
                    "Status: " << (int)status_value
                 << " (65 = both; 86 = gps only; 80 = PPS only; 0 = GPS not "
                    "connected)";
    }
  }
}

void Velodyne64Parser::init_offsets() {
  int width = 64;
  // pre compute col offsets
  for (int i = 0; i < width; ++i) {
    int col = velodyne::ORDER_64[i];
    // compute offset, NOTICE: std::map doesn't have const [] since [] may
    // insert new values into map
    const LaserCorrection& corrections = _calibration._laser_corrections[col];
    int offset = int(corrections.rot_correction / ANGULAR_RESOLUTION + 0.5);
    _offsets[i] = offset;
  }
}

void Velodyne64Parser::generate_pointcloud(
    const std::shared_ptr<proto::VelodyneScan const>& scan_msg,
    std::shared_ptr<proto::PointCloud>& out_msg) {
  // ros::Time begin = ros::Time::Now();
  // allocate a point cloud with same time and frame ID as raw data
  out_msg->mutable_header()->set_frame_id(scan_msg->header().frame_id());
  out_msg->mutable_header()->set_stamp(apollo::cybertron::Time().Now().ToNanosecond());
  out_msg->mutable_header()->set_timestamp_sec(apollo::cybertron::Time().Now().ToSecond());
  out_msg->set_height(1);
  // out_msg->header.seq = scan_msg->header.seq;

  //out_msg->mutable_point()->Reserve(140000);

  bool skip = false;
  for (int i = 0; i < scan_msg->firing_pkts_size(); ++i) {
    if (!_gps_base_usec[0]) {
      // only set one time type when call this function, so cannot break
      set_base_time_from_packets(scan_msg->firing_pkts(i));
      // If base time not ready then set empty_unpack true
      skip = true;
    } else {
      check_gps_status(scan_msg->firing_pkts(i));
      unpack(scan_msg->firing_pkts(i), out_msg);
      // _last_time_stamp = out_msg->header.stamp;
    }
  }

  if (skip) {
    out_msg->Clear();
  } else {
    if (out_msg->point_size() == 0) {
      // we discard this pointcloud if empty
      AINFO << "All points is NAN!Please check velodyne:"
                << _config.model();
    }
    out_msg->set_width(out_msg->point_size());
  }
  // std::cout << ros::Time::Now() - begin << std::endl;
}

uint64_t Velodyne64Parser::get_timestamp(double base_time, float time_offset,
                                         uint16_t block_id) {
  double t = base_time - time_offset;
  uint64_t timestamp = 0;
  int index = 0;

  if (_is_s2) {
    index = block_id & 1;  // % 2
    double& previous_packet_stamp = _previous_packet_stamp[index];
    uint64_t& gps_base_usec = _gps_base_usec[index];
    timestamp = get_gps_stamp(t, previous_packet_stamp, gps_base_usec);
  } else {  // 64E_S3
    index = block_id & 3;  // % 4
    double& previous_packet_stamp = _previous_packet_stamp[index];
    uint64_t& gps_base_usec = _gps_base_usec[index];
    timestamp = get_gps_stamp(t, previous_packet_stamp, gps_base_usec);
  }
  return timestamp;
}

int Velodyne64Parser::intensity_compensate(const LaserCorrection& corrections,
                                           const uint16_t& raw_distance,
                                           int intensity) {

  float tmp = 1 - static_cast<float>(raw_distance) / 65535;
  intensity += corrections.focal_slope *
               (fabs(corrections.focal_offset - 256 * tmp * tmp));

  if (intensity < corrections.min_intensity) {
    intensity = corrections.min_intensity;
  }

  if (intensity > corrections.max_intensity) {
    intensity = corrections.max_intensity;
  }
  return intensity;
}

void Velodyne64Parser::unpack(
    const proto::VelodynePacket& pkt,
    std::shared_ptr<proto::PointCloud>& pc) {

  const RawPacket* raw = (const RawPacket*)pkt.data().c_str();
  double basetime = raw->gps_timestamp;  // usec

  for (int i = 0; i < BLOCKS_PER_PACKET; i++) {  // 12
    if (_mode != DUAL && !_is_s2 && ((i & 3) >> 1) > 0) {
      // i%4/2  even-numbered block contain duplicate data
      continue;
    }

    // upper bank lasers are numbered [0..31], lower bank lasers are [32..63]
    // NOTE: this is a change from the old velodyne_common implementation
    int bank_origin = (raw->blocks[i].laser_block_id == LOWER_BANK) ? 32 : 0;

    for (int j = 0, k = 0; j < SCANS_PER_BLOCK;
         ++j, k += RAW_SCAN_SIZE) {  // 32, 3
      // One point
      uint8_t laser_number;  ///< hardware laser number
      laser_number = j + bank_origin;
      LaserCorrection& corrections =
          _calibration._laser_corrections[laser_number];

      union RawDistance raw_distance;
      raw_distance.bytes[0] = raw->blocks[i].data[k];
      raw_distance.bytes[1] = raw->blocks[i].data[k + 1];

      // compute time
      uint64_t timestamp = get_timestamp(basetime, (*_inner_time)[i][j], i);

      if (j == SCANS_PER_BLOCK - 1) {
        // set header stamp before organize the point cloud
        double d_time = apollo::cybertron::Time(timestamp).ToSecond();
        pc->set_measurement_time(d_time);
        pc->mutable_header()->set_lidar_timestamp(d_time * 1e9);
      }

      float distance = raw_distance.raw_distance * DISTANCE_RESOLUTION +
                       corrections.dist_correction;

      if (raw_distance.raw_distance == 0 ||
          !is_scan_valid(raw->blocks[i].rotation, distance)) {
        // if orgnized append a nan point to the cloud
        if (_config.organized()) {
          proto::PointXYZIT* point = pc->add_point();
          point->set_x(nan);
          point->set_y(nan);
          point->set_z(nan);
          point->set_stamp(timestamp);
          point->set_intensity(0);

        }
        continue;
      }

      proto::PointXYZIT* point = pc->add_point();
      // MOCK for test
      // point->set_stamp(cybertron::Time().Now().to_nanosecond() - 1000000000);
      point->set_stamp(timestamp);
      // Position Calculation, append this point to the cloud
      compute_coords(raw_distance, corrections, raw->blocks[i].rotation, point);
      point->set_intensity(intensity_compensate(
          corrections, raw_distance.raw_distance, raw->blocks[i].data[k + 2]));
    }
  }
}

void Velodyne64Parser::order(
    std::shared_ptr<proto::PointCloud>& cloud) {

  int width = 64;
  cloud->set_width(width);
  int height = cloud->point_size() / cloud->width();
  cloud->set_height(height);

  std::shared_ptr<proto::PointCloud> cloud_origin =
      std::make_shared<proto::PointCloud>();
  cloud_origin->CopyFrom(*cloud);

  for (int i = 0; i < width; ++i) {
      int col = velodyne::ORDER_64[i];

      for (int j = 0; j < height; ++j) {
          // make sure offset is initialized, should be init at setup() just once
          int row = (j + _offsets[i] + height) % height;
          int target_index = j * width + i;
          int origin_index = row * width + col;
          cloud->mutable_point(target_index)->CopyFrom(cloud_origin->point(origin_index));

      }
  }
}

}  // namespace velodyne_data
}
}
