
#include <fstream>

#include "cybertron/common/log.h"
#include "sensor/velodyne/velodyne_parser.h"

namespace apollo {
namespace sensor {
namespace velodyne {

Velodyne16Parser::Velodyne16Parser(proto::VelodyneConfig& config)
    : VelodyneParser(config), _previous_packet_stamp(0), _gps_base_usec(0) {
  _inner_time = &velodyne::INNER_TIME_16;
  _need_two_pt_correction = false;
}

void Velodyne16Parser::generate_pointcloud(
    const std::shared_ptr<proto::VelodyneScan const>& scan_msg,
    std::shared_ptr<proto::PointCloud>& out_msg) {
  // allocate a point cloud with same time and frame ID as raw data
  out_msg->mutable_header()->set_frame_id(scan_msg->header().frame_id());      
  out_msg->mutable_header()->set_stamp(apollo::cybertron::Time().Now().ToNanosecond());
  out_msg->mutable_header()->set_timestamp_sec(apollo::cybertron::Time().Now().ToSecond());      
  out_msg->set_height(1);
  _gps_base_usec = scan_msg->basetime() * 1000000UL;

  for (int i = 0; i < scan_msg->firing_pkts_size(); ++i) {
    unpack(scan_msg->firing_pkts(i), out_msg);
    _last_time_stamp = out_msg->header().stamp();      
  }

  if (out_msg->point_size() == 0) {
    // we discard this pointcloud if empty
    AERROR << "All points is NAN!Please check velodyne:" << _config.model();
  } else {
    auto size = out_msg->point_size();
    uint64_t timestamp = out_msg->point(size - 1).stamp();
    double d_time = apollo::cybertron::Time(timestamp).ToSecond();
    //FOR perception,perception set lidar timestamp by measurement * 1e9
    out_msg->set_measurement_time(d_time);
    out_msg->mutable_header()->set_lidar_timestamp(d_time * 1e9);
  }
}

uint64_t Velodyne16Parser::get_timestamp(double base_time, float time_offset,
                                         uint16_t block_id) {
  (void)block_id;
  double t = base_time - time_offset;
  uint64_t timestamp = Velodyne16Parser::get_gps_stamp(
      t, _previous_packet_stamp, _gps_base_usec);
  return timestamp;
}

/** @brief convert raw packet to point cloud
 *
 *  @param pkt raw packet to unpack
 *  @param pc shared pointer to point cloud (points are appended)
 */
void Velodyne16Parser::unpack(
    const proto::VelodynePacket& pkt,
    std::shared_ptr<proto::PointCloud>& pc) {

  float azimuth_diff = 0.0;
  float last_azimuth_diff = 0.0;
  float azimuth_corrected_f = 0.0;
  int azimuth_corrected = 0.0;

  const RawPacket* raw = (const RawPacket*)pkt.data().c_str();
  double basetime = raw->gps_timestamp;  // usec

  for (int block = 0; block < BLOCKS_PER_PACKET; block++) {
    float azimuth = (float)(raw->blocks[block].rotation);
    if (block < (BLOCKS_PER_PACKET - 1)) {
      azimuth_diff = (float)((36000 + raw->blocks[block + 1].rotation -
                              raw->blocks[block].rotation) %
                             36000);
      last_azimuth_diff = azimuth_diff;
    } else {
      azimuth_diff = last_azimuth_diff;
    }

    for (int firing = 0, k = 0; firing < VLP16_FIRINGS_PER_BLOCK; ++firing) {
      for (int dsr = 0; dsr < VLP16_SCANS_PER_FIRING;
           ++dsr, k += RAW_SCAN_SIZE) {
        LaserCorrection& corrections = _calibration._laser_corrections[dsr];

        /** Position Calculation */
        union RawDistance raw_distance;
        raw_distance.bytes[0] = raw->blocks[block].data[k];
        raw_distance.bytes[1] = raw->blocks[block].data[k + 1];

        /** correct for the laser rotation as a function of timing during the
         * firings **/
        azimuth_corrected_f =
            azimuth + (azimuth_diff * ((dsr * VLP16_DSR_TOFFSET) +
                                       (firing * VLP16_FIRING_TOFFSET)) /
                       VLP16_BLOCK_TDURATION);
        azimuth_corrected = (int)round(fmod(azimuth_corrected_f, 36000.0));

        // set 4th param to LOWER_BANK, only use _lower_gps_base_usec and
        // _lower_previous_packet_stamp
        uint64_t timestamp = get_timestamp(
            basetime,
            (*_inner_time)[block][firing * VLP16_SCANS_PER_FIRING + dsr],
            LOWER_BANK);

        if (block == BLOCKS_PER_PACKET - 1 &&
            firing == VLP16_FIRINGS_PER_BLOCK - 1 &&
            dsr == VLP16_SCANS_PER_FIRING - 1) {
          // set header stamp before organize the point cloud
          pc->mutable_header()->set_stamp(timestamp);      
        }

        float distance = raw_distance.raw_distance * DISTANCE_RESOLUTION +
                         corrections.dist_correction;

        if (raw_distance.raw_distance == 0 ||
            !is_scan_valid(azimuth_corrected, distance)) {
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
        point->set_stamp(timestamp);
        // append this point to the cloud
        compute_coords(raw_distance, corrections, azimuth_corrected, point);
        point->set_intensity(raw->blocks[block].data[k + 2]);
      }
    }
  }
}

void Velodyne16Parser::order(
    std::shared_ptr<proto::PointCloud>& cloud) {
  int width = 16;
  cloud->set_width(width);
  int height = cloud->point_size() / cloud->width();
  cloud->set_height(height);

  std::shared_ptr<proto::PointCloud> cloud_origin =
      std::make_shared<proto::PointCloud>();
  cloud_origin->CopyFrom(*cloud);

  for (int i = 0; i < width; ++i) {
      int col = velodyne::ORDER_16[i];

      for (int j = 0; j < height; ++j) {
          // make sure offset is initialized, should be init at setup() just once
          int target_index = j * width + i;
          int origin_index = j * width + col;
          cloud->mutable_point(target_index)->CopyFrom(cloud_origin->point(origin_index));

      }
  }
}

}  // namespace velodyne_data
}
}
