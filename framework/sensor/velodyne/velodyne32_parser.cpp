
#include <fstream>

#include "cybertron/common/log.h"
#include "sensor/velodyne/velodyne_parser.h"

namespace apollo {
namespace sensor {
namespace velodyne {

Velodyne32Parser::Velodyne32Parser(proto::VelodyneConfig& config)
    : VelodyneParser(config), _previous_packet_stamp(0), _gps_base_usec(0) {
  _inner_time = &velodyne::INNER_TIME_32;
  _need_two_pt_correction = false;
}

void Velodyne32Parser::generate_pointcloud(
    const std::shared_ptr<proto::VelodyneScan const>& scan_msg,
    std::shared_ptr<proto::PointCloud>& out_msg) {

  // allocate a point cloud with same time and frame ID as raw data
  out_msg->mutable_header()->set_frame_id(_config.frame_id());
  out_msg->mutable_header()->set_stamp(apollo::cybertron::Time().Now().ToNanosecond());
  out_msg->set_height(1);
  _gps_base_usec = scan_msg->basetime();

  for (int i = 0; i < scan_msg->firing_pkts_size(); ++i) {
    unpack(scan_msg->firing_pkts(i), out_msg);
    _last_time_stamp = out_msg->header().stamp();
  }

  if (out_msg->point_size() == 0) {
    // we discard this pointcloud if empty
    AERROR << "All points is NAN!Please check velodyne:" << _config.model();
  }
}

uint64_t Velodyne32Parser::get_timestamp(double base_time, float time_offset,
                                         uint16_t block_id) {
  (void)block_id;
  double t = base_time - time_offset;
  uint64_t timestamp = get_gps_stamp(t, _previous_packet_stamp, _gps_base_usec);
  return timestamp;
}

void Velodyne32Parser::unpack(
    const proto::VelodynePacket& pkt,
    std::shared_ptr<proto::PointCloud>& pc) {

  const RawPacket* raw = (const RawPacket*)pkt.data().c_str();
  double basetime = raw->gps_timestamp;  // usec

  for (int i = 0; i < BLOCKS_PER_PACKET; i++) {  // 12
    if (_config.mode() == proto::STRONGEST ||
        _config.mode() == proto::LAST) {
      if (i % 4 / 2 > 0) {  // even-numbered block contain duplicate data
        continue;
      }
    }

    // upper bank lasers are numbered [0..31], lower bank lasers are [32..63]
    // NOTE: this is a change from the old velodyne_common implementation
    // int bank_origin = (raw->blocks[i].laser_block_id == LOWER_BANK) ? 32 : 0;

    for (int laser_id = 0, k = 0; laser_id < SCANS_PER_BLOCK;
         ++laser_id, k += RAW_SCAN_SIZE) {  // 32, 3
      LaserCorrection& corrections = _calibration._laser_corrections[laser_id];

      union RawDistance raw_distance;
      raw_distance.bytes[0] = raw->blocks[i].data[k];
      raw_distance.bytes[1] = raw->blocks[i].data[k + 1];

      // compute time
      uint64_t timestamp =
          get_timestamp(basetime, (*_inner_time)[i][laser_id], i);

      if (laser_id == SCANS_PER_BLOCK - 1) {
        // set header stamp before organize the point cloud
        pc->mutable_header()->set_stamp(timestamp);
      }

      int rotation = (int)raw->blocks[i].rotation;
      float distance = raw_distance.raw_distance * DISTANCE_RESOLUTION +
                       corrections.dist_correction;

      if (raw_distance.raw_distance == 0 ||
          !is_scan_valid(rotation, distance)) {
        // if orgnized append a nan point to the cloud
        if (_config.organized()) {
          pc->add_point();
          // ++pc.width;
        }
        continue;
      }

      proto::PointXYZIT* point = pc->add_point();
      point->set_stamp(timestamp);
      // Position Calculation, append this point to the cloud
      compute_coords(raw_distance, corrections, rotation, point);
      point->set_intensity(raw->blocks[i].data[k + 2]);
      // // append this point to the cloud
      // pc.points.emplace_back(point);
      // ++pc.width;
    }
  }
}

void Velodyne32Parser::order(
    std::shared_ptr<proto::PointCloud>& cloud) {
  (void)cloud;
  // int width = 32;
  // cloud->width = width;
  // cloud->height = cloud->size() / cloud->width;
  // int height = cloud->height;

  // VPointCloud target;
  // target.header = cloud->header;
  // target.resize(cloud->size());
  // target.width = width;
  // target.height = height;

  // for (int i = 0; i < width; ++i) {
  //     int col = velodyne::ORDER_32[i];

  //     for (int j = 0; j < height; ++j) {
  //         target.at(i, j) = cloud->at(col, j);
  //     }
  // }
  // *cloud = target;
}

}
}
}
