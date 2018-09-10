
#include <time.h>
#include <cmath>
#include <cstring>
#include <string>

#include "sensor/fd_util.h"
#include "sensor/velodyne/velodyne_driver.h"

namespace apollo {
namespace sensor {
namespace velodyne {

VelodyneDriver::VelodyneDriver(const proto::VelodyneConfig& velodyne_config,
                               const std::shared_ptr<cybertron::Node>& node)
    : config_(velodyne_config), node_(node), basetime_(0), last_gps_time_(0) {}
VelodyneDriver::~VelodyneDriver(void) { (void)close(sockfd_); }

float VelodyneDriver::GetPacketRate(const proto::Model& model) {
  switch (model) {
    case proto::HDL64E_S2:
      return PACKETCOUNT_ONESECOND_64E_S2;
    case proto::HDL64E_S3S:
      return PACKETCOUNT_ONESECOND_64E_S3S;
    case proto::HDL64E_S3D:
      return PACKETCOUNT_ONESECOND_64E_S3D;
    default:
      return PACKETCOUNT_ONESECOND_64E_S3D;
  }
}

void VelodyneDriver::SetBasetimeFromNmea(const NMEATimePtr& nmea_time,
                                         uint64_t& basetime) {
  tm time;
  memset(&time, 0, sizeof(time));

  time.tm_year = nmea_time->year + (2000 - 1900);
  time.tm_mon = nmea_time->mon - 1;
  time.tm_mday = nmea_time->day;
  time.tm_hour = nmea_time->hour + config_.time_zone();
  time.tm_min = 0;
  time.tm_sec = 0;

  // set last gps time using gps socket packet
  last_gps_time_ = (nmea_time->min * 60 + nmea_time->sec) * 1e6;

  uint64_t unix_base = static_cast<uint64_t>(mktime(&time));
  basetime = unix_base;
}

void VelodyneDriver::Init(const std::shared_ptr<EpollDispatcher>& dispatcher) {
  port_ = config_.firing_data_port();
  InitUdp(nullptr, port_, sockfd_);

  float packet_rate = GetPacketRate(config_.model());
  ;
  float frequency = (config_.rpm() / 60.0);  // expected Hz rate

  config_.set_npackets(ceil(packet_rate / frequency));

  int npackets = config_.npackets();
  int packet_size = PACKET_SIZE;
  AINFO << "nnn:" << npackets;
  std::function<void(proto::VelodyneScan*)> init_func =
      [=](proto::VelodyneScan* scan) {
        for (int i = 0; i < npackets; ++i) {
          auto packet = scan->add_firing_pkts();
          packet->mutable_data()->reserve(packet_size);
        }
      };
  scan_pool_ = cybertron::base::ObjectPool<proto::VelodyneScan>::Instance(
      24, init_func);
  scan_ptr_ = scan_pool_->GetObject();
  // scan_pool_->ReleaseObject(scan_ptr_);
  // scan_ptr_ = scan_pool_->GetObject();

  // deleter_ = [this](proto::VelodyneScan* scan) {
  //   scan_pool_->ReleaseObject(scan);
  // };
  cybertron::proto::RoleAttributes attr;
  attr.set_channel_name(config_.scan_channel());
  writer_ = node_->CreateWriter<proto::VelodyneScan>(attr);

  AINFO << "nnn:" << npackets;
  std::weak_ptr<VelodyneDriver> weak_this = shared_from_this();
  dispatcher->AddHandler(sockfd_, EPOLLIN | EPOLLET,
                         [weak_this](const epoll_event& evt) {
                           auto share_this = weak_this.lock();
                           if (share_this) {
                             share_this->ReceiveFiringData(evt);
                           }
                         });
}

bool VelodyneDriver::SetBasetime() { return true; }

// void VelodyneDriver::MyDeleter(proto::VelodyneScan* scan) {
//   scan_pool_->ReleaseObject(scan);
// }

bool VelodyneDriver::ReceiveFiringData(const epoll_event& evt) {
  auto packet = scan_ptr_->mutable_firing_pkts(packet_count_);
  if (GetFiringPacket(evt, packet)) {
    ++packet_count_;
  } else {
    AERROR << "get packet fail";
  }
  if (packet_count_ == config_.npackets()) {
    // TODO send
    AINFO << "send scan start:" << config_.npackets();
    // TODO:new raw pointer?
    // std::shared_ptr<proto::VelodyneScan> send_scan(
    //     new proto::VelodyneScan(std::move(*scan_ptr_)), deleter_);
    //        [this](proto::VelodyneScan* scan){
    //          scan_pool_->ReleaseObject(scan);
    //        });
    auto send_scan = scan_ptr_;
    writer_->Write(scan_ptr_);
    scan_ptr_ = scan_pool_->GetObject();
    AINFO << "send scan end:" << config_.npackets();
    packet_count_ = 0;
  }
}

bool VelodyneDriver::ReceivePositionData(const epoll_event& evt) {
  return false;
}

void VelodyneDriver::UpdateGpsTopHour(uint32_t current_time) {
  if (last_gps_time_ == 0) {
    last_gps_time_ = current_time;
    return;
  }
  if (last_gps_time_ > current_time) {
    if (std::abs(last_gps_time_ - current_time) > 3599000000) {
      basetime_ += 3600;
    } else {
    }
  }
  last_gps_time_ = current_time;
}

bool VelodyneDriver::GetFiringPacket(const epoll_event& evt,
                                     proto::VelodynePacket* packet) {
  // TODO check evt poll type, set packet directly?
  // Receive packets that should now be available from the
  // socket using a blocking read.
  uint8_t bytes[PACKET_SIZE];
  int nbytes = recvfrom(sockfd_, bytes, PACKET_SIZE, 0, NULL, NULL);

  if (nbytes < 0) {
    if (errno != EWOULDBLOCK) {
      AERROR << " recvfail from port " << port_;
      return false;
    }
  }

  if (nbytes != PACKET_SIZE) {
    AERROR << "wrong packet size:" << nbytes;
    return false;
  } else {
    packet->set_data(bytes, PACKET_SIZE);
  }

  return true;
}

bool VelodyneDriver::ExractNmeaTime(const NMEATimePtr& nmea_time,
                                    const uint8_t* bytes) {
  int gprmc_index = 206;

  int field_count = 0;
  int time_field_index = 0;
  int validity_field_index = 0;
  int date_field_index = 0;
  while (bytes[++gprmc_index] != '*' &&
         gprmc_index < POSITIONING_DATA_PACKET_SIZE) {
    if (bytes[gprmc_index] == ',') {
      ++field_count;
      if (field_count == 1 && time_field_index == 0) {
        time_field_index = gprmc_index + 1;
      } else if (field_count == 2 && validity_field_index == 0) {
        validity_field_index = gprmc_index + 1;
        if (bytes[validity_field_index] == 'V') {
          std::cout << " NAV receiver warning, GPS info is invalid!"
                    << std::endl;
          return false;
        }
      } else if (field_count == 9 && date_field_index == 0) {
        date_field_index = gprmc_index + 1;
        break;
      }
    }
  }

  nmea_time->year = (bytes[date_field_index + 4] - '0') * 10 +
                    (bytes[date_field_index + 5] - '0');
  nmea_time->mon = (bytes[date_field_index + 2] - '0') * 10 +
                   (bytes[date_field_index + 3] - '0');
  nmea_time->day = (bytes[date_field_index] - '0') * 10 +
                   (bytes[date_field_index + 1] - '0');
  nmea_time->hour = (bytes[time_field_index] - '0') * 10 +
                    (bytes[time_field_index + 1] - '0');
  nmea_time->min = (bytes[time_field_index + 2] - '0') * 10 +
                   (bytes[time_field_index + 3] - '0');
  nmea_time->sec = (bytes[time_field_index + 4] - '0') * 10 +
                   (bytes[time_field_index + 5] - '0');

  if (nmea_time->year > 99 || nmea_time->mon > 12 || nmea_time->mon < 1 ||
      nmea_time->day > 31 || nmea_time->day < 1 || nmea_time->hour > 23 ||
      nmea_time->min > 59 || nmea_time->sec > 59) {
    std::cout << " invalid gps time" << std::endl;
    return false;
  } else if (nmea_time->min == 59 && nmea_time->sec >= 57) {
    std::cout << "discard hour bound,sec:" << nmea_time->sec << std::endl;
    return false;
  }
  return true;
}

bool VelodyneDriver::GetPositioningPacktet(const epoll_event& evt,
                                           const NMEATimePtr& nmea_time) {
  // Last 234 bytes not use
  uint8_t bytes[POSITIONING_DATA_PACKET_SIZE];
  ssize_t nbytes =
      recvfrom(sockfd_, bytes, POSITIONING_DATA_PACKET_SIZE, 0, NULL, NULL);

  if (nbytes < 0) {
    if (errno != EWOULDBLOCK) {
      AERROR << " recvfail from port " << port_;
      return false;
    }
  }

  if ((size_t)nbytes == POSITIONING_DATA_PACKET_SIZE) {
    // read successful, exract nmea time
    return ExractNmeaTime(nmea_time, bytes);
  } else {
    return false;
  }
}
}
}
}  // namespace velodyne_driver
