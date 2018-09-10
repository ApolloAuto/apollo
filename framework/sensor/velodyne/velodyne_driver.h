
#ifndef SENSOR_VELODYNE_VELODYNE_DRIVER_H_
#define SENSOR_VELODYNE_VELODYNE_DRIVER_H_

#include <sys/epoll.h>
#include <iostream>
#include <string>

#include "cybertron/common/log.h"
#include "cybertron/base/object_pool.h"
#include "cybertron/node/node.h"

#include "sensor/proto/sensor_velodyne.pb.h"
#include "sensor/proto/velodyne_config.pb.h"

#include "sensor/epoll_dispatcher.h"
#include "sensor/sensor.h"
#include "sensor/velodyne/data_type.h"

namespace apollo {
namespace sensor {
namespace velodyne {

// TODO: static interface
class VelodyneDriver : public Sensor,
                       public std::enable_shared_from_this<VelodyneDriver> {
 public:
  using VelodyneScanPtr = std::shared_ptr<proto::VelodyneScan>;
  using Deleter = std::function<void(proto::VelodyneScan*)>;
  // TODO: move node into base driver
  VelodyneDriver(const proto::VelodyneConfig& velodyne_config,
                 const std::shared_ptr<cybertron::Node>& node);
  virtual ~VelodyneDriver();

  virtual bool ReceiveFiringData(const epoll_event& evt);
  virtual bool ReceivePositionData(const epoll_event& evt);
  virtual void Init(const std::shared_ptr<EpollDispatcher>& dispatcher);

 protected:
  bool SetBasetime();
  void SetBasetimeFromNmea(const NMEATimePtr& nmea_time, uint64_t& basetime);
  void UpdateGpsTopHour(unsigned int current_time);
  float GetPacketRate(const proto::Model& model);
  bool GetFiringPacket(const epoll_event& evt, proto::VelodynePacket* packet);
  bool GetPositioningPacktet(const epoll_event& evt,
                             const NMEATimePtr& nmea_time);
  bool ExractNmeaTime(const NMEATimePtr& nmea_time, const uint8_t* bytes);
  // void MyDeleter(proto::VelodyneScan* scan);

  proto::VelodyneConfig config_;
  // TODO:node write in base
  std::shared_ptr<EpollDispatcher> dispatcher_ = nullptr;
  std::shared_ptr<cybertron::Node> node_ = nullptr;
  std::shared_ptr<cybertron::Writer<proto::VelodyneScan>> writer_ = nullptr;

  std::shared_ptr<proto::VelodyneScan> scan_ptr_;
  std::shared_ptr<cybertron::base::ObjectPool<proto::VelodyneScan>> scan_pool_;
  // Deleter deleter_;

  uint64_t basetime_ = 0;
  uint32_t last_gps_time_ = 0;
  int sockfd_ = -1;
  uint32_t port_ = 0;
  uint32_t packet_count_ = 0;
};
}
}
}  // namespace velodyne_driver

#endif  // SENSOR_VELODYNE_VELODYNE_DRIVER_H_
