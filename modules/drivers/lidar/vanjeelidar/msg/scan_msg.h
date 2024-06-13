
#pragma once
#include "modules/drivers/lidar/vanjeelidar/common/common_header.h"
#include "modules/drivers/lidar/vanjeelidar/msg/packet_msg.h"

namespace apollo {
namespace drivers {
namespace vanjee {
#ifdef _MSC_VER
struct __declspec(align(16)) ScanMsg
#elif __GNUC__
struct __attribute__((aligned(16))) ScanMsg
#endif
{
  double timestamp = 0.0;
  uint32_t seq = 0;
  std::string frame_id = "";
  std::vector<Packet> packets;  ///< A vector which store a scan of packets (the size of the vector is not fix)
};

}  // namespace robosense
}  // namespace drivers
}  // namespace apollo