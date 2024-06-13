
#pragma once
#include "modules/drivers/lidar/vanjeelidar/common/common_header.h"

namespace apollo {
namespace drivers {
namespace vanjee {

#ifdef _MSC_VER
struct __declspec(align(16)) Packet
#elif __GNUC__
struct __attribute__((aligned(16))) Packet  ///< LiDAR single packet message
#endif
{
  double timestamp = 0.0f;    
  uint32_t seq = 0;           
  uint8_t is_difop = 0;       
  uint8_t is_frame_begin = 0; 
  std::vector<uint8_t> buf_;

  Packet(const Packet &msg)
  {
      buf_.assign(msg.buf_.begin(), msg.buf_.end());
  }
  Packet(size_t size = 0)
  {
      buf_.resize(size);
  }
};

}  // namespace robosense
}  // namespace drivers
}  // namespace apollo