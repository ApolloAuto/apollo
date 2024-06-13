#pragma once 
#include <memory>

#include "vanjeelidar/driver/difop/difop_base.hpp"
#include "vanjeelidar/driver/difop/protocol_base.hpp"
#include "vanjeelidar/driver/decoder/wlr720/protocol/difop_vanjee_720.hpp"
#include "vanjeelidar/driver/decoder/wlr720_32/protocol/difop_vanjee_720_32.hpp"

namespace vanjee
{
  namespace lidar
  {
    class DifopFactory
    {
      public:
        static std::shared_ptr<DifopBase> createDifop(LidarType type);
    };

    std::shared_ptr<DifopBase> DifopFactory::createDifop(LidarType type)
    {
      std::shared_ptr<DifopBase> ret_ptr;
      ProtocolBase pb;
      
      switch (type)
      {
        case LidarType::vanjee_720:
        case LidarType::vanjee_720_16:
          ret_ptr = std::make_shared<DifopVanjee720>(); 
          break;
        case LidarType::vanjee_720_32:
          ret_ptr = std::make_shared<DifopVanjee720_32>(); 
          break;
        
        default:
          exit(-1);
      }

      return ret_ptr;
    }
  }
}