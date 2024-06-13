#pragma once

#include "protocol_abstract_720.hpp"
#include "vanjeelidar/driver/decoder/wlr720/protocol/params/params_imu_line.hpp"
#include "vanjeelidar/driver/difop/params_abstract.hpp"

namespace vanjee
{
  namespace lidar
  {
    class Protocol_ImuLineGet720:public ProtocolAbstract720
    {
      public:
        Protocol_ImuLineGet720():ProtocolAbstract720(CmdRepository720::CreateInstance()->sp_imu_line_param_get_,std::make_shared<Params_IMULine720>())
        {
          
        }
    };
  }
}