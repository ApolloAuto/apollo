#pragma once

#include "protocol_abstract_720.hpp"
#include "vanjeelidar/driver/decoder/wlr720/protocol/params/params_imu_add.hpp"
#include "vanjeelidar/driver/difop/params_abstract.hpp"

namespace vanjee
{
  namespace lidar
  {
    class Protocol_ImuAddGet720:public ProtocolAbstract720
    {
      public:
        Protocol_ImuAddGet720():ProtocolAbstract720(CmdRepository720::CreateInstance()->sp_imu_add_param_get_,std::make_shared<Params_IMUAdd720>())
        {
          
        }
    };
  }
}