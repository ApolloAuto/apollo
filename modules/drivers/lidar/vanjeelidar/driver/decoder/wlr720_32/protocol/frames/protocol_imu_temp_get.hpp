#pragma once

#include "protocol_abstract_720_32.hpp"
#include "vanjeelidar/driver/decoder/wlr720_32/protocol/params/params_lidar_run_status.hpp"
#include "vanjeelidar/driver/difop/params_abstract.hpp"

namespace vanjee
{
  namespace lidar
  {
    class Protocol_ImuTempGet720_32 : public ProtocolAbstract720_32
    {
      public:
        Protocol_ImuTempGet720_32():ProtocolAbstract720_32(CmdRepository720_32::CreateInstance()->sp_temperature_param_get_,std::make_shared<Params_LiDARRunStatus720_32>())
        {
          
        }
    };
  }
}