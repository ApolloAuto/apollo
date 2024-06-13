#pragma once

#include "protocol_abstract_720.hpp"
#include "vanjeelidar/driver/decoder/wlr720/protocol/params/params_lidar_run_status.hpp"
#include "vanjeelidar/driver/difop/params_abstract.hpp"

namespace vanjee
{
  namespace lidar
  {
    class Protocol_ImuTempGet : public ProtocolAbstract720
    {
      public:
        Protocol_ImuTempGet():ProtocolAbstract720(CmdRepository720::CreateInstance()->sp_temperature_param_get_,std::make_shared<Params_LiDARRunStatus>())
        {
          
        }
    };
  }
}