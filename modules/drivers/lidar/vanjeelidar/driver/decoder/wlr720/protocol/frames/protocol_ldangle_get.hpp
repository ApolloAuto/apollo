#pragma once

#include "protocol_abstract_720.hpp"
#include "vanjeelidar/driver/decoder/wlr720/protocol/params/params_ld_angle.hpp"
#include "vanjeelidar/driver/difop/params_abstract.hpp"

namespace vanjee
{
  namespace lidar
  {
    class Protocol_LDAngleGet720:public ProtocolAbstract720
    {
      public:
        Protocol_LDAngleGet720():ProtocolAbstract720(CmdRepository720::CreateInstance()->sp_ld_angle_get_,std::make_shared<Params_LDAngle720>())
        {
          
        }
    };
  }
}