#pragma once

#include "protocol_abstract_720_32.hpp"
#include "vanjeelidar/driver/decoder/wlr720_32/protocol/params/params_ld_angle.hpp"
#include "vanjeelidar/driver/difop/params_abstract.hpp"

namespace vanjee
{
  namespace lidar
  {
    class Protocol_LDAngleGet720_32:public ProtocolAbstract720_32
    {
      public:
        Protocol_LDAngleGet720_32():ProtocolAbstract720_32(CmdRepository720_32::CreateInstance()->sp_ld_angle_get_,std::make_shared<Params_LDAngle720_32>())
        {
          
        }
    };
  }
}