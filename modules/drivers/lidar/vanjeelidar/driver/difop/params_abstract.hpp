#pragma once
#include <memory>
#include <iostream>

#include "vanjeelidar/common/super_header.hpp"
#include "protocol_base.hpp"

namespace vanjee
{
  namespace lidar
  {
    class ParamsAbstract
    {
      public:
        int32 CommonRep;
      public:
        virtual void Load(ProtocolBase& protocol) { };
        virtual std::shared_ptr<std::vector<uint8>> GetBytes() {return nullptr;};
    };
  }
}