#pragma once

#include "vanjeelidar/driver/difop/params_abstract.hpp"
#include "vanjeelidar/common/super_header.hpp"

namespace vanjee
{
  namespace lidar
  {
    class Params_LiDARRunStatus : public ParamsAbstract
    {
      public:
        std::array<uint8,19> reserved_field_1_;
        uint16 imu_temp_;
        std::array<uint8,26> reserved_field_2_;
      public:
        virtual std::shared_ptr<std::vector<uint8>> GetBytes()
        {
          std::shared_ptr<std::vector<uint8>> buf = std::make_shared<std::vector<uint8>>();
          return nullptr;
        }

        virtual void Load(ProtocolBase& protocol)
        {
          auto buf = protocol.Content.data();
          imu_temp_ = ((*(buf+19) & 0xFF) << 8) + (*(buf+20) & 0xFF);
        }
    };
  }
}