#pragma once

#include "vanjeelidar/driver/difop/params_abstract.hpp"
#include "vanjeelidar/common/super_header.hpp"

namespace vanjee
{
  namespace lidar
  {
    class Params_LDAngle720 : public ParamsAbstract
    {
      public:
        uint8 lines_num_;
        /// @brief 1000 times the real vertical angle
        std::array<int32,16> ver_angle_;

        std::array<int32,16> hor_angle_;
      public:
        virtual std::shared_ptr<std::vector<uint8>> GetBytes()
        {
          std::shared_ptr<std::vector<uint8>> buf = std::make_shared<std::vector<uint8>>();
          return nullptr;
        }

        virtual void Load(ProtocolBase& protocol)
        {
          auto buf = protocol.Content.data();
          lines_num_ = *buf;
          int32* data = reinterpret_cast<int32*>(buf+1);
          std::copy(data,data+ver_angle_.size(),std::begin(ver_angle_));
        }
    };
  }
}