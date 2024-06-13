#pragma once

#include "vanjeelidar/driver/difop/params_abstract.hpp"
#include "vanjeelidar/common/super_header.hpp"

namespace vanjee
{
  namespace lidar
  {
    class Params_IMULine720 : public ParamsAbstract
    {
      public:
        float x_k_;
        float x_b_;

        float y_k_;
        float y_b_;

        float z_k_;
        float z_b_;

      public:
        virtual std::shared_ptr<std::vector<uint8>> GetBytes()
        {
          std::shared_ptr<std::vector<uint8>> buf = std::make_shared<std::vector<uint8>>();
          return nullptr;
        }

        virtual void Load(ProtocolBase& protocol)
        {
          auto buf = protocol.Content.data();
        
          float* data = reinterpret_cast<float*>(buf);
          x_k_ = *data;
          data = reinterpret_cast<float*>(buf+4);
          x_b_ = *data;

          data = reinterpret_cast<float*>(buf+8);
          y_k_ = *data;
          data = reinterpret_cast<float*>(buf+12);
          y_b_ = *data;

          data = reinterpret_cast<float*>(buf+16);
          z_k_ = *data;
          data = reinterpret_cast<float*>(buf+20);
          z_b_ = *data;
          
        }
    };
  }
}