#pragma once

#include <vanjeelidar/driver/difop/cmd_class.hpp>

namespace vanjee
{
  namespace lidar
  {
    class CmdRepository720
    {
      public:
        const std::shared_ptr<CmdClass> sp_ld_angle_get_ = std::make_shared<CmdClass>(0x05,0x14);
        const std::shared_ptr<CmdClass> sp_imu_line_param_get_ = std::make_shared<CmdClass>(0x06,0x12);
        const std::shared_ptr<CmdClass> sp_imu_add_param_get_ = std::make_shared<CmdClass>(0x06,0x14);
        const std::shared_ptr<CmdClass> sp_temperature_param_get_ = std::make_shared<CmdClass>(0x04,0x02);
        static CmdRepository720* CreateInstance()
        {
          if(p_CmdRepository720 == nullptr)
            p_CmdRepository720 = new CmdRepository720();

          return p_CmdRepository720;
        }
      private:
        static CmdRepository720* p_CmdRepository720;
        CmdRepository720(){}
        CmdRepository720(const CmdRepository720&) = delete;
        CmdRepository720& operator=(const CmdRepository720&) = delete;
    };

    CmdRepository720* CmdRepository720::p_CmdRepository720 = nullptr;
  }
}