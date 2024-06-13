#pragma once

#include <vanjeelidar/driver/difop/cmd_class.hpp>

namespace vanjee
{
  namespace lidar
  {
    class CmdRepository720_32
    {
      public:
        const std::shared_ptr<CmdClass> sp_ld_angle_get_ = std::make_shared<CmdClass>(0x05,0x14);
        const std::shared_ptr<CmdClass> sp_imu_line_Param_get_ = std::make_shared<CmdClass>(0x06,0x12);
        const std::shared_ptr<CmdClass> sp_imu_add_Param_get_ = std::make_shared<CmdClass>(0x06,0x14);
        const std::shared_ptr<CmdClass> sp_temperature_param_get_ = std::make_shared<CmdClass>(0x04,0x02);
        static CmdRepository720_32* CreateInstance()
        {
          if(p_CmdRepository720_32 == nullptr)
            p_CmdRepository720_32 = new CmdRepository720_32();

          return p_CmdRepository720_32;
        }
      private:
        static CmdRepository720_32* p_CmdRepository720_32;
        CmdRepository720_32(){}
        CmdRepository720_32(const CmdRepository720_32&) = delete;
        CmdRepository720_32& operator=(const CmdRepository720_32&) = delete;
    };

    CmdRepository720_32* CmdRepository720_32::p_CmdRepository720_32 = nullptr;
  }
}