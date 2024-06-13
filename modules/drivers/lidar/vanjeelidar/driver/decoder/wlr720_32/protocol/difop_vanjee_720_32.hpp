#pragma once
#include <memory>
#include <vector>

#include "vanjeelidar/common/super_header.hpp"
#include "vanjeelidar/driver/difop/protocol_abstract.hpp"
#include "vanjeelidar/driver/difop/difop_base.hpp"


namespace vanjee
{
  namespace lidar
  {
    class DifopVanjee720_32 : public DifopBase
    {
      public:
        virtual void initGetDifoCtrlDataMapPtr();
    };

    void DifopVanjee720_32::initGetDifoCtrlDataMapPtr()
    {
      getDifoCtrlData_map_ptr_ = std::make_shared<std::map<uint16,GetDifoCtrlClass>>(); 

      GetDifoCtrlClass getDifoCtrlData_LdAngleGet(*(std::make_shared<Protocol_LDAngleGet720_32>()->GetRequest()));
      (*getDifoCtrlData_map_ptr_).emplace(CmdRepository720_32::CreateInstance()->sp_ld_angle_get_->GetCmdKey(),getDifoCtrlData_LdAngleGet);

      GetDifoCtrlClass getDifoCtrlData_ImuLineGet(*(std::make_shared<Protocol_ImuLineGet720_32>()->GetRequest()));
      (*getDifoCtrlData_map_ptr_).emplace(CmdRepository720_32::CreateInstance()->sp_imu_line_Param_get_->GetCmdKey(),getDifoCtrlData_ImuLineGet);

      GetDifoCtrlClass getDifoCtrlData_IMUAddGet(*(std::make_shared<Protocol_ImuAddGet720_32>()->GetRequest()));
      (*getDifoCtrlData_map_ptr_).emplace(CmdRepository720_32::CreateInstance()->sp_imu_add_Param_get_->GetCmdKey(),getDifoCtrlData_IMUAddGet);

       GetDifoCtrlClass getDifoCtrlData_ImuTempGet(*(std::make_shared<Protocol_ImuTempGet720_32>()->GetRequest()),false,10000);
      (*getDifoCtrlData_map_ptr_).emplace(CmdRepository720_32::CreateInstance()->sp_temperature_param_get_->GetCmdKey(),getDifoCtrlData_ImuTempGet);
    }
  }
}
