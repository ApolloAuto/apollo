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
    class DifopVanjee720 : public DifopBase
    {
      public:
        virtual void initGetDifoCtrlDataMapPtr();
    };

    void DifopVanjee720::initGetDifoCtrlDataMapPtr()
    {
      getDifoCtrlData_map_ptr_ = std::make_shared<std::map<uint16,GetDifoCtrlClass>>(); 

      GetDifoCtrlClass getDifoCtrlData_LdAngleGet(*(std::make_shared<Protocol_LDAngleGet720>()->GetRequest()));
      (*getDifoCtrlData_map_ptr_).emplace(CmdRepository720::CreateInstance()->sp_ld_angle_get_->GetCmdKey(),getDifoCtrlData_LdAngleGet);

      GetDifoCtrlClass getDifoCtrlData_ImuLineGet(*(std::make_shared<Protocol_ImuLineGet720>()->GetRequest()));
      (*getDifoCtrlData_map_ptr_).emplace(CmdRepository720::CreateInstance()->sp_imu_line_param_get_->GetCmdKey(),getDifoCtrlData_ImuLineGet);

      GetDifoCtrlClass getDifoCtrlData_IMUAddGet(*(std::make_shared<Protocol_ImuAddGet720>()->GetRequest()));
      (*getDifoCtrlData_map_ptr_).emplace(CmdRepository720::CreateInstance()->sp_imu_add_param_get_->GetCmdKey(),getDifoCtrlData_IMUAddGet);

       GetDifoCtrlClass getDifoCtrlData_ImuTempGet(*(std::make_shared<Protocol_ImuTempGet>()->GetRequest()),false,10000);
      (*getDifoCtrlData_map_ptr_).emplace(CmdRepository720::CreateInstance()->sp_temperature_param_get_->GetCmdKey(),getDifoCtrlData_ImuTempGet);
    }
  }
}
