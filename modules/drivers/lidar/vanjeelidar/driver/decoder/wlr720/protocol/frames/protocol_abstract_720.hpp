#pragma once

#include <memory>
#include <vector>

#include "vanjeelidar/driver/difop/protocol_abstract.hpp"
#include "vanjeelidar/common/super_header.hpp"

namespace vanjee
{
  namespace lidar
  {
    class ProtocolAbstract720:public ProtocolAbstract
    {
      public:
        ProtocolAbstract720(const std::shared_ptr<CmdClass> sp_cmd,std::shared_ptr<ParamsAbstract> content,uint16 idx = 0,uint32 timestamp = 0,
          uint8 checkType = 1,uint8 type = 1):ProtocolAbstract(checkType,type,{0x00,0x0B},
          {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},sp_cmd,{0x00,0x00},content)
          {

          }

        virtual bool Load(ProtocolBase protocol)
        {
          CheckType = protocol.CheckType;
          Type = protocol.Type;
          Sp_Cmd.reset(new CmdClass(protocol.MainCmd,protocol.SubCmd));
          CmdParams = protocol.CmdParams;
          Params->Load(protocol);
          return true;
        }

        virtual std::shared_ptr<std::vector<uint8>> GetRequest(std::shared_ptr<std::vector<uint8>> content = nullptr) override
        {
          if(content == nullptr)
          {
            const uint8 arr[] = {0x00,0x00,0x00,0x00};
            content = std::make_shared<std::vector<uint8>>();
            content->insert(content->end(),arr,arr+sizeof(arr)/sizeof(uint8));
          }
          
          return (std::make_shared<ProtocolBase>(ByteVector({0x00,0x00}),ByteVector({0x00,0x00,0x00,0x00}),CheckType,Type,
            DeviceType,Remain,Sp_Cmd->MainCmd,Sp_Cmd->SubCmd,CmdParams,*content))->GetBytes();
        }

        virtual std::shared_ptr<std::vector<uint8>> SetRequest() override
        {
          return (std::make_shared<ProtocolBase>(ByteVector({0x00,0x00}),ByteVector({0x00,0x00,0x00,0x00}),CheckType,Type,
            DeviceType,Remain,Sp_Cmd->MainCmd,Sp_Cmd->SubCmd,CmdParams,*Params->GetBytes()))->GetBytes();
        }
    };
  }
}