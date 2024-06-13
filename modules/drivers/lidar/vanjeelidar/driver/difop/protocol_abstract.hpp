#pragma once
#include <memory>
#include <vector>

#include "vanjeelidar/common/super_header.hpp"
#include "cmd_class.hpp"
#include "params_abstract.hpp"
#include "protocol_base.hpp"


namespace vanjee
{
  namespace lidar
  {
    class ProtocolAbstract
    {
      public:
        uint16        Idx;
        uint32        Timestamp;
        uint8         CheckType;
        uint8         Type;
        ByteVector    DeviceType;
        ByteVector    Remain;
        std::shared_ptr<CmdClass>  Sp_Cmd;
        ByteVector    CmdParams;
        std::shared_ptr<ParamsAbstract> Params;

      public:
        ProtocolAbstract(uint16 idx,uint32 timestamp,uint8 checkType,uint8 type,const ByteVector &deviceType,
          const ByteVector &remain,std::shared_ptr<CmdClass> sp_cmd,const ByteVector &cmdParams,std::shared_ptr<ParamsAbstract> content)
          {
            Idx = idx;
            Timestamp = timestamp;
            CheckType = checkType;
            Type = type;
            DeviceType = deviceType.size() == 2 ? deviceType : ByteVector({0x00,0x00});
            Remain = remain.size() == 8 ? remain : ByteVector({0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00});
            Sp_Cmd = sp_cmd;
            CmdParams = cmdParams;
            Params = content;
          }

        ProtocolAbstract(uint8 checkType,uint8 type,const ByteVector &deviceType,const ByteVector &remain,
          std::shared_ptr<CmdClass> sp_cmd,const ByteVector &cmdParams,std::shared_ptr<ParamsAbstract> content)
          {
            CheckType = checkType;
            Type = type;
            DeviceType = deviceType;
            Remain = remain;
            Sp_Cmd = sp_cmd;
            CmdParams = cmdParams;
            Params = content;
          }

        virtual bool Load(ProtocolBase protocol)
        {
          Idx = *reinterpret_cast<uint16*>(protocol.Idx.data());
          Timestamp = *reinterpret_cast<uint32*>(protocol.Timestamp.data());
          CheckType = protocol.CheckType;
          Type = protocol.Type;
          DeviceType = protocol.DeviceType;
          Sp_Cmd.reset(new CmdClass(protocol.MainCmd,protocol.SubCmd));
          CmdParams = protocol.CmdParams;
          Params->Load(protocol);
          return true;
        }

        virtual std::shared_ptr<std::vector<uint8>> GetRequest(std::shared_ptr<std::vector<uint8>> content = nullptr)
        {
          if(content == nullptr)
          {
            std::array<uint8,4> arr = {0x00,0x00,0x00,0x00};
            content = std::make_shared<std::vector<uint8>>(arr.begin(),arr.begin()+sizeof(arr)/sizeof(uint8));
          }

          ProtocolBase pb(Idx,Timestamp,CheckType,Type,DeviceType,Remain,Sp_Cmd->MainCmd,Sp_Cmd->SubCmd,
            CmdParams,*content);
          return pb.GetBytes();
        }

        virtual std::shared_ptr<std::vector<uint8>> SetRequest()
        {
          ProtocolBase pb(Idx, Timestamp, CheckType, Type, DeviceType, Remain, Sp_Cmd->MainCmd, Sp_Cmd->SubCmd,
            CmdParams,*Params->GetBytes());

          return pb.GetBytes();
        }

    };
  }
}//namespace vanjee