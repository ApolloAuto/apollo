#pragma once
#include <memory>
#include <vector>

#include "vanjeelidar/common/super_header.hpp"

 namespace vanjee
 {
  namespace lidar
  {
    class CmdClass
    {
      public:
        uint8 MainCmd;
        uint8 SubCmd;

      public:
        CmdClass(uint8 mainCmd,uint8 subCmd)
        {
          MainCmd = mainCmd;
          SubCmd = subCmd;
        }

        CmdClass(CmdClass& cmd)
        {
          MainCmd = cmd.MainCmd;
          SubCmd = cmd.SubCmd;
        }
      
        CmdClass(){}

        std::shared_ptr<std::vector<uint8>> GetBytes()
        {
          std::shared_ptr<std::vector<uint8>> buf(new std::vector<uint8>());

          buf->emplace_back(MainCmd);
          buf->emplace_back(SubCmd);

          return buf;
        }

        uint16 GetCmdKey()
        {
          return (uint16)((MainCmd<<8)+SubCmd);
        }

        bool operator == (const CmdClass& cmd)
        {
          if(&cmd == nullptr)
          {
            return false;
          }

          return ((MainCmd == cmd.MainCmd) && (SubCmd == cmd.SubCmd));
        }

        bool operator != (const CmdClass& cmd)
        {
          return !(*this == cmd);
        }
    };
  }
 }