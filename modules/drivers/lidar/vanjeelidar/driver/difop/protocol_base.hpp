#pragma once
#include <memory>
#include <vector>

#include "vanjeelidar/common/super_header.hpp"

class CheckClass;
namespace vanjee
{
  namespace lidar
  {
    class CheckClass
    {
      public:
        enum CHECKTYPE 
        {
          No_Check = 0x00,
          Xor_Check = 0x01,
          Crc16_Check = 0x02
        };
      public:
        static uint8 Xor(std::vector<uint8>& buf,uint32 start,uint32 len)
        {
          uint8 xor_ = 0x00;

          for(uint32 i=0;i<len;i++)
          {
            xor_ ^= buf[start+i];
          }

          return xor_;
        }

        static uint16 Crc16(std::vector<uint8>& buf,uint32 start,uint32 len)
        {
          static uint16 crctab[256] =
            {
                0x0000, 0x1021, 0x2042, 0x3063, 0x4084, 0x50A5, 0x60C6, 0x70E7,
                0x8108, 0x9129, 0xA14A, 0xB16B, 0xC18C, 0xD1AD, 0xE1CE, 0xF1EF,
                0x1231, 0x0210, 0x3273, 0x2252, 0x52B5, 0x4294, 0x72F7, 0x62D6,
                0x9339, 0x8318, 0xB37B, 0xA35A, 0xD3BD, 0xC39C, 0xF3FF, 0xE3DE,
                0x2462, 0x3443, 0x0420, 0x1401, 0x64E6, 0x74C7, 0x44A4, 0x5485,
                0xA56A, 0xB54B, 0x8528, 0x9509, 0xE5EE, 0xF5CF, 0xC5AC, 0xD58D,
                0x3653, 0x2672, 0x1611, 0x0630, 0x76D7, 0x66F6, 0x5695, 0x46B4,
                0xB75B, 0xA77A, 0x9719, 0x8738, 0xF7DF, 0xE7FE, 0xD79D, 0xC7BC,
                0x48C4, 0x58E5, 0x6886, 0x78A7, 0x0840, 0x1861, 0x2802, 0x3823,
                0xC9CC, 0xD9ED, 0xE98E, 0xF9AF, 0x8948, 0x9969, 0xA90A, 0xB92B,
                0x5AF5, 0x4AD4, 0x7AB7, 0x6A96, 0x1A71, 0x0A50, 0x3A33, 0x2A12,
                0xDBFD, 0xCBDC, 0xFBBF, 0xEB9E, 0x9B79, 0x8B58, 0xBB3B, 0xAB1A,
                0x6CA6, 0x7C87, 0x4CE4, 0x5CC5, 0x2C22, 0x3C03, 0x0C60, 0x1C41,
                0xEDAE, 0xFD8F, 0xCDEC, 0xDDCD, 0xAD2A, 0xBD0B, 0x8D68, 0x9D49,
                0x7E97, 0x6EB6, 0x5ED5, 0x4EF4, 0x3E13, 0x2E32, 0x1E51, 0x0E70,
                0xFF9F, 0xEFBE, 0xDFDD, 0xCFFC, 0xBF1B, 0xAF3A, 0x9F59, 0x8F78,
                0x9188, 0x81A9, 0xB1CA, 0xA1EB, 0xD10C, 0xC12D, 0xF14E, 0xE16F,
                0x1080, 0x00A1, 0x30C2, 0x20E3, 0x5004, 0x4025, 0x7046, 0x6067,
                0x83B9, 0x9398, 0xA3FB, 0xB3DA, 0xC33D, 0xD31C, 0xE37F, 0xF35E,
                0x02B1, 0x1290, 0x22F3, 0x32D2, 0x4235, 0x5214, 0x6277, 0x7256,
                0xB5EA, 0xA5CB, 0x95A8, 0x8589, 0xF56E, 0xE54F, 0xD52C, 0xC50D,
                0x34E2, 0x24C3, 0x14A0, 0x0481, 0x7466, 0x6447, 0x5424, 0x4405,
                0xA7DB, 0xB7FA, 0x8799, 0x97B8, 0xE75F, 0xF77E, 0xC71D, 0xD73C,
                0x26D3, 0x36F2, 0x0691, 0x16B0, 0x6657, 0x7676, 0x4615, 0x5634,
                0xD94C, 0xC96D, 0xF90E, 0xE92F, 0x99C8, 0x89E9, 0xB98A, 0xA9AB,
                0x5844, 0x4865, 0x7806, 0x6827, 0x18C0, 0x08E1, 0x3882, 0x28A3,
                0xCB7D, 0xDB5C, 0xEB3F, 0xFB1E, 0x8BF9, 0x9BD8, 0xABBB, 0xBB9A,
                0x4A75, 0x5A54, 0x6A37, 0x7A16, 0x0AF1, 0x1AD0, 0x2AB3, 0x3A92,
                0xFD2E, 0xED0F, 0xDD6C, 0xCD4D, 0xBDAA, 0xAD8B, 0x9DE8, 0x8DC9,
                0x7C26, 0x6C07, 0x5C64, 0x4C45, 0x3CA2, 0x2C83, 0x1CE0, 0x0CC1,
                0xEF1F, 0xFF3E, 0xCF5D, 0xDF7C, 0xAF9B, 0xBFBA, 0x8FD9, 0x9FF8,
                0x6E17, 0x7E36, 0x4E55, 0x5E74, 0x2E93, 0x3EB2, 0x0ED1, 0x1EF0
            };
          uint16 crc16 = 0x00;

          for(uint32 i=0;i<len;i++)
          {
             crc16 = (uint16) (crctab[(crc16 >> 8) ^ buf[start + i]] ^ (crc16 << 8));
          }

          return crc16;
        }
    };

    class ProtocolBase
    {
      public:
        /// @brief BigEndian-1 LittleEndian-2;
        static uint8 ByteOrder;
        static const uint16 FRAME_MIN_LENGTH = 28;
        ByteVectorPtr Buffer;

        ByteVector Head       = {0xFF,0XAA};
        ByteVector Length     = {0x00,0x00};
        ByteVector Idx        = {0x00,0x00};
        ByteVector Timestamp  = {0x00,0x00,0x00,0x00};
        uint8      CheckType  = 0x01;
        uint8      Type       = 0x00;
        ByteVector DeviceType = {0x00,0x00};
        ByteVector Remain     = {0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00};
        uint8      MainCmd    = 0x00;
        uint8      SubCmd     = 0x00;
        ByteVector CmdParams  = {0x00,0x00};
        ByteVector Content;
        ByteVector Check      = {0x00,0x00};
        ByteVector Tail       = {0xEE,0xEE};

      public:
        ProtocolBase(uint16 idx,uint32 timestamp,uint8 checkType,uint8 type,const ByteVector &deviceType,const ByteVector &remain,
                      uint8 mainCmd,uint8 subCmd,const ByteVector &cmdParams,const ByteVector &content)  
        {
            if(ByteOrder == 1)
            {
              Idx[0] = (idx >> 8) & 0xFF;
              Idx[1] = (idx >> 0) & 0xFF;

              Timestamp[0] = (timestamp >> 24) & 0xFF;
              Timestamp[1] = (timestamp >> 16) & 0xFF;
              Timestamp[2] = (timestamp >> 8) & 0xFF;
              Timestamp[3] = (timestamp >> 0) & 0xFF;
            }
            else
            {
              Idx[0] = (idx >> 0) & 0xFF;
              Idx[1] = (idx >> 8) & 0xFF;

              Timestamp[0] = (timestamp >> 0) & 0xFF;
              Timestamp[1] = (timestamp >> 8) & 0xFF;
              Timestamp[2] = (timestamp >> 16) & 0xFF;
              Timestamp[3] = (timestamp >> 24) & 0xFF;
            }

            
            CheckType = checkType;
            Type = type;
            DeviceType = deviceType;
            Remain = remain;
            MainCmd = mainCmd;
            SubCmd = subCmd;
            CmdParams = cmdParams;
            Content = content;
        }

        ProtocolBase(const ByteVector &idx,const ByteVector &timestamp,uint8 checkType,uint8 type,ByteVector &deviceType,
                      ByteVector &remain,uint8 mainCmd,uint8 subCmd,const ByteVector &cmdParams,const ByteVector &content)
        {
          Idx = idx;
          Timestamp = timestamp;
          CheckType = checkType;
          Type = type;
          DeviceType = deviceType;
          Remain = remain;
          MainCmd = mainCmd;
          SubCmd = subCmd;
          CmdParams = cmdParams;
          Content = content;
        }

        ProtocolBase()
        {
          
        }

        bool Parse(ByteVector& buf)
        {
          Buffer = std::make_shared<ByteVector>(buf);
          if(Buffer->size() < FRAME_MIN_LENGTH)
            return false;

          if(!(Head[0] == (*Buffer)[0] && Head[1] == (*Buffer)[1] && Tail[0] == (*Buffer)[Buffer->size()-2] &&
            Tail[1] == (*Buffer)[Buffer->size()-1]))
            return false;
          int idx = 2;

          auto itr = Buffer->begin()+idx;
          std::copy(itr,itr+Length.size(),Length.begin());
          itr = itr+Length.size();
          std::copy(itr,itr+Idx.size(),Idx.begin());
          itr = itr+Idx.size();
          std::copy(itr,itr+Timestamp.size(),Timestamp.begin());
          itr = itr+Timestamp.size();
          CheckType = *itr++;
          Type = *itr++;
          std::copy(itr,itr+DeviceType.size(),DeviceType.begin());
          itr = itr+DeviceType.size();
          std::copy(itr,itr+Remain.size(),Remain.begin());
          itr = itr+Remain.size();
          MainCmd = *itr++;
          SubCmd = *itr++;

          std::copy(itr,itr+CmdParams.size(),CmdParams.begin());
          itr = itr+CmdParams.size();
          Content.clear();
          Content.shrink_to_fit();
          Content.assign(itr,Buffer->end()-4);
          itr = itr+Content.size();

          std::copy(itr,itr+Check.size(),Check.begin());
          itr = itr+Check.size();

          if(CheckType == CheckClass::No_Check)
            return true;

          if(CheckType == CheckClass::Xor_Check)
          {
            if(CheckClass::Xor(buf,2,buf.size()-6) == (uint16)((Check[0]<<8)+Check[1]))
            {
              return true;
            }
            else
            {
              return false;
            }
          }

          if(CheckType == CheckClass::Crc16_Check)
          {
            if(CheckClass::Crc16(buf,2,buf.size()-6) == (uint16)((Check[0]<<8)+Check[1]))
            {
              return true;
            }
            else
            {
              return false;
            }
          }

          return false;
        }

        ByteVectorPtr GetBytes()
        {
          int len = Length.size() + Idx.size() + Timestamp.size() + 1 + 1 + DeviceType.size() + Remain.size() + 1 + 1 +
            CmdParams.size() + Content.size() + Check.size();

          if(ByteOrder == 1)
          {
            Length[0] = (uint8)((len >> 8) & 0xFF);
            Length[1] = (uint8)(len & 0xFF);
          }
          else
          {
            Length[0] = (uint8)(len & 0xFF);
            Length[1] = (uint8)((len >> 8) & 0xFF);
          }

          ByteVectorPtr buf = std::make_shared<std::vector<uint8>>();
          buf->insert(buf->end(),Head.begin(),Head.end());
          buf->insert(buf->end(),Length.begin(),Length.end());
          buf->insert(buf->end(),Idx.begin(),Idx.end());
          buf->insert(buf->end(),Timestamp.begin(),Timestamp.end());
          buf->emplace_back(CheckType);
          buf->emplace_back(Type);
          buf->insert(buf->end(),DeviceType.begin(),DeviceType.end());
          buf->insert(buf->end(),Remain.begin(),Remain.end());
          buf->emplace_back(MainCmd);
          buf->emplace_back(SubCmd);
          buf->insert(buf->end(),CmdParams.begin(),CmdParams.end());
          buf->insert(buf->end(),Content.begin(),Content.end());
          if(CheckType == (uint8)CheckClass::Xor_Check)
          {
            Check[0] = 0;
            Check[1] = CheckClass::Xor(*buf,2,buf->size()-2);
          }
          else if(CheckType == (uint8)CheckClass::Crc16_Check)
          {
            Check[0] = 0;
            Check[1] = CheckClass::Crc16(*buf,2,buf->size()-2);
          }

          buf->insert(buf->end(),Check.begin(),Check.end());
          buf->insert(buf->end(),Tail.begin(),Tail.end());

          return buf;
        }

        std::shared_ptr<ProtocolBase> CreateNew()
        {
          std::shared_ptr<ProtocolBase> pb(new ProtocolBase());

          pb->Head = Head;
          pb->Length = Length;
          pb->Idx = Idx;
          pb->Timestamp = Timestamp;
          pb->DeviceType = DeviceType;
          pb->Remain = Remain;
          pb->CmdParams = CmdParams;
          pb->Check = Check;
          pb->Tail = Tail;

          return pb;
        }
    };

    uint8 ProtocolBase::ByteOrder = 1;
    
    // class ProtocolBase
    // {
    //   public:
    //     /// @brief BigEndian-1  LittleEndian-2
    //     static const int32 LowHighMode = 1; 
    //     static const int32 FRAME_MIN_LENGTH = 28;  
    //     std::vector<uint8> Buffer;
    //     std::array<uint8,2> Head = {{0xFF,0xAA}};
    //     std::array<uint8,2> Length = {{0x00,0x00}};
    //     std::array<uint8,2> Idx = {{0x00,0x00}};
    //     std::array<uint8,4> Timestamp = {{0x00,0x00,0x00,0x00}};
    //     uint8 CheckType = 0x00;
    //     uint8 Type = 0x00;
    //     std::array<uint8,2> DeviceType = {{0x00,0x00}};
    //     std::array<uint8,8> Remain = {{0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00}};
    //     uint8 MainCmd = 0x00;
    //     uint8 SubCmd = 0x00;
    //     std::array<uint8,2> CmdParams = {{0x00,0x00}};
    //     std::vector<uint8> Content;
    //     std::array<uint8,2> Check = {{0x00,0x00}};
    //     std::array<uint8,2> Tail = {{0xEE,0xEE}};

    //     public:
    //       ProtocolBase(uint16 idx,uint32 timestamp,uint8 checkType,uint8 type,std::array<uint8,2>& deviceType,std::array<uint8,8>& remain,
    //         uint8 mainCmd,uint8 subCmd,std::array<uint8,2>& cmdParams,std::vector<uint8>& content)
    //       {
    //         if(LowHighMode == 1)
    //         {
    //           Idx[0] = (idx >> 8) & 0xFF;
    //           Idx[1] = (idx >> 0) & 0xFF;

    //           Timestamp[0] = (timestamp >> 24) & 0xFF;
    //           Timestamp[1] = (timestamp >> 16) & 0xFF;
    //           Timestamp[2] = (timestamp >> 8) & 0xFF;
    //           Timestamp[3] = (timestamp >> 0) & 0xFF;
    //         }
    //         else
    //         {
    //           Idx[0] = (idx >> 0) & 0xFF;
    //           Idx[1] = (idx >> 8) & 0xFF;

    //           Timestamp[0] = (timestamp >> 0) & 0xFF;
    //           Timestamp[1] = (timestamp >> 8) & 0xFF;
    //           Timestamp[2] = (timestamp >> 16) & 0xFF;
    //           Timestamp[3] = (timestamp >> 24) & 0xFF;
    //         }

            
    //         CheckType = checkType;
    //         Type = type;
    //         DeviceType = deviceType;
    //         Remain = remain;
    //         MainCmd = mainCmd;
    //         SubCmd = subCmd;
    //         CmdParams = cmdParams;
    //         Content = content;
    //       }

    //       ProtocolBase(const std::array<uint8,2>& idx,const std::array<uint8,4>& timestamp,uint8 checkType,uint8 type,std::array<uint8,2>& deviceType,std::array<uint8,8>& remain,
    //         uint8 mainCmd,uint8 subCmd,std::array<uint8,2>& cmdParams,std::vector<uint8>& content)
    //       {
    //         Idx = idx;
    //         Timestamp = timestamp;
    //         CheckType = checkType;
    //         Type = type;
    //         DeviceType = deviceType;
    //         Remain = remain;
    //         MainCmd = mainCmd;
    //         SubCmd = subCmd;
    //         CmdParams = cmdParams;
    //         Content = content;
    //       }

    //       ProtocolBase()
    //       {}

    //       bool Parse(std::vector<uint8>& buf)
    //       {
    //         Buffer.clear();
    //         Buffer.shrink_to_fit();
    //         Buffer.assign(buf.begin(),buf.end());
    //         if(Buffer.size() < FRAME_MIN_LENGTH)
    //           return false;

    //         if(!(Head[0] == Buffer[0] && Head[1] == Buffer[1] && Tail[0] == Buffer[Buffer.size()-2] &&
    //           Tail[1] == Buffer[Buffer.size()-1]))
    //           return false;
    //         int idx = 2;

    //         auto itr = Buffer.begin()+idx;
    //         std::copy(itr,itr+Length.size(),Length.begin());
    //         itr = itr+Length.size();
    //         std::copy(itr,itr+Idx.size(),Idx.begin());
    //         itr = itr+Idx.size();
    //         std::copy(itr,itr+Timestamp.size(),Timestamp.begin());
    //         itr = itr+Timestamp.size();
    //         CheckType = *itr++;
    //         Type = *itr++;
    //         std::copy(itr,itr+DeviceType.size(),DeviceType.begin());
    //         itr = itr+DeviceType.size();
    //         std::copy(itr,itr+Remain.size(),Remain.begin());
    //         itr = itr+Remain.size();
    //         MainCmd = *itr++;
    //         SubCmd = *itr++;

    //         std::copy(itr,itr+CmdParams.size(),CmdParams.begin());
    //         itr = itr+CmdParams.size();
    //         Content.clear();
    //         Content.shrink_to_fit();
    //         Content.assign(itr,Buffer.end()-4);
    //         itr = itr+Content.size();

    //         std::copy(itr,itr+Check.size(),Check.begin());
    //         itr = itr+Check.size();

    //         if(CheckType == CheckClass::No_Check)
    //           return true;

    //         if(CheckType == CheckClass::Xor_Check)
    //         {
    //           if(CheckClass::Xor(buf,2,buf.size()-6) == (uint16)((Check[0]<<8)+Check[1]))
    //           {
    //             return true;
    //           }
    //           else
    //           {
    //             return false;
    //           }
    //         }

    //         if(CheckType == CheckClass::Crc16_Check)
    //         {
    //           if(CheckClass::Crc16(buf,2,buf.size()-6) == (uint16)((Check[0]<<8)+Check[1]))
    //           {
    //             return true;
    //           }
    //           else
    //           {
    //             return false;
    //           }
    //         }
    //       }

    //       std::shared_ptr<std::vector<uint8>> GetBytes()
    //       {
    //         int len = Length.size() + Idx.size() + Timestamp.size() + 1 + 1 + DeviceType.size() + Remain.size() + 1 + 1 +
    //           CmdParams.size() + Content.size() + Check.size();

    //         if(LowHighMode == 1)
    //         {
    //           Length[0] = (uint8)((len >> 8) & 0xFF);
    //           Length[1] = (uint8)(len & 0xFF);
    //         }
    //         else
    //         {
    //           Length[0] = (uint8)(len & 0xFF);
    //           Length[1] = (uint8)((len >> 8) & 0xFF);
    //         }

    //         std::shared_ptr<std::vector<uint8>> buf = std::make_shared<std::vector<uint8>>();
    //         buf->insert(buf->end(),Head.begin(),Head.end());
    //         buf->insert(buf->end(),Length.begin(),Length.end());
    //         buf->insert(buf->end(),Idx.begin(),Idx.end());
    //         buf->insert(buf->end(),Timestamp.begin(),Timestamp.end());
    //         buf->emplace_back(CheckType);
    //         buf->emplace_back(Type);
    //         buf->insert(buf->end(),DeviceType.begin(),DeviceType.end());
    //         buf->insert(buf->end(),Remain.begin(),Remain.end());
    //         buf->emplace_back(MainCmd);
    //         buf->emplace_back(SubCmd);
    //         buf->insert(buf->end(),CmdParams.begin(),CmdParams.end());
    //         buf->insert(buf->end(),Content.begin(),Content.end());
    //         if(CheckType == (uint8)CheckClass::Xor_Check)
    //         {
    //           Check[0] = 0;
    //           Check[1] = CheckClass::Xor(*buf,2,buf->size()-2);
    //         }
    //         else if(CheckType == (uint8)CheckClass::Crc16_Check)
    //         {
    //           Check[0] = 0;
    //           Check[1] = CheckClass::Crc16(*buf,2,buf->size()-2);
    //         }

    //         buf->insert(buf->end(),Check.begin(),Check.end());
    //         buf->insert(buf->end(),Tail.begin(),Tail.end());

    //         return buf;
    //       }

    //       std::shared_ptr<ProtocolBase> CreateNew()
    //       {
    //         std::shared_ptr<ProtocolBase> pb(new ProtocolBase());

    //         Head = pb->Head;
    //         Length = pb->Length;
    //         Idx = pb->Idx;
    //         Timestamp = pb->Timestamp;
    //         DeviceType = pb->DeviceType;
    //         Remain = pb->Remain;
    //         CmdParams = pb->CmdParams;
    //         Check = pb->Check;
    //         Tail = pb->Tail;
    //         return pb;
    //       }

    // };
  }
}