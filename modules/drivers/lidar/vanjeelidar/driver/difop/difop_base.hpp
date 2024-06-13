#pragma once
#include <memory>
#include <functional>
#include <map>
#include <thread>
#include <vector>
#include <algorithm>


#include "vanjeelidar/common/super_header.hpp"
#include <vanjeelidar/utility/sync_queue.hpp>
#include "protocol_base.hpp"

namespace vanjee
{
  namespace lidar
  {
    class BufInfo
    {
      public:
        std::string Ip;
        std::shared_ptr<std::vector<uint8>> Buf;
      public:
        BufInfo(std::string ip,std::shared_ptr<std::vector<uint8>> buf)
        {
          Ip = ip;
          Buf = buf;
        }

        BufInfo(){}

        void setData(std::string ip,std::shared_ptr<std::vector<uint8>> buf)
        {
          Ip = ip;
          Buf = buf;
        }

    };

    class GetDifoCtrlClass
    {
      private:
        std::vector<uint8> byteStream_protocol_pkt;
        bool stopFlag_send_protocol_pkt;
        /// @brief unit:ms
        uint32 interval_send_protocol_pkt;

      public:
        GetDifoCtrlClass(std::vector<uint8> &pkt,bool stopFlag=false,uint32 interval=1000):
                      byteStream_protocol_pkt(pkt),stopFlag_send_protocol_pkt(stopFlag),
                      interval_send_protocol_pkt(interval)
        {

        }

        GetDifoCtrlClass()
        {

        }

        void setStopFlag(bool stopFlag)
        {
          stopFlag_send_protocol_pkt = stopFlag;
        }

        bool getStopFlag()
        {
          return stopFlag_send_protocol_pkt;
        }

        uint32 getSendInterval()
        {
          return interval_send_protocol_pkt;
        }

        std::vector<uint8> getSendBuf()
        {
          return byteStream_protocol_pkt;
        }
    };

    class DifopBase
    {
      public:
        typedef std::function<int32(uint8*,uint32)> udpSendCallback;
        typedef std::function<void(std::shared_ptr<ProtocolBase> protocol)> frameCallback;
        typedef std::function<bool()> deviceInfoIsReadyCallback;

        DifopBase();
        bool init();
        bool start(uint16_t dataParsingType);
        bool stop();
        void udpSendData();
        void loopParsingProcess();
        void singleParsingProcess();
        bool processDifoPktFlag();
        void setOrgProtocolBase(ProtocolBase &protocolBase);
        void dataEnqueue(std::string ip,std::shared_ptr<std::vector<uint8>> buf);
        bool regCallback(udpSendCallback udpsend,frameCallback frameCb);
        
        virtual void initGetDifoCtrlDataMapPtr() = 0;

      private:
        SyncQueue<std::shared_ptr<BufInfo>> BufInfo_Queue_;
        std::map<std::string,std::shared_ptr<std::vector<uint8>>> BufVectorCaches;
        ProtocolBase OrgProtocol;
        udpSendCallback udpSend_;
        frameCallback frameCb_;
        bool init_flag_;
        bool start_flag_;
        bool to_exit_recv_;
        bool process_difoPkt_flag_;
        std::thread send_thread_;
        std::thread handle_thread_;
      public:
        std::shared_ptr<std::map<uint16,GetDifoCtrlClass>> getDifoCtrlData_map_ptr_; 
    };

    DifopBase::DifopBase():init_flag_(false),start_flag_(false),to_exit_recv_(true),process_difoPkt_flag_(true)
    {

    }

    bool DifopBase::init()
    {
      if(init_flag_)
      {
        return false;
      }
      
      init_flag_ = true;

      return true;
    }

    bool DifopBase::start(uint16_t dataParsingType)
    {
      if(start_flag_)
      {
        return false;
      }

      to_exit_recv_ = false;
      send_thread_ = std::thread(std::bind(&DifopBase::udpSendData,this));
      if(dataParsingType == 2)
        handle_thread_ = std::thread(std::bind(&DifopBase::loopParsingProcess,this));
      else
        handle_thread_ = std::thread(std::bind(&DifopBase::singleParsingProcess,this));
      start_flag_ = true;
      
      return true;
    }

    bool DifopBase::stop()
    {
      if(!start_flag_)
      {
        return false;
      }

      to_exit_recv_ = true;
      
      send_thread_.join();
      handle_thread_.join();

      start_flag_ = false;
      return true;
    }

    bool DifopBase::regCallback(udpSendCallback udpSend,frameCallback frameCb)
    {
      udpSend_ = udpSend;
      frameCb_ = frameCb;

      return true;
    }

    void DifopBase::setOrgProtocolBase(ProtocolBase &protocolBase)
    {
      OrgProtocol = protocolBase;
    }

    void DifopBase::udpSendData()
    {
      uint64 duration = 0;
      uint8  getDifo_cnt = 0;
      while(!to_exit_recv_)
      {
        duration += 1000;
        if(getDifoCtrlData_map_ptr_)
        {
          for(const auto &iter : *getDifoCtrlData_map_ptr_)
          {
            GetDifoCtrlClass GetDifoCtrlData = iter.second;

            if(!GetDifoCtrlData.getStopFlag())
            {
              if(duration % GetDifoCtrlData.getSendInterval() == 0)
              {
                udpSend_(GetDifoCtrlData.getSendBuf().data(),GetDifoCtrlData.getSendBuf().size());
              }
              getDifo_cnt++;
            }
          }

          if(getDifo_cnt == 0)
          {
            process_difoPkt_flag_ = false;
          }
          else
          {
            process_difoPkt_flag_ = true;
          }
          getDifo_cnt = 0;
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(1000));
      }
    }

    void DifopBase::dataEnqueue(std::string ip,std::shared_ptr<std::vector<uint8>> buf)
    {
      std::shared_ptr<BufInfo> bufInfo = std::make_shared<BufInfo>();
      bufInfo->setData(ip,buf);
      BufInfo_Queue_.push(bufInfo);
    }

    bool DifopBase::processDifoPktFlag()
    {
      return process_difoPkt_flag_;
    }

    void DifopBase::loopParsingProcess()
    {
      std::shared_ptr<BufInfo> bufInfo;
      std::shared_ptr<std::vector<uint8>> bufCache;
      while(!to_exit_recv_)
      {
        bufInfo = BufInfo_Queue_.popWait(100000);
        if(bufInfo.get() == NULL)
          continue;

        if(BufVectorCaches.find(bufInfo->Ip) == BufVectorCaches.end())
        {
          BufVectorCaches.insert(std::pair<std::string,std::shared_ptr<std::vector<uint8>>>(bufInfo->Ip,
            std::make_shared<std::vector<uint8>>()));
        }

        bufCache = BufVectorCaches[bufInfo->Ip];
        std::vector<uint8> data;
        if(bufCache->size() > 0)
        {
          std::copy(bufCache->begin(),bufCache->end(),std::back_inserter(data));
          std::copy(bufInfo->Buf->begin(),bufInfo->Buf->end(),std::back_inserter(data));
        }
        else
        {
          std::copy(bufInfo->Buf->begin(),bufInfo->Buf->end(),std::back_inserter(data));
        }

        bufCache->clear();
        bufCache->shrink_to_fit();

        uint32 indexLast = 0;
        std::shared_ptr<std::vector<std::vector<uint8>>> frames = std::make_shared<std::vector<std::vector<uint8>>>();
        for(uint32 i = 0;i<data.size();i++)
        {
          if(data.size() - i < ProtocolBase::FRAME_MIN_LENGTH)
            break;

          if(!(data[i] == 0xFF && data[i + 1] == 0xAA))
          {
            indexLast = i + 1;
            continue;
          }
 
          uint32 frameLen = 0;
          if (ProtocolBase::ByteOrder == 1)
            frameLen = (data[i + 2] << 8) + data[i + 3] + 4;
          else
            frameLen = data[i + 2] + (data[i + 3] << 8) + 4;
          
          if(i + frameLen > data.size())
            break;

          if(!(data[i + frameLen - 2] == 0xEE && data[i + frameLen - 1] == 0xEE))
          {
            indexLast = i + 1;
            continue;
          }

          std::vector<uint8> tmp(data.begin()+i,data.begin()+i+frameLen);

          // for(int j = 0; j < tmp.size(); j++)
          //   std::cout << std::hex << (int)tmp[j] << " "; 
          // std::cout << std::endl;

          frames->emplace_back(tmp);
          i += frameLen - 1;
          indexLast = i + 1;
        }

        if(indexLast < data.size())
        {
          bufCache->assign(data.begin()+indexLast,data.end());
        }
        
        for(auto item : *frames)
        {
          auto protocol = OrgProtocol.CreateNew();
          if(protocol->Parse(item))
          {
            frameCb_(protocol);
          }
        }

      }
    }

    void DifopBase::singleParsingProcess()
    {
      std::shared_ptr<BufInfo> bufInfo;
      while(!to_exit_recv_)
      {
        bufInfo = BufInfo_Queue_.popWait(100000);
        if(bufInfo.get() == NULL)
          continue;
        std::vector<uint8>& data = *(bufInfo->Buf);

        if(data.size() < ProtocolBase::FRAME_MIN_LENGTH)
          continue;

        if(!(data[0] == 0xFF && data[1] == 0xAA))
          continue;

        uint32 frameLen = 0;
        if (ProtocolBase::ByteOrder == 1)
          frameLen = (data[2] << 8) + data[3] + 4;
        else
          frameLen = data[2] + (data[3] << 8) + 4;
        
        if(frameLen > data.size())
          continue;

        if(!(data[frameLen - 2] == 0xEE && data[frameLen - 1] == 0xEE))
          continue;

        // for(int j = 0; j < data.size(); j++)
        //   std::cout << std::hex << (int)data[j] << " "; 
        // std::cout << std::endl;

        auto protocol = OrgProtocol.CreateNew();
        if(protocol->Parse(data))
        {
          frameCb_(protocol);
        }
                  
      }
    }
  }
}