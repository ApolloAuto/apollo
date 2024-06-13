
#pragma once

#include <vanjeelidar/driver/input/input.hpp>
#include <pcap.h>
#include <sstream>

namespace vanjee
{
namespace lidar
{
        class InputPcap : public Input
        {

        public:
            InputPcap(const WJInputParam &input_param, double sec_to_delay);
            virtual bool init();
            virtual bool start();
            virtual ~InputPcap();

        private:
            void recvPacket();

        private:
            pcap_t *pcap_;                 
            size_t pcap_offset_;           
            size_t pcap_tail_;             
            std::string msop_filter_str_;  
            std::string difop_filter_str_; 
            bpf_program msop_filter_;      
            bpf_program difop_filter_;     
            bool difop_filter_valid_;      
            uint64_t msec_to_delay;        
        };

        InputPcap::InputPcap(const WJInputParam &input_param, double sec_to_delay)
            : Input(input_param), pcap_(NULL), pcap_offset_(input_param.connect_type == 2 ? TCP_ETH_HDR_LEN : UDP_ETH_HDR_LEN), pcap_tail_(0), difop_filter_valid_(false), 
            msec_to_delay((uint64_t)(sec_to_delay / input_param.pcap_rate * 1000000))
        {
            if (input_param.use_vlan)
            {
                pcap_offset_ += VLAN_HDR_LEN;
            }
            pcap_offset_ += input_param.user_layer_bytes;
            pcap_tail_ += input_param.tail_layer_bytes;

            std::stringstream msop_stream, difop_stream;

            if (input_param.use_vlan)
            {
                msop_stream << "vlan &&";
                difop_stream << "vlan &&";
            }

            if(input_param.connect_type == 2)
            {
                msop_stream << "tcp dst port " << input_param.host_msop_port;
                difop_stream << "tcp dst port " << input_param.difop_port;   
            }
            else
            {
                msop_stream << "udp dst port " << input_param.host_msop_port;
                difop_stream << "udp dst port " << input_param.difop_port;   
            }

            msop_filter_str_ = msop_stream.str();
            difop_filter_str_ = difop_stream.str();
        }

        inline bool InputPcap::init()
        {
            if (init_flag_)
            {
                return true;
            }
            char errbuf[PCAP_ERRBUF_SIZE];
            pcap_ = pcap_open_offline(input_param_.pcap_path.c_str(), errbuf);
            if (pcap_ == NULL)
            {
                cb_excep_(Error(ERRCODE_PCAPWRONGPATH));
                return false;
            }
            pcap_compile(pcap_, &msop_filter_, msop_filter_str_.c_str(), 1, 0xFFFFFFFF);
            if ((input_param_.difop_port != 0) && (input_param_.difop_port != input_param_.host_msop_port))
            {
                pcap_compile(pcap_, &difop_filter_, difop_filter_str_.c_str(), 1, 0xFFFFFFFF);
                difop_filter_valid_ = true;
            }
            init_flag_ = true;
            return true;
        }

        inline bool InputPcap::start()
        {
            if (start_flag_)
            {
                return true;
            }
            if (!init_flag_)
            {
                cb_excep_(Error(ERRCODE_STARTBEFOREINIT));
                return false;
            }

            to_exit_recv_ = false;
            recv_thread_ = std::thread(std::bind(&InputPcap::recvPacket, this));
            start_flag_ = true;
            return true;
        }

        inline InputPcap::~InputPcap()
        {
            stop();
            if (pcap_ != NULL)
            {
                pcap_close(pcap_);
                pcap_ = NULL;
            }
        }

        inline void InputPcap::recvPacket()
        {
            while (!to_exit_recv_)
            {
                struct pcap_pkthdr *header;
                const u_char *pkt_data;
                int net = pcap_next_ex(pcap_, &header, &pkt_data);
                if (net < 0) 
                {
                    pcap_close(pcap_);
                    pcap_ = NULL;
                    if (input_param_.pcap_repeat)
                    {
                        cb_excep_(Error(ERRCODE_PCAPREPAT));
                        char errbuf[PCAP_ERRBUF_SIZE];
                        pcap_ = pcap_open_offline(input_param_.pcap_path.c_str(), errbuf);
                        continue;
                    }
                    else
                    {
                        cb_excep_(Error(ERRCODE_PCAPEXIT));
                        break;
                    }
                }
                if (pcap_offline_filter(&msop_filter_, header, pkt_data) != 0)
                {
                    /// @brief 获取缓存长度为ETH_LEN
                    std::shared_ptr<Buffer> pkt = cb_get_pkt_(input_param_.connect_type == 2 ? TCP_ETH_LEN : UDP_ETH_LEN);
                    /// @brief 调用memcpy将数据复制到缓存中，并调用Buffer::setData()设置pkt的长度
                    memcpy(pkt->data(), pkt_data + pcap_offset_, header->len - pcap_offset_ - pcap_tail_);
                    pkt->setData(0, header->len - pcap_offset_ - pcap_tail_);
                    pushPacket(pkt);
                }
                else if (difop_filter_valid_ && (pcap_offline_filter(&msop_filter_, header, pkt_data) != 0))
                {
                    std::shared_ptr<Buffer> pkt = cb_get_pkt_(input_param_.connect_type == 2 ? TCP_ETH_LEN : UDP_ETH_LEN);
                    memcpy(pkt->data(), pkt_data + pcap_offset_, header->len - pcap_offset_ - pcap_tail_);
                    pkt->setData(0, header->len - pcap_offset_ - pcap_tail_);
                    pushPacket(pkt);
                }
                else
                {
                    continue;
                }
                std::this_thread::sleep_for(std::chrono::microseconds(msec_to_delay));
            }
        }

    } 
} 
