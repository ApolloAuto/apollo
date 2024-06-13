
#pragma once
#include <vanjeelidar/driver/input/input.hpp>
#include <vanjeelidar/driver/input/input_socket.hpp>

#ifndef DISABLE_PCAP_PARSE
#include <vanjeelidar/driver/input/input_pcap.hpp>
#endif

#include <vanjeelidar/driver/input/input_raw.hpp>

namespace vanjee
{
namespace lidar
{
        class InputFactory
        {

        public:
            static std::shared_ptr<Input> createInput(InputType type, const WJInputParam &param,
                                                      double sec_to_delay, std::function<void(const uint8_t*, size_t)>& cb_feed_pkt);
        };

        inline std::shared_ptr<Input> InputFactory::createInput(InputType type, const WJInputParam &param,
                                                                double sec_to_delay, std::function<void(const uint8_t*, size_t)>& cb_feed_pkt)
        {
            std::shared_ptr<Input> input;
            switch (type)
            {
            case InputType::ONLINE_LIDAR:
            {
                if(param.connect_type == 2)
                    input = std::make_shared<InputTcpSocket>(param);
                else
                    input = std::make_shared<InputUdpSocket>(param);
            }
            break;
            
#ifndef DISABLE_PCAP_PARSE
            case InputType::PCAP_FILE:
            {
                input = std::make_shared<InputPcap>(param, sec_to_delay);
            }
            break;

#endif
            case InputType::RAW_PACKET:
            {
                std::shared_ptr<InputRaw> inputRaw;
                inputRaw = std::make_shared<InputRaw>(param);

                cb_feed_pkt = std::bind(&InputRaw::feedPacket, inputRaw, 
                    std::placeholders::_1, std::placeholders::_2);

                input = inputRaw;
            }
            break;

            default:
                WJ_ERROR << "Wrong Input Type " << type << "." << WJ_REND;
                if (type == InputType::PCAP_FILE)
                {
                    WJ_ERROR << "To use InputType::PCAP_FILE, please do not specify the make option DISABLE_PCAP_PARSE." << WJ_REND;
                }
                exit(-1);
            }
            return input;       
        }
} 
} 
