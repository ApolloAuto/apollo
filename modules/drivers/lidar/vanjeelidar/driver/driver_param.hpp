
#pragma once

#include <vanjeelidar/common/wj_log.hpp>
#include <string>
#include <map>
#include <vector>

namespace vanjee
{
namespace lidar
{
    enum LidarType
    {
      vanjee_720,
      vanjee_720_16,
      vanjee_720_32
    };
    inline std::string lidarTypeToStr(const LidarType &type)
    {
      std::string str = "";
      switch (type)
      {
      case LidarType::vanjee_720:
      case LidarType::vanjee_720_16:
        str = "vanjee_720_16";
        break;
      case LidarType::vanjee_720_32:
        str = "vanjee_720_32";
        break;
      
      default:
        str = "ERROR";
        WJ_ERROR << "WJ_ERROR" << WJ_REND;
        break;
      }
      return str;
    }
    inline LidarType strToLidarType(const std::string &type)
    {
      else if (type == "vanjee_720" || type == "vanjee_720_16")
      {
        return LidarType::vanjee_720_16;
      }
      else if (type == "vanjee_720_32")
      {
        return LidarType::vanjee_720_32;
      }

      else
      {
        WJ_ERROR << "Wrong lidar type: " << type << WJ_REND;
        exit(-1);
      }
    }
    enum InputType
    {
      ONLINE_LIDAR = 1,
      PCAP_FILE = 2,
      RAW_PACKET

    };
    inline std::string inputTypeToStr(const InputType &type)
    {
      std::string str = "";
      switch (type)
      {
      case InputType::ONLINE_LIDAR:
        str = "ONLINE_LIDAR";
        break;
      case InputType::PCAP_FILE:
        str = "PCAP_FILE";
        break;

      default:
        str = "ERROR";
        WJ_ERROR << "WJ_ERROR" << WJ_REND;
        break;
      }
      return str;
    }

    struct WJTransfromParam 
    {
      float x = 0.0f;     
      float y = 0.0f;     
      float z = 0.0f;     
      float roll = 0.0f;  
      float pitch = 0.0f; 
      float yaw = 0.0f;   
      void print() const
      {
        WJ_INFO << "------------------------------------------------------" << WJ_REND;
        WJ_INFO << "          VanjeeLidar Transform Parameters            " << WJ_REND;
        WJ_INFOL << "x: " << x << WJ_REND;
        WJ_INFOL << "y: " << y << WJ_REND;
        WJ_INFOL << "z: " << z << WJ_REND;
        WJ_INFOL << "roll: " << roll << WJ_REND;
        WJ_INFOL << "pitch: " << pitch << WJ_REND;
        WJ_INFOL << "yaw: " << yaw << WJ_REND;
        WJ_INFO << "------------------------------------------------------" << WJ_REND;
      }
    };
    struct WJDecoderParam
    {
      bool config_from_file = true;                                    
      bool wait_for_difop = true;                                       
      float min_distance = 0.0f;                                        
      float max_distance = 0.0f;                                        
      float start_angle = 0.0f;                                         
      float end_angle = 360.0f;                                                                      
      bool use_lidar_clock = false;                                     
      bool dense_points = false;
      bool ts_first_point = false;  
      uint16_t publish_mode = 2; 
      uint16_t rpm = 0;
      std::string angle_path_ver = "";                                      
      std::string angle_path_hor = ""; 
      std::string imu_param_path = "";                                     
      WJTransfromParam transform_param;                                 
      void print() const
      {
        WJ_INFO << "------------------------------------------------" << WJ_REND;
        WJ_INFO << "----------VANJEE Decoder Parameters-------------" << WJ_REND;
        WJ_INFOL << "wait_for_difop: " << wait_for_difop << WJ_REND;
        WJ_INFOL << "min_distance: " << min_distance << WJ_REND;
        WJ_INFOL << "max_distance: " << max_distance << WJ_REND;
        WJ_INFOL << "start_angle: " << start_angle << WJ_REND;
        WJ_INFOL << "end_angle: " << end_angle << WJ_REND;
        WJ_INFOL << "use_lidar_clock: " << use_lidar_clock << WJ_REND;
        WJ_INFOL << "dense_point: " << dense_points << WJ_REND;
        WJ_INFOL << "config_from_file: " << config_from_file << WJ_REND;
        WJ_INFOL << "angle_path_ver: " << angle_path_ver << WJ_REND;
        WJ_INFOL << "angle_path_hor: " << angle_path_hor << WJ_REND;
        WJ_INFOL << "imu_param_path: " << imu_param_path << WJ_REND;
        WJ_INFOL << "publish_mode: " << publish_mode << WJ_REND;
        WJ_INFO << "------------------------------------------------" << WJ_REND;
        transform_param.print();
      }
    };
    struct WJInputParam 
    {
      uint16_t connect_type = 1;
      uint16_t host_msop_port = 3333;             
      uint16_t lidar_msop_port = 3001;
      uint16_t difop_port = 0;            
      std::string host_address = "0.0.0.0";  
      std::string lidar_address = "0.0.0.0"; 
      std::string group_address = "0.0.0.0"; 
      std::string pcap_path = "";            
      bool pcap_repeat = true;               
      float pcap_rate = 1.0f;                
      bool use_vlan = false;                 
      uint16_t user_layer_bytes = 0;         
      uint16_t tail_layer_bytes = 0;         
      void print() const
      {
        WJ_INFO << "-----------------------------------------------------------" << WJ_REND;
        WJ_INFO << "             VANJEE Input Parameters                     " << WJ_REND;
        WJ_INFO << "connect_type: " << (connect_type == 2 ? "tcp" : "udp") << WJ_REND;
        WJ_INFOL << "host_msop_port: " << host_msop_port << WJ_REND;
        WJ_INFOL << "lidar_msop_port: " << lidar_msop_port << WJ_REND; 
        WJ_INFOL << "host_address: " << host_address << WJ_REND;
        WJ_INFOL << "lidar_address: "<< lidar_address <<WJ_REND;
        WJ_INFOL << "group_address: " << group_address << WJ_REND;
        WJ_INFOL << "pcap_path: " << pcap_path << WJ_REND;
        WJ_INFOL << "pcap_repeat: " << pcap_repeat << WJ_REND;
        WJ_INFOL << "pcap_rate: " << pcap_rate << WJ_REND;
        WJ_INFOL << "use_vlan: " << use_vlan << WJ_REND;
        WJ_INFOL << "user_layer_bytes: " << user_layer_bytes << WJ_REND;
        WJ_INFOL << "tail_layer_bytes: " << tail_layer_bytes << WJ_REND;
        WJ_INFO << "-----------------------------------------------------------" << WJ_REND;
      }
    };
    struct WJDriverParam
    {
      LidarType lidar_type = LidarType::vanjee_721; 
      InputType input_type = InputType::PCAP_FILE;  
      WJInputParam input_param;                     
      WJDecoderParam decoder_param;                 
      void print() const
      {
        WJ_INFO << "----------------------------------------------" << WJ_REND;
        WJ_INFO << "            VANJEE Driver Parameters          " << WJ_REND;
        WJ_INFOL << "input_type: " << inputTypeToStr(input_type) << WJ_REND;
        WJ_INFOL << "lidar_type: " << lidarTypeToStr(lidar_type) << WJ_REND;
        WJ_INFO << "----------------------------------------------" << WJ_REND;
        input_param.print();
        decoder_param.print();
      }
    };
    struct WJMemsParam
    {
      float Rotate_mirror_pitch = 0.0f;       
      std::vector<float> Rotate_mirror_offset{0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
      std::vector<float> View_center_yaws{0.0f, 0.0f, 0.0f, 0.0f, 0.0f};    
      float rwadata_yaw_resolution = 0.0f;    
      float rwadata_pitch_resolution = 0.0f;  
      float start_pitch = 0.0f;               
      float end_pitch = 0.0f;                 
      float start_yaw = 0.0f;                 
      float end_yaw = 0.0f;                   
      float beta = 0.0f;                      
      float gama_z = 0.0f;                    
      float gama_y = 0.0f;                    
      bool reversal_horizontzal = false;      
      bool reversal_vertical = false;         
      int scan_echo_type =1;                  
    };
    

} 

} 
