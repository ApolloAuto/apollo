
#pragma once

#include <vanjeelidar/driver/decoder/decoder.hpp>
#include <vanjeelidar/driver/decoder/wlr720/decoder/decoder_vanjee_720.hpp>
#include <vanjeelidar/driver/decoder/wlr720_32/decoder/decoder_vanjee_720_32.hpp>

namespace vanjee
{
namespace lidar
{ 
    template <typename T_PointCloud>
    class DecoderFactory
    {
    public:
        static std::shared_ptr<Decoder<T_PointCloud>> createDecoder(LidarType type, const WJDecoderParam &param);
    };
    template <typename T_PointCloud>
    inline std::shared_ptr<Decoder<T_PointCloud>> DecoderFactory<T_PointCloud>::createDecoder(LidarType type, const WJDecoderParam &param)
    {
        std::shared_ptr<Decoder<T_PointCloud>> ret_ptr;
        switch (type)
        {
            case LidarType::vanjee_720:
            case LidarType::vanjee_720_16:
                ret_ptr = std::make_shared<DecoderVanjee720<T_PointCloud>>(param);
                break;
            case LidarType::vanjee_720_32:
                ret_ptr = std::make_shared<DecoderVanjee720_32<T_PointCloud>>(param);
                break;
            
            default:
                exit(-1);
        }
        return ret_ptr;
    }

} 
} 
