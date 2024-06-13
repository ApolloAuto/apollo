
#pragma once

#include <vanjeelidar/driver/input/input.hpp>

namespace vanjee
{
namespace lidar
{
class InputRaw : public Input
{
public:

  virtual bool init(){return true;}
  virtual bool start(){return true;};
  virtual void stop(){}
  virtual ~InputRaw(){}

  void feedPacket(const uint8_t* data, size_t size);

  InputRaw(const WJInputParam& input_param);

protected:
  size_t pkt_buf_len_;
  size_t raw_offset_;
  size_t raw_tail_;
};

inline InputRaw::InputRaw(const WJInputParam& input_param)
  : Input(input_param), pkt_buf_len_(ETH_LEN), 
    raw_offset_(0), raw_tail_(0)
{
  raw_offset_ += input_param.user_layer_bytes;
  raw_tail_   += input_param.tail_layer_bytes;
}

inline void InputRaw::feedPacket(const uint8_t* data, size_t size)
{
  std::shared_ptr<Buffer> pkt = cb_get_pkt_(pkt_buf_len_);
  memcpy(pkt->data(), data + raw_offset_, size - raw_offset_ - raw_tail_);
  pkt->setData(0, size - raw_offset_ - raw_tail_);
  pushPacket(pkt);
}

}  // namespace lidar
}  // namespace vanjee