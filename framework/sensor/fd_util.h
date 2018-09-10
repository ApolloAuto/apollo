
#ifndef SENSOR_FD_UTIL_H_
#define SENSOR_FD_UTIL_H_ 

namespace apollo {
namespace sensor {

bool InitUdp(const char* ip, uint32_t port, int& fd);

} // namespace sensor 
} // namespace apollo 

#endif //SENSOR_FD_UTIL_H_
