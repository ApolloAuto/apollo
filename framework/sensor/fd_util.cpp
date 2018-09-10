
#include <arpa/inet.h>
#include <errno.h>
#include <fcntl.h>
#include <iostream>
#include <sys/file.h>
#include <unistd.h>
#include <string>
#include <sys/socket.h>
#include <stdint.h>
#include <string.h>
#include "cybertron/common/log.h"

#include "sensor/fd_util.h"

namespace apollo {
namespace sensor {

bool InitUdp(const char* ip, uint32_t port, int& fd) {

  if (fd != -1) {
    (void)close(fd);
  }

  AINFO << "Opening UDP socket:" << port;
  fd = socket(AF_INET, SOCK_DGRAM, 0);

  if (fd == -1) {
    AERROR << " Init socket failed, UDP port is " << port;
    return false;
  }

  sockaddr_in my_addr;                       // my address information
  memset(&my_addr, 0, sizeof(my_addr));      // initialize to zeros
  my_addr.sin_family = AF_INET;              // host byte order
  my_addr.sin_port = htons(uint16_t(port));  // short, in network byte order
  my_addr.sin_addr.s_addr = INADDR_ANY;      // automatically fill in my IP

  int opt=SO_REUSEADDR;
  if (setsockopt(fd,SOL_SOCKET,SO_REUSEADDR,&opt,sizeof(opt)) != 0) {
    AERROR << " Socket setsockopt failed!";
    return false;
  }

  if (bind(fd, (sockaddr *)&my_addr, sizeof(sockaddr)) == -1) {
    AERROR << " Socket bind failed! Port " << port;
    return false;
  }

  if (fcntl(fd, F_SETFL, fcntl(fd, F_GETFD, 0) | O_NONBLOCK) < 0) {
    AERROR << " non-block! Port " << port;
    return false;
  }

  AINFO << "Velodyne socket fd is " << fd << ", port " << port;

  return true;

}

} // namespace sensor 
} // namespace apollo 

