
#pragma once
#ifdef _WIN32
#include <vanjeelidar/driver/input/input.hpp>

#include <winsock2.h>
#include <ws2tcpip.h>

#pragma warning(disable : 4244)

namespace vanjee
{
  namespace lidar
  {
    class InputUdpSocket : public Input
    {
      public:
        InputUdpSocket(const WJInputParam& input_param)
          : Input(input_param), pkt_buf_len_(UDP_ETH_LEN),
            sock_offset_(0), sock_tail_(0)
        {
          sock_offset_ += input_param.user_layer_bytes;
          sock_tail_   += input_param.tail_layer_bytes;
        }

        virtual bool init();
        virtual bool start();
        virtual int32 send_(uint8* buf,uint32 size);
        virtual ~InputUdpSocket();

      private:
        inline void recvPacket();
        inline int createUdpSocket(uint16_t port, const std::string& hostIp, const std::string& grpIp);

      protected:
        size_t pkt_buf_len_;
        int fds_;
        size_t sock_offset_;
        size_t sock_tail_;
    };

    inline bool InputUdpSocket::init()
    {
      if (init_flag_)
        return true;

      int msop_fd = -1;

      const int create_socket_retry_num = 60;
      int attempt = 0;

      WORD version = MAKEWORD(2, 2);
      WSADATA wsaData;

      int ret = -1;
      while(ret < 0 && attempt < create_socket_retry_num)
      {
        ret = WSAStartup(version, &wsaData);
        if(msop_fd > 0)
              break;;
          Sleep(5000);
          attempt++;
          WJ_WARNING << "failed to start WSA, retrying..." << WJ_REND;
      }
      if(ret < 0)
      {
        WJ_ERROR << "failed to start WSA, timeout!" << WJ_REND;
        goto failWsa;
      }
      
      attempt = 0;
      while(msop_fd < 0 && attempt < create_socket_retry_num)
      {
          msop_fd = createUdpSocket(input_param_.host_msop_port, input_param_.host_address, input_param_.group_address);
          if(msop_fd > 0)
              break;;
          Sleep(5000);
          attempt++;
          WJ_WARNING << "failed to create udp socket, retrying..." << WJ_REND;
      }
      
      if (msop_fd < 0)
      {
          WJ_ERROR << "failed to create udp socket, timeout!" << WJ_REND;
          goto failMsop;
      }

      fds_ = msop_fd;

      init_flag_ = true;
      return true;

      failMsop:
      failWsa:
        return false;
    }

    inline bool InputUdpSocket::start()
    {
      if (start_flag_)
        return true;

      if (!init_flag_)
      {
        cb_excep_(Error(ERRCODE_STARTBEFOREINIT));
        return false;
      }

      to_exit_recv_ = false;
      recv_thread_ = std::thread(std::bind(&InputUdpSocket::recvPacket, this));
      start_flag_ = true;
      return true;
    }

    inline InputUdpSocket::~InputUdpSocket()
    {
      if(init_flag_)
      {
        stop();
        closesocket(fds_);
      }
    }

    /// @brief reateSocket()用于创建UDP Socket。
    inline int InputUdpSocket::createUdpSocket(uint16_t port, const std::string& hostIp, const std::string& grpIp)
    {
      int fd;
      int ret;
      int reuse = 1;
      if (hostIp == "0.0.0.0" && grpIp == "0.0.0.0")
      {
        perror("ip err: ");
        goto failSocket;
      }

      fd = socket(PF_INET, SOCK_DGRAM, 0);
      if (fd < 0)
      {
        perror("socket: ");
        goto failSocket;
      }

      ret = setsockopt(fd, SOL_SOCKET, SO_REUSEADDR, (const char*)&reuse, sizeof(reuse));
      if (ret < 0)
      {
        perror("setsockopt(SO_REUSEADDR): ");
        goto failOption;
      }

      {
        Sleep(1000);
        int set_rcv_size = 1024*1024;//设置为1M
        int set_optlen=sizeof(set_rcv_size);
        ret = setsockopt(fd,SOL_SOCKET,SO_RCVBUF,(const char*)&set_rcv_size,set_optlen);
      }

      struct sockaddr_in host_addr;
      memset(&host_addr, 0, sizeof(host_addr));
      host_addr.sin_family = AF_INET;
      host_addr.sin_port = htons(port);
      host_addr.sin_addr.s_addr = INADDR_ANY;
      if (hostIp != "0.0.0.0" && grpIp == "0.0.0.0")
      {
        inet_pton(AF_INET, hostIp.c_str(), &(host_addr.sin_addr));
      }

      ret = ::bind(fd, (struct sockaddr*)&host_addr, sizeof(host_addr));
      if (ret < 0)
      {
        perror("bind: ");
        goto failBind;
      }

      if (grpIp != "0.0.0.0")
      {
    #if 0
        struct ip_mreqn ipm;
        memset(&ipm, 0, sizeof(ipm));
        inet_pton(AF_INET, grpIp.c_str(), &(ipm.imr_multiaddr));
        inet_pton(AF_INET, hostIp.c_str(), &(ipm.imr_address));
    #else
        struct ip_mreq ipm;
        inet_pton(AF_INET, grpIp.c_str(), &(ipm.imr_multiaddr));
        inet_pton(AF_INET, hostIp.c_str(), &(ipm.imr_interface));
    #endif
        ret = setsockopt(fd, IPPROTO_IP, IP_ADD_MEMBERSHIP, (const char*)&ipm, sizeof(ipm));
        if (ret < 0)
        {
          perror("setsockopt(IP_ADD_MEMBERSHIP): ");
          goto failGroup;
        }
      }

    #ifdef ENABLE_DOUBLE_RCVBUF
      {
        uint32_t opt_val;
        socklen_t opt_len = sizeof(uint32_t);
        getsockopt(fd, SOL_SOCKET, SO_RCVBUF, (char*)&opt_val, &opt_len);
        opt_val *= 4;
        setsockopt(fd, SOL_SOCKET, SO_RCVBUF, (char*)&opt_val, opt_len);
      }
    #endif

      {
        u_long mode = 1;
        ret = ioctlsocket(fd, FIONBIO, &mode);
        if (ret < 0)
        {
          perror("ioctlsocket: ");
          goto failNonBlock;
        }
      }
      return fd;

      failNonBlock:
      failGroup:
      failBind:
      failOption:
        closesocket(fd);
      failSocket:
        return -1;
    }

    int32 InputUdpSocket::send_(uint8* buf,uint32 size)
    {
        int32 ret = -1;
        struct sockaddr_in addr;
        socklen_t addrLen = sizeof(struct sockaddr_in);
        memset(&addr,0,addrLen);
        addr.sin_family = AF_INET;
        addr.sin_port = htons(input_param_.lidar_msop_port);

    //   if(input_param_.group_address != "0.0.0.0")
    //   {
    //     if(inet_pton(AF_INET,input_param_.group_address.c_str(),&addr.sin_addr) <= 0)
    //     {
    //         WJ_WARNING << "Invalid LiDAR Ip address." << WJ_REND;
    //         return -1;
    //     }
        
    //   }
    //   else 
    //   {
        if(inet_pton(AF_INET,input_param_.lidar_address.c_str(),&addr.sin_addr) <= 0)
        {
            WJ_WARNING << "Invalid LiDAR Ip address." << WJ_REND;
            return -1;
        }
    //   }

        if(fds_ > 0)
        {
            ret = sendto(fds_,(const char*)buf,size,0,(struct sockaddr*)&addr,addrLen);
        }

        return ret;
    }

    inline void InputUdpSocket::recvPacket()
    {
      int max_fd = fds_;

      while (!to_exit_recv_)
      {
        fd_set rfds;
        FD_ZERO(&rfds);
        FD_SET(fds_, &rfds);

        struct timeval tv;
        tv.tv_sec = 1;
        tv.tv_usec = 0;
        int retval = select(max_fd + 1, &rfds, NULL, NULL, &tv);
        if (retval == 0)
        {
          cb_excep_(Error(ERRCODE_MSOPTIMEOUT));
        }
        else if (retval < 0)
        {
          if (errno == EINTR)
          {
              Sleep(1000);
              continue;
          }

          perror("select: ");
          break;
        }

        if ((fds_ >= 0) && FD_ISSET(fds_, &rfds))
        {
          std::shared_ptr<Buffer> pkt = cb_get_pkt_(pkt_buf_len_);
          int ret = recvfrom(fds_, (char*)pkt->buf(), (int)pkt->bufSize(), 0, NULL, NULL);
              
          if (ret < 0)
          {
            perror("recvfrom: ");
          }
          else if (ret > 0)
          {
            pkt->setData(sock_offset_, ret - sock_offset_ - sock_tail_);
            pushPacket(pkt);
          }
        }
      }
    }
  }  
}  

#else
#ifdef ENABLE_EPOLL_RECEIVE
#include <vanjeelidar/driver/input/input.hpp>
#include <vanjeelidar/common/error_code.hpp>

#include <unistd.h>
#include <fcntl.h>
#include <arpa/inet.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <sys/epoll.h>

namespace vanjee
{
  namespace lidar
  {
    class InputUdpSocket : public Input
    {
    protected:
        size_t pkt_buf_len_;
        size_t socket_offset_;
        size_t socket_tail_;
        int fds_;
        int epfd_;

    public:
        InputUdpSocket(const WJInputParam &input_param);
        virtual bool init();
        virtual bool start();
        virtual int32 send_(uint8* buf,uint32 size);
        virtual ~InputUdpSocket();

    private:
        inline int createUdpSocket(uint16_t port, const std::string &honstIp, const std::string &grpIp);
        inline void recvPacket();
        inline std::string getIpAddr(const struct sockaddr_in& addr);
    };

    InputUdpSocket ::InputUdpSocket(const WJInputParam &input_param)
        : Input(input_param), pkt_buf_len_(UDP_ETH_LEN), socket_offset_(0), socket_tail_(0)
    {
        socket_offset_ += input_param.user_layer_bytes;
        socket_tail_ += input_param.tail_layer_bytes;
    }
    
    inline bool InputUdpSocket::init()
    {
      if (init_flag_)
        return true;
      int msop_fd = -1;
      
      const int create_socket_retry_num = 60;
      int attempt = 0;

      /// @brief 创建一个epoll的句柄，size用来告诉内核这个监听的数目一共有多大
      int epfd = -1;
      while(epfd < 0 && attempt < create_socket_retry_num)
      {
        epfd = epoll_create(1);
        if(epfd > 0)
              break;;
          sleep(5);
          attempt++;
          WJ_WARNING << "failed to create epoll, retrying..." << WJ_REND;
      }
      if (epfd < 0)
      {
        WJ_ERROR << "failed to create epoll, timeout!" << WJ_REND;
        goto failEpfd;
      }
      
      attempt = 0;
      while(msop_fd < 0 && attempt < create_socket_retry_num)
      {
          msop_fd = createUdpSocket(input_param_.host_msop_port, input_param_.host_address, input_param_.group_address);
          if(msop_fd > 0)
              break;;
          sleep(5);
          attempt++;
          WJ_WARNING << "failed to create udp socket, retrying..." << WJ_REND;
      }
      
      if (msop_fd < 0)
      {
          WJ_ERROR << "failed to create udp socket, timeout!" << WJ_REND;
          goto failMsop;
      }
      
      struct epoll_event ev;
      ev.data.fd = msop_fd;
      ev.events = EPOLLIN; 
      epoll_ctl(epfd, EPOLL_CTL_ADD, msop_fd, &ev);
      
      epfd_ = epfd;
      fds_ = msop_fd;
      init_flag_ = true;
      return true;

      failMsop:
          close(epfd);
      failEpfd:
          return false;
    }
    
    inline bool InputUdpSocket::start()
    {
      if (start_flag_)
        return true;

      if (!init_flag_)
      {
        cb_excep_(Error(ERRCODE_STARTBEFOREINIT));
        return false;
      }
      to_exit_recv_ = false;
      recv_thread_ = std::thread(std::bind(&InputUdpSocket::recvPacket, this));
      start_flag_ = true;
      return true;
    }
    
    inline InputUdpSocket ::~InputUdpSocket()
    {
      if(init_flag_)
      {
        stop();
        close(fds_);
        close(epfd_);
      }
    }

    /// @brief reateSocket()用于创建UDP Socket。
    inline int InputUdpSocket::createUdpSocket(uint16_t port, const std::string &honstIp, const std::string &grpIp)
    {
      int fd;
      int ret;
      int reuse = 1;
      if (honstIp == "0.0.0.0" && grpIp == "0.0.0.0")
      {
        perror("ip err: ");
        goto failSocket;
      }

      fd = socket(PF_INET, SOCK_DGRAM, 0);
      if (fd < 0)
      {
        perror("Socket: ");
        goto failSocket;
      }

      ret = setsockopt(fd, SOL_SOCKET, SO_REUSEADDR, &reuse, sizeof(reuse));
      if (ret < 0)
      {
        perror("secsockopt: ");
        goto failOption;
      }

      {
        sleep(1);
        int set_rcv_size = 1024*1024;//设置为1M
        int set_optlen=sizeof(set_rcv_size);
        ret = setsockopt(fd,SOL_SOCKET,SO_RCVBUF,(const int*)&set_rcv_size,set_optlen);
      } 

      struct sockaddr_in host_addr;
      memset(&host_addr, 0, sizeof(host_addr));
      host_addr.sin_port = htons(port);
      host_addr.sin_family = AF_INET;
      host_addr.sin_addr.s_addr = INADDR_ANY;
      if (honstIp != "0.0.0.0" && grpIp == "0.0.0.0")
      {
          inet_pton(AF_INET, honstIp.c_str(), &(host_addr.sin_addr));
      }

      /// @brief 给 socket 绑定一个地址结构 (IP +port)
      ret = bind(fd, (struct sockaddr *)&host_addr, sizeof(host_addr));
      if (ret < 0)
      {
          perror("bind: ");
          goto failBind;
      }

      if (grpIp != "0.0.0.0")
      {
#if 0
        struct ip_mreqn ipm;
        memset(&ipm, 0, sizeof(ipm));
        inet_pton(AF_INET, honstIp.c_str(), &(ipm.imr_address));
        inet_pton(AF_INET, grpIp.c_str(), &(ipm.imr_multiaddr));
#else
        struct ip_mreq ipm;
        memset(&ipm, 0, sizeof(ipm));
        inet_pton(AF_INET, honstIp.c_str(), (&ipm.imr_interface));
        inet_pton(AF_INET, grpIp.c_str(), (&ipm.imr_multiaddr));
#endif
        ret = setsockopt(fd, IPPROTO_IP, IP_ADD_MEMBERSHIP, &ipm, sizeof(ipm));
        if (ret < 0)
        {
          perror("setsockopt(IP_ADD_MENBERSHIP): ");
          goto failGroup;
        }
      }
      {
        int flags = fcntl(fd, F_GETFL, 0);
        ret = fcntl(fd, F_SETFL, (flags | O_NONBLOCK));
        if (ret < 0)
        {
            perror("fcntl: ");
            goto failNonBlock;
        }
      }
      return fd;
      failNonBlock:
      failGroup:
      failBind:
      failOption:
          close(fd);
      failSocket:
      return -1;
    }

    inline std::string InputUdpSocket::getIpAddr(const struct sockaddr_in& addr)
    {
      char ip[INET_ADDRSTRLEN];
      inet_ntop(AF_INET, &(addr.sin_addr), ip, INET_ADDRSTRLEN);
      return std::string(ip);
    }

    inline void InputUdpSocket::recvPacket()
    {
      while (!to_exit_recv_)
      {
        struct epoll_event events[8]; 
        int retval = epoll_wait(epfd_, events, 8, 1000);
        if (retval == 0)
        {
          cb_excep_(Error(ERRCODE_MSOPTIMEOUT));
        }
        else if (retval < 0)
        {
          if (errno == EINTR)
          {
              sleep(1);
              continue;
          }
          perror("epoll_wait: ");
          break;
        }
        for (int i = 0; i < retval; i++)
        {
          /// @brief 用于从(已连接)套接口上接收数据，并捕获数据发送源的地址
          std::shared_ptr<Buffer> pkt = cb_get_pkt_(pkt_buf_len_);

          struct sockaddr_in addr;
          socklen_t addrLen = sizeof(struct sockaddr_in);
          ssize_t ret = recvfrom(events[i].data.fd, pkt->buf(), pkt->bufSize(), 0, (struct sockaddr*)&addr, &addrLen);
          if (ret < 0)
          {
              perror("recvfrom: ");
          }
          else if (ret > 0)
          {
              pkt->setData(socket_offset_, ret - socket_offset_ - socket_tail_,getIpAddr(addr));
              pushPacket(pkt);
          }
        }
      }
    }

    int32 InputUdpSocket::send_(uint8* buf,uint32 size)
    {
      int32 ret = -1;
      struct sockaddr_in addr;
      socklen_t addrLen = sizeof(struct sockaddr_in);
      memset(&addr,0,addrLen);
      addr.sin_family = AF_INET;
      addr.sin_port = htons(input_param_.lidar_msop_port);

    //   if(input_param_.group_address != "0.0.0.0")
    //   {
    //     if(inet_pton(AF_INET,input_param_.group_address.c_str(),&addr.sin_addr) <= 0)
    //     {
    //         WJ_WARNING << "Invalid LiDAR Ip address." << WJ_REND;
    //         return -1;
    //     }
        
    //   }
    //   else 
    //   {
        if(inet_pton(AF_INET,input_param_.lidar_address.c_str(),&addr.sin_addr) <= 0)
        {
            WJ_WARNING << "Invalid LiDAR Ip address." << WJ_REND;
            return -1;
        }
    //   }

      if(fds_ > 0)
      {
        ret = sendto(fds_,buf,size,0,(struct sockaddr*)&addr,addrLen);
      }
      return ret;
    }
  } 
} 
#else
#include <vanjeelidar/driver/input/input.hpp>
#include <vanjeelidar/common/super_header.hpp>

#include <unistd.h>
#include <fcntl.h>
#include <arpa/inet.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <sys/select.h>

namespace vanjee
{
  namespace lidar
  {
    class InputUdpSocket : public Input
    {
    protected:
      size_t pkt_buf_len_;
      size_t socket_offset_;
      size_t socket_tail_;
      int fds_;

    private:
      inline std::string getIpAddr(const struct sockaddr_in& addr);

    public:
      InputUdpSocket(const WJInputParam &input_param);
      /// @brief 调用createSocket()，创建两个Socket,分别接收MSOP Packet和DIFOP Packet
      virtual bool init();
      /// @brief 启动接收线程，线程函数为InputSock::recvPacket()00
      virtual bool start();
      /// @brief 用于创建UDP Socket
      inline int createUdpSocket(uint16_t port, const std::string &hostIp, const std::string &grpIp);
      virtual int32 send_(uint8* buf,uint32 size);
      inline void recvPacket();
      virtual ~InputUdpSocket();
    };

    InputUdpSocket::InputUdpSocket(const WJInputParam &input_param)
        : Input(input_param), pkt_buf_len_(UDP_ETH_LEN),
          socket_offset_(0), socket_tail_(0)
    {
      socket_offset_ += input_param.user_layer_bytes;
      socket_tail_ += input_param.tail_layer_bytes;
    }

    inline bool InputUdpSocket::init()
    {
      if (init_flag_)
        return true;

      int msop_fd = -1;

      const int create_socket_retry_num = 60;
      int attempt = 0;
      while(msop_fd < 0 && attempt < create_socket_retry_num)
      {
          msop_fd = createUdpSocket(input_param_.host_msop_port, input_param_.host_address, input_param_.group_address);
          if(msop_fd > 0)
              break;;
          sleep(5);
          attempt++;
          WJ_WARNING << "failed to create udp socket, retrying..." << WJ_REND;
      }
      
      if (msop_fd < 0)
      {
          WJ_ERROR << "failed to create udp socket, timeout!" << WJ_REND;
          goto failMsop;
      }

      fds_ = msop_fd;
      init_flag_ = true;
      return true;

      failMsop:
        return false;
    }

    inline bool InputUdpSocket::start()
    {
      if (start_flag_)
        return true;

      if (!init_flag_)
      {
        cb_excep_(Error(ERRCODE_STARTBEFOREINIT));
        return false;
      }

      to_exit_recv_ = false;
      recv_thread_ = std::thread(std::bind(&InputUdpSocket::recvPacket, this));
      start_flag_ = true;
      return true;
    }

    InputUdpSocket::~InputUdpSocket()
    {
      if(init_flag_)
      {
        stop();
        close(fds_);
      }
    }
    
    /// @brief reateSocket()用于创建UDP Socket。
    inline int InputUdpSocket::createUdpSocket(uint16_t port, const std::string &hostIp, const std::string &grpIp)
    {
      int fd;
      int ret;
      int reuse = 1;
      if (hostIp == "0.0.0.0" && grpIp == "0.0.0.0")
      { 
        perror("ip err: ");
        goto failSocket;
      }

      fd = socket(PF_INET, SOCK_DGRAM, 0);
      if (fd < 0)
      {
        perror("socket: ");
        goto failSocket;
      }

      ret = setsockopt(fd, SOL_SOCKET, SO_REUSEADDR, &reuse, sizeof(reuse));
      if (ret < 0)
      {
        perror("setsockopt(SO_REUSEADDR): ");
        goto failOption;
      }

      {
        sleep(1);
        int set_rcv_size = 1024*1024;//设置为1M
        int set_optlen=sizeof(set_rcv_size);
        ret = setsockopt(fd,SOL_SOCKET,SO_RCVBUF,(const int*)&set_rcv_size,set_optlen);
        
        // sleep(1);
        // int recv;
        // socklen_t recvlen;
        // ret = getsockopt(fd,SOL_SOCKET,SO_RCVBUF,(int*)&recv,(socklen_t *)&recvlen);
        // WJ_INFO << "recv:" << recv << " , recvlen:" << recvlen << WJ_REND;

      }

      struct sockaddr_in host_addr;
      memset(&host_addr, 0, sizeof(host_addr));
      host_addr.sin_family = AF_INET;         
      host_addr.sin_port = htons(port);       
      host_addr.sin_addr.s_addr = INADDR_ANY; 
      if (hostIp != "0.0.0.0" && grpIp == "0.0.0.0")
      { 
        inet_pton(AF_INET, hostIp.c_str(), &(host_addr.sin_addr));
      }
      /// @brief 给 socket 绑定一个地址结构 (IP +port)
      ret = bind(fd, (struct sockaddr *)&host_addr, sizeof(host_addr));
      if (ret < 0)
      {
        perror("bind: ");
        goto failBind;              
      }
      if (grpIp != "0.0.0.0")
      {
#if 0               
        struct ip_mreqn ipm;
        memset(&ipm, 0, sizeof(ipm));
        inet_pton(AF_INET, grpIp.c_str(), &(ipm.imr_multiaddr));
        inet_pton(AF_INET, hostIp.c_str(), &(ipm.imr_address));
#else
        struct ip_mreq ipm;
        memset(&ipm, 0, sizeof(ipm));
        inet_pton(AF_INET, grpIp.c_str(), &(ipm.imr_multiaddr));
        inet_pton(AF_INET, hostIp.c_str(), &(ipm.imr_interface));
#endif
        ret = setsockopt(fd, IPPROTO_IP, IP_ADD_MEMBERSHIP, &ipm, sizeof(ipm));
        if (ret < 0)
        {
          perror("setsockopt(IP_ADD_MENBERSHIP): ");
          goto failGroup;
        }
      }
#ifdef ENABLE_DOUBLE_RCVBUF
      {
        uint32_t opt_val;  
        socklen_t opt_len; 
        getsockopt(fd, SOL_SOCKET, SO_RCVBUF, (char *)&opt_val, &opt_len);
        opt_val *= 4;
        setsockopt(fd, SOL_SOCKET, SO_RCVBUF, (char *)&opt_val, opt_len);
      }
#endif
      {
        int flags = fcntl(fd, F_GETFL, 0);
        ret = fcntl(fd, F_SETFL, flags | O_NONBLOCK);
        if (ret < 0)
        {
          perror("fcntl: ");
          goto failNonBlock;
        }
      }
      return fd;

      failNonBlock:
      failGroup:
      failBind:
      failOption:
        close(fd);
      failSocket:
        return -1;
    }

    inline std::string InputUdpSocket::getIpAddr(const struct sockaddr_in& addr)
    {
      char ip[INET_ADDRSTRLEN];
      inet_ntop(AF_INET, &(addr.sin_addr), ip, INET_ADDRSTRLEN);
      return std::string(ip);
    }

    int32 InputUdpSocket::send_(uint8* buf,uint32 size)
    {
      int32 ret = -1;
      struct sockaddr_in addr;
      socklen_t addrLen = sizeof(struct sockaddr_in);
      memset(&addr,0,addrLen);
      addr.sin_family = AF_INET;
      addr.sin_port = htons(input_param_.lidar_msop_port);

    //   if(input_param_.group_address != "0.0.0.0")
    //   {
    //     if(inet_pton(AF_INET,input_param_.group_address.c_str(),&addr.sin_addr) <= 0)
    //     {
    //         WJ_WARNING << "Invalid LiDAR Ip address." << WJ_REND;
    //         return -1;
    //     }
    //   }
    //   else 
    //   {
        if(inet_pton(AF_INET,input_param_.lidar_address.c_str(),&addr.sin_addr) <= 0)
        {
            WJ_WARNING << "Invalid LiDAR Ip address." << WJ_REND;
            return -1;
        }
    //   }

      if(fds_ > 0)
      {
        ret = sendto(fds_,buf,size,0,(struct sockaddr*)&addr,addrLen);
      }

      return ret;
    }

    inline void InputUdpSocket::recvPacket()
    {
        int max_fd = fds_;
        while (!to_exit_recv_)
        {
            fd_set rfds;
            FD_ZERO(&rfds);
            FD_SET(fds_, &rfds);

            struct timeval tv;
            tv.tv_sec = 1;
            tv.tv_usec = 0;
            int retval = select(max_fd + 1, &rfds, NULL, NULL, &tv);
            if (retval == 0)
            {
              cb_excep_(Error(ERRCODE_MSOPTIMEOUT));
            }
            else if (retval < 0)
            {
              if (errno == EINTR)
              {
                sleep(1);
                continue;
              }
                  
              perror("select: ");
              break;
            }
            
            if ((fds_ >= 0) && FD_ISSET(fds_, &rfds))
            {
              std::shared_ptr<Buffer> pkt = cb_get_pkt_(pkt_buf_len_);

              /// @brief 用于从(已连接)套接口上接收数据，并捕获数据发送源的地址
              struct sockaddr_in addr;
              socklen_t addrLen = sizeof(struct sockaddr_in);
              ssize_t ret = recvfrom(fds_, pkt->buf(), pkt->bufSize(), 0, (struct sockaddr*)&addr, &addrLen);

              if (ret < 0)
              {
                perror("recvfrom: \r\n");
              }
              else if (ret > 0)
              {
                pkt->setData(socket_offset_, ret - socket_offset_ - socket_tail_,getIpAddr(addr));
                pushPacket(pkt);
              }
            }
        }
    }
  } 
} 
#endif
#endif
