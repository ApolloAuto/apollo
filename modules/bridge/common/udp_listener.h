/******************************************************************************
 * Copyright 2018 The Apollo Authors. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *****************************************************************************/

#pragma once

#include <arpa/inet.h>
#include <fcntl.h>
#include <netinet/in.h>
#include <pthread.h>
#include <sys/epoll.h>
#include <sys/resource.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <sys/wait.h>
#include <unistd.h>

namespace apollo {
namespace bridge {

constexpr int MAXEPOLLSIZE = 100;

template <typename T>
class UDPListener {
 public:
  typedef bool (T::*func)(int fd);
  UDPListener() {}
  explicit UDPListener(T *receiver, uint16_t port, func msg_handle) {
    receiver_ = receiver;
    listened_port_ = port;
    msg_handle_ = msg_handle;
  }
  ~UDPListener() {
    if (listener_sock_ != -1) {
      close(listener_sock_);
    }
  }

  void SetMsgHandle(func msg_handle) { msg_handle_ = msg_handle; }
  bool Initialize(T *receiver, func msg_handle, uint16_t port);
  bool Listen();

  static void *pthread_handle_message(void *param);

 public:
  struct Param {
    int fd_ = 0;
    UDPListener<T> *listener_ = nullptr;
  };

 private:
  bool setnonblocking(int sockfd);
  void MessageHandle(int fd);

 private:
  T *receiver_ = nullptr;
  uint16_t listened_port_ = 0;
  int listener_sock_ = -1;
  func msg_handle_ = nullptr;
  int kdpfd_ = 0;
};

template <typename T>
bool UDPListener<T>::Initialize(T *receiver, func msg_handle, uint16_t port) {
  msg_handle_ = msg_handle;
  if (!msg_handle_) {
    return false;
  }

  receiver_ = receiver;
  if (!receiver_) {
    return false;
  }
  listened_port_ = port;
  struct rlimit rt;
  rt.rlim_max = rt.rlim_cur = MAXEPOLLSIZE;
  if (setrlimit(RLIMIT_NOFILE, &rt) == -1) {
    return false;
  }

  listener_sock_ = socket(AF_INET, SOCK_DGRAM, 0);
  if (listener_sock_ == -1) {
    return false;
  }
  int opt = SO_REUSEADDR;
  setsockopt(listener_sock_, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));
  setnonblocking(listener_sock_);

  struct sockaddr_in serv_addr;
  serv_addr.sin_family = PF_INET;
  serv_addr.sin_port = htons((uint16_t)listened_port_);
  serv_addr.sin_addr.s_addr = INADDR_ANY;
  if (bind(listener_sock_, (struct sockaddr *)&serv_addr,
           sizeof(struct sockaddr)) == -1) {
    close(listener_sock_);
    return false;
  }
  kdpfd_ = epoll_create(MAXEPOLLSIZE);
  struct epoll_event ev;
  ev.events = EPOLLIN | EPOLLET;
  ev.data.fd = listener_sock_;
  if (epoll_ctl(kdpfd_, EPOLL_CTL_ADD, listener_sock_, &ev) < 0) {
    close(listener_sock_);
    return false;
  }
  return true;
}

template <typename T>
bool UDPListener<T>::Listen() {
  int nfds = -1;
  bool res = true;
  struct epoll_event events[MAXEPOLLSIZE];
  while (true) {
    nfds = epoll_wait(kdpfd_, events, 10000, -1);
    if (nfds == -1) {
      res = false;
      break;
    }

    for (int i = 0; i < nfds; ++i) {
      if (events[i].data.fd == listener_sock_) {
        pthread_t thread;
        pthread_attr_t attr;
        pthread_attr_init(&attr);
        pthread_attr_setscope(&attr, PTHREAD_SCOPE_SYSTEM);
        pthread_attr_setdetachstate(&attr, PTHREAD_CREATE_DETACHED);
        Param *par = new Param;
        par->fd_ = events[i].data.fd;
        par->listener_ = this;
        if (pthread_create(&thread, &attr,
                           &UDPListener<T>::pthread_handle_message,
                           reinterpret_cast<void *>(par))) {
          res = false;
          return res;
        }
      }
    }
  }
  close(listener_sock_);
  return res;
}

template <typename T>
bool UDPListener<T>::setnonblocking(int sockfd) {
  if (fcntl(sockfd, F_SETFL, fcntl(sockfd, F_GETFD, 0) | O_NONBLOCK) == -1) {
    return false;
  }
  return true;
}

template <typename T>
void *UDPListener<T>::pthread_handle_message(void *param) {
  Param *par = static_cast<Param *>(param);
  int fd = par->fd_;
  UDPListener<T> *listener = par->listener_;
  if (par) {
    delete par;
  }
  par = nullptr;
  if (!listener) {
    pthread_exit(nullptr);
  }
  listener->MessageHandle(fd);
  pthread_exit(nullptr);
}

template <typename T>
void UDPListener<T>::MessageHandle(int fd) {
  if (!receiver_ || !msg_handle_) {
    return;
  }
  (receiver_->*msg_handle_)(fd);
}

}  // namespace bridge
}  // namespace apollo
