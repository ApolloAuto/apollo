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

#include <arpa/inet.h>
#include <errno.h>
#include <fcntl.h>
#include <netinet/in.h>
#include <pthread.h>
#include <setjmp.h>
#include <signal.h>
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <strings.h>
#include <sys/ipc.h>
#include <sys/msg.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <syslog.h>
#include <unistd.h>

#include "src/util.h"

#define DEFAULT_TIMEOUT 10 /*secondes waitting for read/write*/

int sys_readn(int fd, void* vptr, int n) {
  // printf("start sys_readn: %d....\n", n);
  int nleft, nread;
  char* ptr;

  ptr = vptr;
  nleft = n;
  while (nleft > 0) {
    // printf("start read\n");
    if ((nread = read(fd, ptr, nleft)) < 0) {
      if (errno == EINTR)
        nread = 0;
      else
        return -1;
    } else if (nread == 0) {
      break;
    }
    // printf("end read, read: %d\n", nread);
    nleft -= nread;
    ptr += nread;
  }
  // printf("stop sys_readn....\n");

  return n - nleft;
}

int sys_writen(int fd, const void* vptr, int n) {
  int nleft;
  int nwritten;
  const char* ptr;

  ptr = vptr;
  nleft = n;
  while (nleft > 0) {
    if ((nwritten = write(fd, ptr, nleft)) <= 0) {
      if (nwritten < 0 && errno == EINTR)
        nwritten = 0; /* and call write() again */
      else
        return (-1); /* error */
    }

    nleft -= nwritten;
    ptr += nwritten;
  }

  return n;
}

int tcp_open(const char* ipaddr, int port) {
  int sockfd;
  struct sockaddr_in servaddr;

  if ((sockfd = socket(AF_INET, SOCK_STREAM, 0)) == -1) return -1;

  bzero(&servaddr, sizeof(servaddr));
  servaddr.sin_family = AF_INET;
  servaddr.sin_port = htons(port);
  if (inet_pton(AF_INET, ipaddr, &servaddr.sin_addr) <= 0) {
    close(sockfd);
    return -1;
  }

  if (connect(sockfd, (struct sockaddr*)&servaddr, sizeof(servaddr)) == -1) {
    close(sockfd);
    return -1;
  }

  return sockfd;
}

/**
 *Description:check the socket  state
 *
 * @param
 * fd: socket
 * timeout:the time out of select
 * wait_for:socket state(r,w,conn)
 *
 * @return 1 if everything was ok, 0 otherwise
 */
int select_fd(int fd, int timeout, int wait_for) {
  fd_set fdset;
  fd_set *rd = NULL, *wr = NULL;
  struct timeval tmo;
  int result;

  FD_ZERO(&fdset);
  FD_SET(fd, &fdset);
  if (wait_for == WAIT_FOR_READ) {
    rd = &fdset;
  }
  if (wait_for == WAIT_FOR_WRITE) {
    wr = &fdset;
  }
  if (wait_for == WAIT_FOR_CONN) {
    rd = &fdset;
    wr = &fdset;
  }

  tmo.tv_sec = timeout;
  tmo.tv_usec = 0;
  do {
    result = select(fd + 1, rd, wr, NULL, &tmo);
  } while (result < 0 && errno == EINTR);

  return result;
}
