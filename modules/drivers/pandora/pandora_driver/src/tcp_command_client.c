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
#include <sys/ipc.h>
#include <sys/msg.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <syslog.h>
#include <unistd.h>
#include <linux/sockios.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include "src/util.h"
#include "src/tcp_command_client.h"

typedef struct TcpCommandClient_s {
  pthread_mutex_t lock;
  pthread_t tid;

  int exit;

  char ip[256];
  unsigned short port;

  int fd;
} TcpCommandClient;

#ifdef DEBUG
static void print_mem(char* mem, int len) {
  int i = 0;
  for (int i = 0; i < len; ++i) {
    printf("%02x ", mem[i]);
  }
  printf("\n");
}
#else
static void print_mem(char* mem, int len) {}
#endif

static int tcpCommandHeaderParser(unsigned char* buffer, int len,
                                  TcpCommandHeader* header) {
  int index = 0;
  header->cmd = buffer[index++];
  header->ret_code = buffer[index++];
  header->len =
      ((buffer[index] & 0xff) << 24) | ((buffer[index + 1] & 0xff) << 16) |
      ((buffer[index + 2] & 0xff) << 8) | ((buffer[index + 3] & 0xff) << 0);
  return 0;
}

static int tcpCommandReadCommand(int connfd, TC_Command* cmd) {
  int ret = 0;
  if (!cmd) {
    return -1;
  }
  memset(cmd, 0, sizeof(TC_Command));
  unsigned char buffer[1500];
  ret = sys_readn(connfd, buffer, 2);
  if (ret <= 0 || buffer[0] != 0x47 || buffer[1] != 0x74) {
    printf("Server Read failed\n");
    return -1;
  }

  ret = sys_readn(connfd, buffer + 2, 6);
  if (ret != 6) {
    printf("Server Read failed\n");
    return -1;
  }

  print_mem(buffer, 8);

  tcpCommandHeaderParser(buffer + 2, 6, &cmd->header);

  if (cmd->header.len > 0) {
    cmd->data = malloc(cmd->header.len);
    if (!cmd->data) {
      printf("malloc data error\n");
      return -1;
    }
  }

  ret = sys_readn(connfd, cmd->data, cmd->header.len);
  if (ret != cmd->header.len) {
    free(cmd->data);
    printf("Server Read failed\n");
    return -1;
  }

  // cmd->ret_size = cmd->header.len;

  print_mem(cmd->data, cmd->header.len);

  return 0;
}

static int TcpCommand_buildHeader(char* buffer, TC_Command* cmd) {
  if (!buffer) {
    return -1;
  }
  int index = 0;
  buffer[index++] = 0x47;
  buffer[index++] = 0x74;
  buffer[index++] = cmd->header.cmd;
  buffer[index++] = cmd->header.ret_code;  // color or mono
  buffer[index++] = (cmd->header.len >> 24) & 0xff;
  buffer[index++] = (cmd->header.len >> 16) & 0xff;
  buffer[index++] = (cmd->header.len >> 8) & 0xff;
  buffer[index++] = (cmd->header.len >> 0) & 0xff;

  return index;
}

static PTC_ErrCode tcpCommandClient_SendCmd(TcpCommandClient* client,
                                            TC_Command* cmd) {
  if (!client && !cmd) {
    printf("Bad Parameter\n");
    return PTC_ERROR_BAD_PARAMETER;
  }

  if (cmd->header.len != 0 && cmd->data == NULL) {
    printf("Bad Parameter : payload is null\n");
    return PTC_ERROR_BAD_PARAMETER;
  }

  pthread_mutex_lock(&client->lock);
  int fd = tcp_open(client->ip, client->port);
  if (fd < 0) {
    printf("Connect to Server Failed\n");
    pthread_mutex_unlock(&client->lock);
    return PTC_ERROR_CONNECT_SERVER_FAILED;
  }

  unsigned char buffer[128];
  int size = TcpCommand_buildHeader(buffer, cmd);

  print_mem(buffer, size);
  int ret = write(fd, buffer, size);
  if (ret != size) {
    close(fd);
    pthread_mutex_unlock(&client->lock);
    printf("Write header error\n");
    return PTC_ERROR_TRANSFER_FAILED;
  }

  if (cmd->header.len > 0 && cmd->data) {
    print_mem(cmd->data, cmd->header.len);
    ret = write(fd, cmd->data, cmd->header.len);
    if (ret != cmd->header.len) {
      printf("Write Payload error\n");
      close(fd);
      pthread_mutex_unlock(&client->lock);
      return PTC_ERROR_TRANSFER_FAILED;
    }
  }

  TC_Command feedBack;
  ret = tcpCommandReadCommand(fd, &feedBack);
  if (ret != 0) {
    printf("Receive feed back failed!!!\n");
    close(fd);
    pthread_mutex_unlock(&client->lock);
    return PTC_ERROR_TRANSFER_FAILED;
  }
  // printf("feed back : %d %d %d \n", feedBack.header.len ,
  // cmd->header.ret_code,
  //        cmd->header.cmd);

  cmd->ret_data = feedBack.data;
  cmd->ret_size = feedBack.header.len;
  cmd->header.ret_code = feedBack.header.ret_code;

  close(fd);
  pthread_mutex_unlock(&client->lock);
  return PTC_ERROR_NO_ERROR;
}

void* TcpCommandClientNew(const char* ip, const unsigned short port) {
  if (!ip) {
    printf("Bad Parameter\n");
    return NULL;
  }

  TcpCommandClient* client =
      (TcpCommandClient*)malloc(sizeof(TcpCommandClient));
  if (!client) {
    printf("No Memory!!!\n");
    return NULL;
  }
  memset(client, 0, sizeof(TcpCommandClient));
  client->fd = -1;
  strcpy(client->ip, ip);
  client->port = port;

  pthread_mutex_init(&client->lock, NULL);

  printf("TCP Command Client Init Success!!!\n");
  return (void*)client;
}

PTC_ErrCode TcpCommandSetCalibration(const void* handle, const char* buffer,
                                     unsigned int len) {
  if (!handle || !buffer || len <= 0) {
    printf("Bad Parameter!!!\n");
    return PTC_ERROR_BAD_PARAMETER;
  }
  TcpCommandClient* client = (TcpCommandClient*)handle;

  TC_Command cmd;
  memset(&cmd, 0, sizeof(TC_Command));
  cmd.header.cmd = PTC_COMMAND_SET_CALIBRATION;
  cmd.header.len = len;
  cmd.data = strdup(buffer);

  PTC_ErrCode errorCode = tcpCommandClient_SendCmd(client, &cmd);
  if (errorCode != PTC_ERROR_NO_ERROR) {
    printf("Set Calibration Failed\n");
    free(cmd.data);
    return errorCode;
  }
  free(cmd.data);

  if (cmd.ret_data) {
    // useless data;
    free(cmd.ret_data);
  }

  return cmd.header.ret_code;
}

PTC_ErrCode TcpCommandGetCalibration(const void* handle, char** buffer,
                                     unsigned int* len) {
  if (!handle || !buffer || !len) {
    printf("Bad Parameter!!!\n");
    return PTC_ERROR_BAD_PARAMETER;
  }
  TcpCommandClient* client = (TcpCommandClient*)handle;

  TC_Command cmd;
  memset(&cmd, 0, sizeof(TC_Command));
  cmd.header.cmd = PTC_COMMAND_GET_CALIBRATION;
  cmd.header.len = 0;
  cmd.data = NULL;

  PTC_ErrCode errorCode = tcpCommandClient_SendCmd(client, &cmd);
  if (errorCode != PTC_ERROR_NO_ERROR) {
    printf("Set Calibration Failed\n");
    return errorCode;
  }

  char* ret_str = (char*)malloc(cmd.ret_size + 1);
  memcpy(ret_str, cmd.ret_data, cmd.ret_size);
  ret_str[cmd.ret_size] = '\0';

  free(cmd.ret_data);

  *buffer = ret_str;
  *len = cmd.ret_size + 1;

  return cmd.header.ret_code;
}
PTC_ErrCode TcpCommandGetLidarCalibration(const void* handle, char** buffer,
                                          unsigned int* len) {
  if (!handle || !buffer || !len) {
    printf("Bad Parameter!!!\n");
    return PTC_ERROR_BAD_PARAMETER;
  }
  TcpCommandClient* client = (TcpCommandClient*)handle;

  TC_Command cmd;
  memset(&cmd, 0, sizeof(TC_Command));
  cmd.header.cmd = PTC_COMMAND_GET_LIDAR_CALIBRATION;
  cmd.header.len = 0;
  cmd.data = NULL;

  PTC_ErrCode errorCode = tcpCommandClient_SendCmd(client, &cmd);
  if (errorCode != PTC_ERROR_NO_ERROR) {
    printf("Set Calibration Failed\n");
    return errorCode;
  }

  char* ret_str = (char*)malloc(cmd.ret_size + 1);
  memcpy(ret_str, cmd.ret_data, cmd.ret_size);
  ret_str[cmd.ret_size] = '\0';

  free(cmd.ret_data);

  *buffer = ret_str;
  *len = cmd.ret_size + 1;

  return cmd.header.ret_code;
}

PTC_ErrCode TcpCommandResetCalibration(const void* handle) {
  if (!handle) {
    printf("Bad Parameter!!!\n");
    return PTC_ERROR_BAD_PARAMETER;
  }
  TcpCommandClient* client = (TcpCommandClient*)handle;

  TC_Command cmd;
  memset(&cmd, 0, sizeof(TC_Command));
  cmd.header.cmd = PTC_COMMAND_RESET_CALIBRATION;
  cmd.header.len = 0;
  cmd.data = NULL;

  PTC_ErrCode errorCode = tcpCommandClient_SendCmd(client, &cmd);
  if (errorCode != PTC_ERROR_NO_ERROR) {
    printf("Set Calibration Failed\n");
    return errorCode;
  }

  if (cmd.ret_data) {
    // useless data;
    free(cmd.ret_data);
  }

  return cmd.header.ret_code;
}

void TcpCommandClientDestroy(const void* handle) {}
