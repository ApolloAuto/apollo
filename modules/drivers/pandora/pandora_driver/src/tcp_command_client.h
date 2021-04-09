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

#ifndef SRC_TCP_COMMAND_CLIENT_H_
#define SRC_TCP_COMMAND_CLIENT_H_

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
  PTC_COMMAND_GET_CALIBRATION = 0,
  PTC_COMMAND_SET_CALIBRATION,
  PTC_COMMAND_HEARTBEAT,
  PTC_COMMAND_RESET_CALIBRATION,
  PTC_COMMAND_TEST,
  PTC_COMMAND_GET_LIDAR_CALIBRATION,
} PTC_COMMAND;

typedef enum {
  PTC_ERROR_NO_ERROR = 0,
  PTC_ERROR_BAD_PARAMETER,
  PTC_ERROR_CONNECT_SERVER_FAILED,
  PTC_ERROR_TRANSFER_FAILED,
  PTC_ERROR_NO_MEMORY,
} PTC_ErrCode;

typedef struct TcpCommandHeader_s {
  unsigned char cmd;
  unsigned char ret_code;
  unsigned int len;
} TcpCommandHeader;

typedef struct TC_Command_s {
  TcpCommandHeader header;
  unsigned char* data;

  unsigned char* ret_data;
  unsigned int ret_size;
} TC_Command;

void* TcpCommandClientNew(const char* ip, const unsigned short port);
PTC_ErrCode TcpCommandSetCalibration(const void* handle, const char* buffer,
                                     unsigned int len);
PTC_ErrCode TcpCommandGetCalibration(const void* handle, char** buffer,
                                     unsigned int* len);
PTC_ErrCode TcpCommandGetLidarCalibration(const void* handle, char** buffer,
                                          unsigned int* len);
PTC_ErrCode TcpCommandResetCalibration(const void* handle);
void TcpCommandClientDestroy(const void* handle);

#ifdef __cplusplus
}
#endif

#endif  //  SRC_TCP_COMMAND_CLIENT_H_
