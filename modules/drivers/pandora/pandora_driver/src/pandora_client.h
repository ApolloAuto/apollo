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

#ifndef SRC_PANDORA_CLIENT_H_
#define SRC_PANDORA_CLIENT_H_

#ifdef __cplusplus
extern "C" {
#endif

#define PANDORA_CAMERA_UNIT (5)

#define UTC_TIME
#ifdef UTC_TIME
typedef struct {
  unsigned char UTC_Year;
  unsigned char UTC_Month;
  unsigned char UTC_Day;
  unsigned char UTC_Hour;
  unsigned char UTC_Minute;
  unsigned char UTC_Second;
} UTC_Time_T;
#endif
typedef struct _PandoraPicHeader_s {
  char SOP[2];
  unsigned char pic_id;
  unsigned char type;
  unsigned int width;
  unsigned int height;
  unsigned timestamp;
  unsigned len;
  unsigned int totalLen;
  unsigned int position;
#ifdef UTC_TIME
  UTC_Time_T UTC_Time;
#endif
} PandoraPicHeader;

typedef struct _PandoraPic {
  PandoraPicHeader header;
  void* yuv;
} PandoraPic;

#ifdef UTC_TIME
#define PANDORA_CLIENT_HEADER_SIZE (34)
#else
#define PANDORA_CLIENT_HEADER_SIZE (28)
#endif

typedef int (*CallBack)(void* handle, int cmd, void* param, void* userp);

void* PandoraClientNew(const char* ip, int port, CallBack callback,
                       void* userp);
void PandoraClientDestroy(void* handle);

#ifdef __cplusplus
}
#endif

#endif  // SRC_PANDORA_CLIENT_H_
