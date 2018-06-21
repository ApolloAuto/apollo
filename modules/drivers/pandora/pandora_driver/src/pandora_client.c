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

#include "src/pandora_client.h"
#include "src/util.h"

typedef struct _PandoraClient_s {
  pthread_mutex_t cliSocketLock;
  int cliSocket;

  pthread_t receiveTask;
  pthread_t heartBeatTask;

  int exit;
  char* ip;
  int port;

  CallBack callback;
  void* userp;

  unsigned int position[PANDORA_CAMERA_UNIT];
  unsigned int startTimestamp[PANDORA_CAMERA_UNIT];
  PandoraPic* pics[PANDORA_CAMERA_UNIT];
} PandoraClient;

void PandoraClientTask(void* handle);
void PandoraClientHeartBeatTask(void* handle);
void parseHeader(char* header, int len, PandoraPicHeader* picHeader) {
  int index = 0;
  picHeader->SOP[0] = header[index];
  picHeader->SOP[1] = header[index + 1];
  index += 2;

  picHeader->pic_id = header[index];
  picHeader->type = header[index + 1];
  index += 2;

  picHeader->width =
      (header[index + 0] & 0xff) << 24 | (header[index + 1] & 0xff) << 16 |
      (header[index + 2] & 0xff) << 8 | (header[index + 3] & 0xff) << 0;
  index += 4;

  picHeader->height =
      (header[index + 0] & 0xff) << 24 | (header[index + 1] & 0xff) << 16 |
      (header[index + 2] & 0xff) << 8 | (header[index + 3] & 0xff) << 0;
  index += 4;

  picHeader->timestamp =
      (header[index + 0] & 0xff) << 24 | (header[index + 1] & 0xff) << 16 |
      (header[index + 2] & 0xff) << 8 | (header[index + 3] & 0xff) << 0;
  index += 4;

  picHeader->len =
      (header[index + 0] & 0xff) << 24 | (header[index + 1] & 0xff) << 16 |
      (header[index + 2] & 0xff) << 8 | (header[index + 3] & 0xff) << 0;
  index += 4;

  picHeader->totalLen =
      (header[index + 0] & 0xff) << 24 | (header[index + 1] & 0xff) << 16 |
      (header[index + 2] & 0xff) << 8 | (header[index + 3] & 0xff) << 0;

  index += 4;

  picHeader->position =
      (header[index + 0] & 0xff) << 24 | (header[index + 1] & 0xff) << 16 |
      (header[index + 2] & 0xff) << 8 | (header[index + 3] & 0xff) << 0;
#ifdef UTC_TIME
  index += 4;
  picHeader->UTC_Time.UTC_Year = header[index++];
  picHeader->UTC_Time.UTC_Month = header[index++];
  picHeader->UTC_Time.UTC_Day = header[index++];
  picHeader->UTC_Time.UTC_Hour = header[index++];
  picHeader->UTC_Time.UTC_Minute = header[index++];
  picHeader->UTC_Time.UTC_Second = header[index];
#endif
}
void* PandoraClientNew(const char* ip, int port, CallBack callback,
                       void* userp) {
  if (!ip || !callback || !userp) {
    printf("Bad Parameter\n");
    return NULL;
  }

  int ret = 0;

  PandoraClient* client = (PandoraClient*)malloc(sizeof(PandoraClient));
  if (!client) {
    printf("No Memory\n");
    return NULL;
  }

  memset(client, 0, sizeof(PandoraClient));
  client->callback = callback;
  client->userp = userp;
  client->cliSocket = -1;
  client->ip = strdup(ip);
  client->port = port;

  pthread_mutex_init(&client->cliSocketLock, NULL);

  ret = pthread_create(&client->receiveTask, NULL, (void*)PandoraClientTask,
                       (void*)client);
  if (ret != 0) {
    printf("Create Task Failed\n");
    free(client);
    return NULL;
  }

  ret = pthread_create(&client->heartBeatTask, NULL,
                       (void*)PandoraClientHeartBeatTask, (void*)client);
  if (ret != 0) {
    printf("Create heart beat Task Failed\n");
    client->exit = 1;
    pthread_join(client->receiveTask, NULL);
    free(client);
    return NULL;
  }
  return (void*)client;
}

void PandoraClientDestroy(void* handle) {
  PandoraClient* client = (PandoraClient*)handle;
  if (!client) {
    printf("Bad Parameter\n");
    return;
  }

  client->exit = 1;
  pthread_join(client->heartBeatTask, NULL);
  pthread_join(client->receiveTask, NULL);
  close(client->cliSocket);
  free(client);
}

void PandoraClientHeartBeatTask(void* handle) {
  PandoraClient* client = (PandoraClient*)handle;
  if (!client) {
    printf("Bad Parameter\n");
    return;
  }
  int ret = 0;

  while (!client->exit) {
    if (-1 == client->cliSocket) {
      sleep(1);
      continue;
    }
    ret = select_fd(client->cliSocket, 1, WAIT_FOR_WRITE);
    if (ret > 0) {
      ret = write(client->cliSocket, "HEARTBEAT", strlen("HEARTBEAT"));
      if (ret < 0) {
        printf("Write Error\n");
      }
    }

    sleep(1);
  }
}

void PandoraClientTask(void* handle) {
  PandoraClient* client = (PandoraClient*)handle;
  if (!client) {
    printf("Bad Parameter\n");
    return;
  }

  int connfd = client->cliSocket;

  while (!client->exit) {
    if (client->cliSocket == -1) {
      printf("Camera: Connecting......\n");
      connfd = tcp_open(client->ip, client->port);
      if (connfd < 0) {
        printf("Connect to server failed\n");
        sleep(1);
        continue;
      }
      pthread_mutex_lock(&client->cliSocketLock);
      client->cliSocket = connfd;
      pthread_mutex_unlock(&client->cliSocketLock);
      printf("Camera: connect to server successfully!\n");
      struct timeval tv;
      tv.tv_sec = 5;   /* 5 Secs Timeout */
      tv.tv_usec = 0;  // Not init'ing this can cause strange errors
      setsockopt(client->cliSocket, SOL_SOCKET, SO_RCVTIMEO, (const char*)&tv,
                 sizeof(struct timeval));
      connfd = client->cliSocket;
    }

    int ret = 0;
    // printf("start select_fd for read\n");
    ret = select_fd(connfd, 1, WAIT_FOR_READ);
    if (ret == 0) {
      printf("select for read wrong\n");
      pthread_mutex_lock(&client->cliSocketLock);
      close(client->cliSocket);
      client->cliSocket = -1;
      pthread_mutex_unlock(&client->cliSocketLock);
      sleep(5);
      continue;
    } else if (ret > 0) {
      char header[64];
      int n = sys_readn(connfd, header, 2);
      if (n < 0) {
        printf("read header wrong!\n");
        pthread_mutex_lock(&client->cliSocketLock);
        close(client->cliSocket);
        client->cliSocket = -1;
        pthread_mutex_unlock(&client->cliSocketLock);
        continue;
      }

      if (header[0] != 0x47 || header[1] != 0x74) {
        printf("InValid Header SOP\n");
        printf("%02x %02x \n", header[0], header[1]);
        exit(0);
        continue;
      }

      n = sys_readn(connfd, header + 2, PANDORA_CLIENT_HEADER_SIZE - 2);
      if (n <= 0) {
        printf("read header2 wrong!\n");
        pthread_mutex_lock(&client->cliSocketLock);
        close(client->cliSocket);
        client->cliSocket = -1;
        pthread_mutex_unlock(&client->cliSocketLock);
        continue;
      }
      PandoraPic* pic = (PandoraPic*)malloc(sizeof(PandoraPic));
      if (!pic) {
        printf("No Memory\n");
        continue;
      }

      parseHeader(header, n + 2, &pic->header);

      pic->yuv = malloc(pic->header.len);
      if (!pic->yuv) {
        printf("No Memory\n");
        free(pic);
        continue;
      }

      n = sys_readn(connfd, pic->yuv, pic->header.len);
      if (n <= 0) {
        printf("read pic_yuv wrong!\n");
        pthread_mutex_lock(&client->cliSocketLock);
        close(client->cliSocket);
        client->cliSocket = -1;
        pthread_mutex_unlock(&client->cliSocketLock);
        continue;
      }
      if (n != pic->header.len) {
        printf("picLength wrong\n");
        free(pic->yuv);
        free(pic);
        continue;
      }

      // check frame id
      if (pic->header.pic_id < 0 || pic->header.pic_id >= PANDORA_CAMERA_UNIT) {
        free(pic->yuv);
        free(pic);
        continue;
      }

      // check the desired frame postion packet.
      if (pic->header.position != client->position[pic->header.pic_id]) {
        client->position[pic->header.pic_id] = 0;
        free(pic->yuv);
        free(pic);
        continue;
      }

      if (!client->pics[pic->header.pic_id]) {
        client->pics[pic->header.pic_id] = malloc(sizeof(PandoraPic));
        memcpy(client->pics[pic->header.pic_id], pic, sizeof(PandoraPic));
        client->pics[pic->header.pic_id]->yuv = malloc(pic->header.totalLen);
      }

      memcpy(client->pics[pic->header.pic_id]->yuv +
                 client->position[pic->header.pic_id],
             pic->yuv, pic->header.len);

      if (client->position[pic->header.pic_id] == 0) {
        client->startTimestamp[pic->header.pic_id] = pic->header.timestamp;
      }

      client->position[pic->header.pic_id] += pic->header.len;

      if (client->position[pic->header.pic_id] == pic->header.totalLen) {
        client->pics[pic->header.pic_id]->header.len = pic->header.totalLen;
        client->pics[pic->header.pic_id]->header.timestamp =
            pic->header.timestamp;
        client->pics[pic->header.pic_id]->header.UTC_Time =
            pic->header.UTC_Time;
        // a whole frame
        if (client->callback)
          client->callback(client, 0, client->pics[pic->header.pic_id],
                           client->userp);
        client->pics[pic->header.pic_id] = NULL;
        client->position[pic->header.pic_id] = 0;
      }
      free(pic->yuv);
      free(pic);

    } else {
      printf("Read Error\n");
      pthread_mutex_lock(&client->cliSocketLock);
      close(client->cliSocket);
      client->cliSocket = -1;
      pthread_mutex_unlock(&client->cliSocketLock);
      continue;
    }
  }
}
