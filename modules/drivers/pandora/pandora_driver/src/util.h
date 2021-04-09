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

#ifndef SRC_UTIL_H_
#define SRC_UTIL_H_

#ifdef __cplusplus
extern "C" {
#endif

int sys_readn(int fd, void* vptr, int n);
int sys_writen(int fd, const void* vptr, int n);
int tcp_open(const char* ipaddr, int port);
int select_fd(int fd, int timeout, int wait_for);

enum { WAIT_FOR_READ, WAIT_FOR_WRITE, WAIT_FOR_CONN };

#ifdef __cplusplus
}
#endif

#endif  //  SRC_UTIL_H_
