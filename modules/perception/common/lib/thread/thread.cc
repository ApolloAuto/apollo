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
#include "modules/perception/common/lib/thread/thread.h"

#include <csignal>

#include "cyber/common/log.h"

namespace apollo {
namespace perception {
namespace lib {

void Thread::Start() {
  pthread_attr_t attr;
  CHECK_EQ(pthread_attr_init(&attr), 0);
  CHECK_EQ(
      pthread_attr_setdetachstate(
          &attr, joinable_ ? PTHREAD_CREATE_JOINABLE : PTHREAD_CREATE_DETACHED),
      0);
  CHECK_EQ(pthread_setcancelstate(PTHREAD_CANCEL_ENABLE, nullptr), 0);
  CHECK_EQ(pthread_setcanceltype(PTHREAD_CANCEL_ASYNCHRONOUS, nullptr), 0);

  int result = pthread_create(&tid_, &attr, &ThreadRunner, this);
  CHECK_EQ(result, 0) << "Could not create thread (" << result << ")";

  CHECK_EQ(pthread_attr_destroy(&attr), 0);

  started_ = true;
}

void Thread::Join() {
  ACHECK(joinable_) << "Thread is not joinable";
  int result = pthread_join(tid_, nullptr);
  CHECK_EQ(result, 0) << "Could not join thread (" << tid_ << ", "
                      << thread_name_ << ")";
  tid_ = 0;
}

bool Thread::IsAlive() {
  if (tid_ == 0) {
    return false;
  }
  // no signal sent, just check existence for thread
  int ret = pthread_kill(tid_, 0);
  if (ret == ESRCH) {
    return false;
  }
  if (ret == EINVAL) {
    AWARN << "Invalid singal.";
    return false;
  }

  return true;
}

}  // namespace lib
}  // namespace perception
}  // namespace apollo
