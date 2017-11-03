#include "modules/perception/lib/base/thread.h"

#include <assert.h>
#include <signal.h>

#include "modules/common/log.h"

namespace apollo {
namespace perception {

void Thread::Start() {
  pthread_attr_t attr;
  CHECK_EQ(pthread_attr_init(&attr), 0);
  CHECK_EQ(
      pthread_attr_setdetachstate(
              &attr, joinable_ ? PTHREAD_CREATE_JOINABLE : PTHREAD_CREATE_DETACHED),
      0);
  CHECK_EQ(pthread_setcancelstate(PTHREAD_CANCEL_ENABLE, NULL), 0);
  CHECK_EQ(pthread_setcanceltype(PTHREAD_CANCEL_ASYNCHRONOUS, NULL), 0);

  int result = pthread_create(&tid_, &attr, &ThreadRunner, this);
  CHECK_EQ(result, 0) << "Could not create thread (" << result << ")";

  CHECK_EQ(pthread_attr_destroy(&attr), 0);

  started_ = true;
}

void Thread::Join() {
  CHECK(joinable_) << "Thread is not joinable";
  int result = pthread_join(tid_, NULL);
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

}  // namespace perception
}  // namespace apollo
