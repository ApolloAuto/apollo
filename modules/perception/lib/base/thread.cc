#include "modules/perception/lib/base/thread.h"

#include <assert.h>
#include <signal.h>

#include "modules/common/log.h"

namespace apollo {
namespace perception {

void Thread::start() {
  pthread_attr_t attr;
  CHECK_EQ(pthread_attr_init(&attr), 0);
  CHECK_EQ(
      pthread_attr_setdetachstate(
          &attr, _joinable ? PTHREAD_CREATE_JOINABLE : PTHREAD_CREATE_DETACHED),
      0);
  CHECK_EQ(pthread_setcancelstate(PTHREAD_CANCEL_ENABLE, NULL), 0);
  CHECK_EQ(pthread_setcanceltype(PTHREAD_CANCEL_ASYNCHRONOUS, NULL), 0);

  int result = pthread_create(&_tid, &attr, &thread_runner, this);
  CHECK_EQ(result, 0) << "Could not create thread (" << result << ")";

  CHECK_EQ(pthread_attr_destroy(&attr), 0);

  _started = true;
}

void Thread::join() {
  CHECK(_joinable) << "Thread is not joinable";
  int result = pthread_join(_tid, NULL);
  CHECK_EQ(result, 0) << "Could not join thread (" << _tid << ", "
                      << _thread_name << ")";
  _tid = 0;
}

bool Thread::is_alive() {
  if (_tid == 0) {
    return false;
  }
  // no signal sent, just check existence for thread
  int ret = pthread_kill(_tid, 0);
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
