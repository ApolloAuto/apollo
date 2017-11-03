#ifndef apollo_PERCEPTION_LIB_BASE_THREAD_H
#define apollo_PERCEPTION_LIB_BASE_THREAD_H

#include <pthread.h>
#include <string>

#include "modules/common/macro.h"

namespace apollo {
namespace perception {

class Thread {
 public:
  Thread(bool joinable = false, const std::string& name = "Thread")
      : _tid(0), _started(false), _joinable(joinable), _thread_name(name) {}

  pthread_t tid() const {
    return _tid;
  }

  void set_joinable(bool joinable) {
    if (!_started) {
      _joinable = joinable;
    }
  }

  void start();

  void join();

  bool is_alive();

  std::string get_thread_name() const {
    return _thread_name;
  }
  void set_thread_name(const std::string& name) {
    _thread_name = name;
  }

 protected:
  virtual void run() = 0;

  static void* thread_runner(void* arg) {
    Thread* t = reinterpret_cast<Thread*>(arg);
    t->run();
    return NULL;
  }

  pthread_t _tid;
  bool _started;
  bool _joinable;
  std::string _thread_name;

 private:
  DISALLOW_COPY_AND_ASSIGN(Thread);
};

}  // namespace perception
}  // namespace apollo

#endif  // apollo_PERCEPTION_LIB_BASE_THREAD_H
