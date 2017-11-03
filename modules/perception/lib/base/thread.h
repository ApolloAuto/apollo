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
          : tid_(0), started_(false), joinable_(joinable), thread_name_(name) {
  }

  pthread_t Tid() const {
    return tid_;
  }

  void SetJoinable(bool joinable) {
    if (!started_) {
      joinable_ = joinable;
    }
  }

  void Start();

  void Join();

  bool IsAlive();

  std::string thread_name() const {
    return thread_name_;
  }
  void set_thread_name(const std::string& name) {
    thread_name_ = name;
  }

 protected:
  virtual void Run() = 0;

  static void *ThreadRunner(void *arg) {
    Thread* t = reinterpret_cast<Thread*>(arg);
    t->Run();
    return NULL;
  }

  pthread_t tid_;
  bool started_;
  bool joinable_;
  std::string thread_name_;

 private:
  DISALLOW_COPY_AND_ASSIGN(Thread);
};

}  // namespace perception
}  // namespace apollo

#endif  // apollo_PERCEPTION_LIB_BASE_THREAD_H
