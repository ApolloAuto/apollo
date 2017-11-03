#ifndef apollo_PERCEPTION_LIB_BASE_MUTEX_H
#define apollo_PERCEPTION_LIB_BASE_MUTEX_H

#include <pthread.h>

#include "modules/common/macro.h"

namespace apollo {
namespace perception {

class Mutex {
 public:
  Mutex() {
    pthread_mutex_init(&_mu, NULL);
  }

  ~Mutex() {
    pthread_mutex_destroy(&_mu);
  }

  inline void lock() {
    pthread_mutex_lock(&_mu);
  }

  inline void unlock() {
    pthread_mutex_unlock(&_mu);
  }

  inline bool try_lock() {
    return pthread_mutex_trylock(&_mu) == 0;
  }

 private:
  DISALLOW_COPY_AND_ASSIGN(Mutex);

  friend class CondVar;
  pthread_mutex_t _mu;
};

class MutexLock {
 public:
  explicit MutexLock(Mutex* mu) : _mu(mu) {
    _mu->lock();
  }
  ~MutexLock() {
    _mu->unlock();
  }

 private:
  Mutex* const _mu;
  DISALLOW_COPY_AND_ASSIGN(MutexLock);
};

// Wrapper for pthread_cond_t
class CondVar {
 public:
  CondVar() {
    pthread_cond_init(&_cv, NULL);
  }

  ~CondVar() {
    pthread_cond_destroy(&_cv);
  }

  void wait(Mutex* mu) {
    pthread_cond_wait(&_cv, &mu->_mu);
  }

  void signal() {
    pthread_cond_signal(&_cv);
  }

  void signalall() {
    pthread_cond_broadcast(&_cv);
  }

 private:
  DISALLOW_COPY_AND_ASSIGN(CondVar);

  pthread_cond_t _cv;
};

class BlockingCounter {
 public:
  explicit BlockingCounter(size_t cnt) : _counter(cnt) {}

  bool decrement() {
    MutexLock lock(&_mutex);
    --_counter;

    if (_counter == 0u) {
      _cond.signalall();
    }

    return _counter == 0u;
  }

  void reset(size_t cnt) {
    MutexLock lock(&_mutex);
    _counter = cnt;
  }

  void wait() {
    MutexLock lock(&_mutex);

    while (_counter != 0u) {
      _cond.wait(&_mutex);
    }
  }

 private:
  DISALLOW_COPY_AND_ASSIGN(BlockingCounter);
  Mutex _mutex;
  CondVar _cond;
  size_t _counter;
};

class RwMutex {
 public:
  RwMutex() {
    pthread_rwlock_init(&_mu, NULL);
  }
  ~RwMutex() {
    pthread_rwlock_destroy(&_mu);
  }

  inline void reader_lock() {
    pthread_rwlock_rdlock(&_mu);
  }
  inline void writer_lock() {
    pthread_rwlock_wrlock(&_mu);
  }

  inline void unlock() {
    pthread_rwlock_unlock(&_mu);
  }

 private:
  DISALLOW_COPY_AND_ASSIGN(RwMutex);
  pthread_rwlock_t _mu;
};

class ReaderMutexLock {
 public:
  explicit ReaderMutexLock(RwMutex* mu) : _mu(mu) {
    _mu->reader_lock();
  }
  ~ReaderMutexLock() {
    _mu->unlock();
  }

 private:
  DISALLOW_COPY_AND_ASSIGN(ReaderMutexLock);
  RwMutex* _mu = nullptr;
};

class WriterMutexLock {
 public:
  explicit WriterMutexLock(RwMutex* mu) : _mu(mu) {
    _mu->writer_lock();
  }
  ~WriterMutexLock() {
    _mu->unlock();
  }

 private:
  DISALLOW_COPY_AND_ASSIGN(WriterMutexLock);
  RwMutex* _mu = nullptr;
};

}  // namespace perception
}  // namespace apollo

#endif  // apollo_PERCEPTION_LIB_BASE_MUTEX_H
