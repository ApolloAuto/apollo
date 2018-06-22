/******************************************************************************
 * Copyright 2017 The Apollo Authors. All Rights Reserved.
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

#ifndef THREADPOOL_POOL_CORE_HPP_INCLUDED
#define THREADPOOL_POOL_CORE_HPP_INCLUDED

#include <condition_variable>
#include <deque>
#include <functional>
#include <memory>
#include <mutex>
#include <thread>
#include <vector>

#include "modules/common/log.h"
#include "modules/common/macro.h"

/// The namespace threadpool contains a thread pool and related utility classes.
namespace apollo {
namespace localization {

template <typename T, typename Mutex>
class LockingPtr {
 public:
  LockingPtr(volatile const T& obj, volatile const Mutex& mtx)
      : obj_(const_cast<T*>(&obj)), mutex_(*const_cast<Mutex*>(&mtx)) {
    mutex_.lock();
  }

  ~LockingPtr() { mutex_.unlock(); }

  T& operator*() const { return *obj_; }

  T* operator->() const { return obj_; }

 private:
  T* obj_;
  Mutex& mutex_;
  DISALLOW_COPY_AND_ASSIGN(LockingPtr);
};

class ScopeGuard {
 public:
  explicit ScopeGuard(std::function<void()> const& call_on_exit)
      : function_(call_on_exit), is_active_(true) {}

  ~ScopeGuard() {
    if (is_active_ && function_) {
      function_();
    }
  }

  void Disable() { is_active_ = false; }

 private:
  std::function<void()> const function_;
  bool is_active_;
  DISALLOW_COPY_AND_ASSIGN(ScopeGuard);
};

class FifoScheduler {
 public:
  typedef std::function<void()>
      TaskType;  //!< Indicates the scheduler's task type.

 public:
  bool Push(TaskType const& task) {
    task_queue_.push_back(task);
    return true;
  }

  void Pop() { task_queue_.pop_front(); }

  TaskType const& Top() const { return task_queue_.front(); }

  size_t Size() const { return task_queue_.size(); }

  bool Empty() const { return task_queue_.empty(); }

  void Clear() { task_queue_.clear(); }

 protected:
  std::deque<TaskType> task_queue_;
};

template <typename ThreadPool>
class WorkerThread
    : public std::enable_shared_from_this<WorkerThread<ThreadPool>> {
 public:
  explicit WorkerThread(std::shared_ptr<ThreadPool> const& pool) : pool_(pool) {
    DCHECK(pool);
  }

  void DiedUnexpectedly() {
    pool_->WorkerDiedUnexpectedly(this->shared_from_this());
  }

 public:
  void run() {
    ScopeGuard notify_exception(bind(&WorkerThread::DiedUnexpectedly, this));

    while (pool_->execute_task()) {
    }

    notify_exception.Disable();
    pool_->worker_destructed(this->shared_from_this());
  }

  void join() { thread_->join(); }

  static void create_and_attach(std::shared_ptr<ThreadPool> const& pool) {
    std::shared_ptr<WorkerThread> worker(new WorkerThread(pool));
    if (worker) {
      worker->thread_.reset(
          new std::thread(std::bind(&WorkerThread::run, worker)));
    }
  }

 private:
  std::shared_ptr<ThreadPool>
      pool_;  //!< Pointer to the pool which created the worker.
  std::shared_ptr<std::thread>
      thread_;  //!< Pointer to the thread which executes the run loop.
};

class ThreadPool;
class ThreadPoolImpl : public std::enable_shared_from_this<ThreadPoolImpl> {
 public:  // Type definitions
  typedef std::function<void()> TaskType;
  typedef FifoScheduler TaskQueueType;
  typedef ThreadPoolImpl PoolType;
  typedef WorkerThread<PoolType> WorkerType;

 private:
  friend class WorkerThread<PoolType>;
  friend class ThreadPool;

 public:
  /// Constructor.
  ThreadPoolImpl()
      : terminate_all_workers_(false),
        worker_count_(0),
        target_worker_count_(0),
        active_worker_count_(0) {
    m_scheduler.Clear();
  }

  /// Destructor.
  ~ThreadPoolImpl() {}

  /*! Gets the number of threads in the pool.
   * \return The number of threads.
   */
  size_t Size() const volatile { return worker_count_; }

  // is only called once
  void shutdown() {
    this->wait();
    this->terminate_all_workers(true);
  }

  /*! Schedules a task for asynchronous execution. The task will be executed
   * once only. \param task The task function object. It should not throw
   * execeptions. \return true, if the task could be scheduled and false
   * otherwise.
   */
  bool schedule(TaskType const& task) volatile {
    LockingPtr<PoolType, std::recursive_mutex> locked_this(*this, monitor_);

    if (locked_this->m_scheduler.Push(task)) {
      // std::cerr << "push task to scheduler" << std::endl;
      locked_this->task_or_terminate_workers_event_.notify_one();
      return true;
    } else {
      return false;
    }
  }

  /*! Returns the number of tasks which are currently executed.
   * \return The number of active tasks.
   */
  size_t active() const volatile { return active_worker_count_; }

  /*! Returns the number of tasks which are ready for execution.
   * \return The number of pending tasks.
   */
  size_t pending() const volatile {
    LockingPtr<const PoolType, std::recursive_mutex> locked_this(*this,
                                                                 monitor_);
    return locked_this->m_scheduler.Size();
  }

  /*! Removes all pending tasks from the pool's scheduler.
   */
  void Clear() volatile {
    LockingPtr<PoolType, std::recursive_mutex> locked_this(*this, monitor_);
    locked_this->m_scheduler.Clear();
  }

  /*! Indicates that there are no tasks pending.
   * \return true if there are no tasks ready for execution.
   * \remarks This function is more efficient that the check 'pending() == 0'.
   */
  bool Empty() const volatile {
    LockingPtr<const PoolType, std::recursive_mutex> locked_this(*this,
                                                                 monitor_);
    return locked_this->m_scheduler.Empty();
  }

  /*! The current thread of execution is blocked until the sum of all active
   *  and pending tasks is equal or less than a given threshold.
   * \param task_threshold The maximum number of tasks in pool and scheduler.
   */
  void wait(size_t const task_threshold = 0) const volatile {
    const PoolType* self = const_cast<const PoolType*>(this);
    // std::recursive_mutex::scoped_lock lock(self->monitor_);
    std::unique_lock<std::recursive_mutex> lock(self->monitor_);

    if (0 == task_threshold) {
      while (0 != self->active_worker_count_ || !self->m_scheduler.Empty()) {
        self->worker_idle_or_terminated_event_.wait(lock);
      }
    } else {
      while (task_threshold <
             self->active_worker_count_ + self->m_scheduler.Size()) {
        self->worker_idle_or_terminated_event_.wait(lock);
      }
    }
  }

 private:
  void terminate_all_workers(bool const wait) volatile {
    PoolType* self = const_cast<PoolType*>(this);
    // std::recursive_mutex::scoped_lock lock(self->monitor_);
    std::unique_lock<std::recursive_mutex> lock(self->monitor_);

    self->terminate_all_workers_ = true;

    target_worker_count_ = 0;
    self->task_or_terminate_workers_event_.notify_all();

    if (wait) {
      while (worker_count_ > 0) {
        self->worker_idle_or_terminated_event_.wait(lock);
      }

      for (typename std::vector<std::shared_ptr<WorkerType>>::iterator it =
               self->terminated_workers_.begin();
           it != self->terminated_workers_.end(); ++it) {
        (*it)->join();
      }
      self->terminated_workers_.clear();
    }
  }

  /*! Changes the number of worker threads in the pool. The resizing
   *  is handled by the SizePolicy.
   * \param threads The new number of worker threads.
   * \return true, if pool will be resized and false if not.
   */
  bool resize(size_t const worker_count) volatile {
    LockingPtr<PoolType, std::recursive_mutex> locked_this(*this, monitor_);

    if (!terminate_all_workers_) {
      target_worker_count_ = worker_count;
    } else {
      return false;
    }

    if (worker_count_ <= target_worker_count_) {  // increase worker count
      while (worker_count_ < target_worker_count_) {
        try {
          WorkerThread<PoolType>::create_and_attach(
              locked_this->shared_from_this());
          worker_count_++;
          active_worker_count_++;
        } catch (...) {
          return false;
        }
      }
    } else {  // decrease worker count
      locked_this->task_or_terminate_workers_event_
          .notify_all();  // TODO(Apollo): Optimize number of notified workers
    }

    return true;
  }

  // worker died with unhandled exception
  void WorkerDiedUnexpectedly(std::shared_ptr<WorkerType> worker) volatile {
    LockingPtr<PoolType, std::recursive_mutex> locked_this(*this, monitor_);

    worker_count_--;
    active_worker_count_--;
    locked_this->worker_idle_or_terminated_event_.notify_all();

    if (terminate_all_workers_) {
      locked_this->terminated_workers_.push_back(worker);
    } else {
      // locked_this->m_size_policy->WorkerDiedUnexpectedly(worker_count_);
      resize(worker_count_ + 1);
    }
  }

  void worker_destructed(std::shared_ptr<WorkerType> worker) volatile {
    LockingPtr<PoolType, std::recursive_mutex> locked_this(*this, monitor_);
    worker_count_--;
    active_worker_count_--;
    locked_this->worker_idle_or_terminated_event_.notify_all();

    if (terminate_all_workers_) {
      locked_this->terminated_workers_.push_back(worker);
    }
  }

  bool execute_task() volatile {
    std::function<void()> task;

    {  // fetch task
      PoolType* locked_this = const_cast<PoolType*>(this);
      // std::recursive_mutex::scoped_lock lock(locked_this->monitor_);
      std::unique_lock<std::recursive_mutex> lock(locked_this->monitor_);

      // std::cerr << "execute_task" << std::endl;

      // decrease number of threads if necessary
      if (worker_count_ > target_worker_count_) {
        return false;  // terminate worker
      }

      // wait for tasks
      while (locked_this->m_scheduler.Empty()) {
        // decrease number of workers if necessary
        if (worker_count_ > target_worker_count_) {
          return false;  // terminate worker
        } else {
          active_worker_count_--;
          locked_this->worker_idle_or_terminated_event_.notify_all();
          locked_this->task_or_terminate_workers_event_.wait(lock);
          active_worker_count_++;
          // std::cerr << "active_worker_count_ ++" << std::endl;
        }
      }

      task = locked_this->m_scheduler.Top();
      locked_this->m_scheduler.Pop();
    }

    // call task function
    if (task) {
      task();
    }

    // guard->Disable();
    return true;
  }

 private:  // The following members are accessed only by _one_ thread at the
           // same time:
  TaskQueueType m_scheduler;

  bool terminate_all_workers_;
  std::vector<std::shared_ptr<WorkerType>> terminated_workers_;  // List of
                                                                 // workers
                                                                 // which are
                                                                 // terminated
                                                                 // but not
                                                                 // fully
                                                                 // destructed.

 private:  // The following members are implemented thread-safe:
  mutable std::recursive_mutex monitor_;
  mutable std::condition_variable_any
      worker_idle_or_terminated_event_;  // A worker is idle or was terminated.
  mutable std::condition_variable_any
      task_or_terminate_workers_event_;  // Task is available OR total worker
                                         // count should be reduced.

 private:  // The following members may be accessed by _multiple_ threads at the
           // same time:
  volatile size_t worker_count_;
  volatile size_t target_worker_count_;
  volatile size_t active_worker_count_;
  DISALLOW_COPY_AND_ASSIGN(ThreadPoolImpl);
};

class ThreadPool {
  typedef std::function<void()> TaskType;

 public:
  explicit ThreadPool(size_t initial_threads = 0)
      : threadpool_impl_(new ThreadPoolImpl()) {
    bool suc = threadpool_impl_->resize(initial_threads);
    if (suc == false) {
      std::cerr << "ThreadPool can't resize thread num." << std::endl;
    }
  }

  ~ThreadPool() { Shutdown(); }

  size_t Size() const { return threadpool_impl_->Size(); }

  void Shutdown() {
    threadpool_impl_->shutdown();
    return;
  }

  bool schedule(TaskType const& task) {
    return threadpool_impl_->schedule(task);
  }

  size_t active() const { return threadpool_impl_->active(); }

  size_t pending() const { return threadpool_impl_->pending(); }

  void Clear() { return threadpool_impl_->Clear(); }

  bool Empty() const { return threadpool_impl_->Empty(); }

  void wait(size_t const task_threshold = 0) const {
    threadpool_impl_->wait(task_threshold);
    return;
  }

 private:
  std::shared_ptr<ThreadPoolImpl> threadpool_impl_;

  DISALLOW_COPY_AND_ASSIGN(ThreadPool);
};

}  // namespace localization
}  // namespace apollo

#endif  // THREADPOOL_POOL_CORE_HPP_INCLUDED
