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

#ifndef MODULES_MONITOR_SYSMON_QUERY_RUNNER_H_
#define MODULES_MONITOR_SYSMON_QUERY_RUNNER_H_

#include <functional>
#include <list>
#include <mutex>
#include <thread>

#include "defs.h"
#include "periodic_task.h"


/**
 * @namespace apollo::monitor::sysmon
 * @brief apollo::monitor::sysmon
 */
namespace apollo {
namespace monitor {
namespace sysmon {

typedef std::function<int()> QueryTask;

class PeriodicSingleQuery: public PeriodicTask {
public:
  PeriodicSingleQuery(uint64_t interval_ns, QueryTask task_fn)
      : PeriodicTask(interval_ns), _task_fn(task_fn) {}

  virtual ~PeriodicSingleQuery() {}

  inline void add_task(QueryTask tsk) { _task_fn = tsk; }

  inline bool has_task() { return true; }

protected:
  QueryTask _task_fn;

  void do_task() final { _task_fn(); }

private:
  DISALLOW_COPY_AND_ASSIGN(PeriodicSingleQuery);
};

/// Class that runs a given set of query tasks periodically.
class PeriodicQueryRunner : public PeriodicTask {
public:
  PeriodicQueryRunner(uint64_t interval_ns, std::list<QueryTask> tasks)
      : PeriodicTask(interval_ns), _tasks(std::move(tasks)) {}

  virtual ~PeriodicQueryRunner() {}

  inline void add_task(QueryTask tsk) { _tasks.push_back(tsk); }

  inline bool has_task() { return _tasks.size() > 0; }

protected:
  void do_task() final;

private:
  std::list<QueryTask> _tasks;

  DISALLOW_COPY_AND_ASSIGN(PeriodicQueryRunner);
};

class QueryThread {
public:
  virtual ~QueryThread() { if (_thrd.get()) { _thrd->join(); } }

  virtual void start()=0;
  virtual void stop()=0;

  virtual void wait() {
    if (_thrd.get()) {
      _thrd->join();
      _thrd.reset(nullptr);
    }
  }

  virtual void stop_and_wait() {
    stop();
    wait();
  }

  virtual void add_task(QueryTask) {}

protected:
  std::unique_ptr<std::thread> _thrd;
};

template <class RunnerCl=PeriodicQueryRunner, class TaskCl=std::list<QueryTask>>
class PeriodicQueryThread : public QueryThread {
public:
  PeriodicQueryThread(uint64_t interval_ns, TaskCl task)
      : _query_runner(interval_ns, task) {}

  virtual ~PeriodicQueryThread() { stop(); }

  /// Starts thread to do the queries, doesn't block.
  inline void start() override {
    if (_query_runner.has_task()) {
      _thrd.reset(new std::thread(_query_runner.get_runner()));
    }
  }

  /// Stops thread, no block -- thread will finish sometime later.
  inline void stop() override {
    _query_runner.stop();
  }

  inline void add_task(QueryTask tsk) override { _query_runner.add_task(tsk); }

private:
  RunnerCl _query_runner;

  DISALLOW_COPY_AND_ASSIGN(PeriodicQueryThread);
};

/// A thread that runs a set of query tasks.
/// Usage:
/// @code{.cc}
/// QueryOnceThread thrd(std::move(tasks));
/// // Spawn a thread to do queries, calling thread doesn't block.
/// thrd.run();
/// // Do your other stuff.
/// // Now wait for thread to finish.
/// thrd.wait();
class QueryOnceThread : public QueryThread {
public:
  explicit QueryOnceThread(std::list<QueryTask> tasks) :  _tasks(tasks) {}

  explicit QueryOnceThread(QueryTask tsk) { _tasks.emplace_back(tsk); }

  virtual ~QueryOnceThread() { }

  /// Starts thread to do the queries, doesn't block.
  void start() override;

  inline void stop() override { }

private:
  std::list<QueryTask> _tasks;

  void do_tasks();

  DISALLOW_COPY_AND_ASSIGN(QueryOnceThread);
};


/// Class that runs a given set of query tasks periodically;
/// a task is removed if returns >= 0 (indicating it is done).
class UntilSuccessQueryRunner : public PeriodicTask {
public:
  UntilSuccessQueryRunner(uint64_t interval_ns, std::list<QueryTask> tasks)
      : PeriodicTask(interval_ns), _tasks(std::move(tasks)) {}

  UntilSuccessQueryRunner(uint64_t interval_ns, QueryTask tsk)
      : PeriodicTask(interval_ns) { _tasks.emplace_back(tsk); }

  virtual ~UntilSuccessQueryRunner() {}

  inline void add_task(QueryTask tsk) { _tasks.push_back(tsk); }

  inline bool has_task() { return _tasks.size() > 0; }

protected:
  void do_task() final;

private:
  std::list<QueryTask> _tasks;

  DISALLOW_COPY_AND_ASSIGN(UntilSuccessQueryRunner);
};

}  // namespace sysmon
}  // namespace monitor
}  // namespace apollo

#endif  // MODULES_MONITOR_SYSMON_QUERY_RUNNER_H_
