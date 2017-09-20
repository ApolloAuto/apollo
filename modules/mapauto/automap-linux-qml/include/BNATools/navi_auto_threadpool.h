

#ifndef THREADPOOL_H__
#define THREADPOOL_H_

#include <iostream>
#include <cstdio>
#include <cstdlib>
#include <unistd.h>
#include <pthread.h>
#include <deque>


using namespace std;

class NaviAdapterThreadPool {
public:
    // Event
    typedef struct _Event{
        void (*callbacks)(void*);        // Callback function
        void *args;                     // Arguments
    }Event;

    NaviAdapterThreadPool();
    ~NaviAdapterThreadPool();

    static NaviAdapterThreadPool* getInstance();
    /*
     *  Name: init
     *  Brief: Initialize the thread pool
     *  @thread_count: Number of thread, the default value is 4
     *  return: Success return true , else return false
     */
    bool init(const int thread_count=4);
    /*
     *  Name: destroy
     *  Brief: Destroy and free the thread pool after all events done
     *
     */
    void destroy();

    /*
     *  Name: add_event
     *  Brief: Add event to event buffer
     *  @ent: The event that include callback function and argument
     *  return: Success return true, else return false
     *
     */
    bool add_event(const NaviAdapterThreadPool::Event ent);

private:
//    ThreadPool* m_instance;
    bool m_is_init;                 // Determine that if the thread pool initialized
    pthread_mutex_t m_mutex;        // The mutex for thread-safe
    pthread_cond_t m_cond;          // The cond value
    deque<pthread_t> m_thrs;        // The threads id
    deque<Event> m_events;           // The events
    bool m_is_closing;              // When thread pool is closing

    /*
     *  Name: put_error
     *  Brief: Output error string
     *  @err: Error string
     *
     */
    void put_error(const string err);

    /*
     *  Name: thread_procee
     *  Brief: Threads wait here for event
     *  @m_this: Point to thread pool because this is static function
     *
     */
    static void* thread_process(void *m_this);
};

#endif
