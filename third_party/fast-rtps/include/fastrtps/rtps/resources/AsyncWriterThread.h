// Copyright 2016 Proyectos y Sistemas de Mantenimiento SL (eProsima).
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

/**
 * @file AsyncWriterThread.h
 *
 */
#ifndef _RTPS_RESOURCES_ASYNCWRITERTHREAD_H_
#define _RTPS_RESOURCES_ASYNCWRITERTHREAD_H_

#include <thread>
#include <atomic>
#include <mutex>
#include <condition_variable>
#include <list>

#include <fastrtps/rtps/resources/AsyncInterestTree.h>

namespace eprosima{
namespace fastrtps{
namespace rtps{
class RTPSWriter;

/**
 * @brief This static class owns a thread that manages asynchronous writes.
 * Asynchronous writes happen directly (when using an async writer) and
 * indirectly (when responding to a NACK).
 * @ingroup COMMON_MODULE
 */
class AsyncWriterThread
{
public:
    /**
     * @brief Adds a writer to be managed by this thread.
     * Only asynchronous writers are permitted.
     * @param writer Asynchronous writer to be added. 
     * @return Result of the operation.
     */
    static bool addWriter(RTPSWriter& writer);

    /**
     * @brief Removes a writer.
     * @param writer Asynchronous writer to be removed.
     * @return Result of the operation.
     */
    static bool removeWriter(RTPSWriter& writer);

    /**
     * Wakes the thread up.
     * @param interestedParticipant The participant interested in an async write.
     */
    static void wakeUp(const RTPSParticipantImpl* interestedParticipant);

    /**
     * Wakes the thread up.
     * @param interestedParticipant The writer interested in an async write.
     */
    static void wakeUp(const RTPSWriter* interestedWriter);

private:
    AsyncWriterThread() = delete;
    ~AsyncWriterThread() = delete;
    AsyncWriterThread(const AsyncWriterThread&) = delete;
    const AsyncWriterThread& operator=(const AsyncWriterThread&) = delete;

    //! @brief runs main method
    static void run();

    static std::thread* thread_;
    static std::mutex data_structure_mutex_;
    static std::mutex condition_variable_mutex_;
    
    //! List of asynchronous writers.
    static std::list<RTPSWriter*> async_writers;
    static AsyncInterestTree interestTree;

    static bool running_;
    static bool run_scheduled_;
    static std::condition_variable cv_;
};

} // namespace rtps
} // namespace fastrtps
} // namespace eprosima

#endif // _RTPS_RESOURCES_ASYNCWRITERTHREAD_H_
