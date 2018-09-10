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

#ifndef RECEIVER_RESOURCE_H
#define RECEIVER_RESOURCE_H

#include <functional>
#include <vector>
#include <fastrtps/transport/TransportInterface.h>

namespace eprosima{
namespace fastrtps{
namespace rtps{

/**
 * RAII object that encapsulates the Receive operation over one chanel in an unknown transport.
 * A Receiver resource is always univocally associated to a transport channel; the
 * act of constructing a Receiver Resource opens the channel and its destruction
 * closes it.
 * @ingroup NETWORK_MODULE
 */
class ReceiverResource 
{
   //! Only NetworkFactory is ever allowed to construct a ReceiverResource from scratch.
   //! In doing so, it guarantees the transport and channel are in a valid state for
   //! this resource to exist.
friend class NetworkFactory;

public:
  /**
   * Performs a blocking receive through the channel managed by this resource,
   * notifying about the origin locator.
   * @param receiveBuffer Pointer to a buffer where to store the received message.
   * @param receiveBufferCapacity Capacity of the reception buffer.
   * Will be used as a boundary check for the previous parameter.
   * @param[out] receiveBufferSize Final size of the received message.
   * @param[out] originLocator Address of the remote sender.
   * @return Success of the managed Receive operation.
   */
   bool Receive(octet* receiveBuffer, uint32_t receiveBufferCapacity, uint32_t& receiveBufferSize,
                Locator_t& originLocator);

  /**
   * Reports whether this resource supports the given local locator (i.e., said locator
   * maps to the transport channel managed by this resource).
   */
   bool SupportsLocator(const Locator_t& localLocator);

   /**
    * Aborts a blocking receive (thread safe).
    */
   void Abort();

   /**
    * Resources can only be transfered through move semantics. Copy, assignment, and 
    * construction outside of the factory are forbidden.
    */
   ReceiverResource(ReceiverResource&&);
   ~ReceiverResource();

private:
   ReceiverResource()                                   = delete;
   ReceiverResource(const ReceiverResource&)            = delete;
   ReceiverResource& operator=(const ReceiverResource&) = delete;

   ReceiverResource(TransportInterface&, const Locator_t&);
   std::function<void()> Cleanup;
   std::function<bool(octet*, uint32_t, uint32_t&, Locator_t&)> ReceiveFromAssociatedChannel;
   std::function<bool(const Locator_t&)> LocatorMapsToManagedChannel;
   bool mValid; // Post-construction validity check for the NetworkFactory
};

} // namespace rtps
} // namespace fastrtps
} // namespace eprosima

#endif
