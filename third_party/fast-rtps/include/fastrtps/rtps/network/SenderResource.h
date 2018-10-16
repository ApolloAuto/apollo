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

#ifndef SENDER_RESOURCE_H
#define SENDER_RESOURCE_H

#include <functional>
#include <vector>
#include <fastrtps/transport/TransportInterface.h>

namespace eprosima{
namespace fastrtps{
namespace rtps{

/**
 * RAII object that encapsulates the Send operation over one chanel in an unknown transport.
 * A Sender resource is always univocally associated to a transport channel; the
 * act of constructing a Sender Resource opens the channel and its destruction
 * closes it.
 * @ingroup NETWORK_MODULE
 */
class SenderResource 
{
   //! Only NetworkFactory is ever allowed to construct a SenderResource from scratch.
   //! In doing so, it guarantees the transport and channel are in a valid state for
   //! this resource to exist.
   friend class NetworkFactory;

public:
   /**
    * Sends to a destination locator, through the channel managed by this resource.
    * @param data Raw data slice to be sent.
    * @param dataLength Length of the data to be sent. Will be used as a boundary for
    * the previous parameter.
    * @param destinationLocator Locator describing the destination endpoint.
    * @return Success of the send operation.
    */
   bool Send(const octet* data, uint32_t dataLength, const Locator_t& destinationLocator);

   /** 
   * Reports whether this resource supports the given local locator (i.e., said locator
   * maps to the transport channel managed by this resource).
   */
   bool SupportsLocator(const Locator_t& local);
   /** 
   * Reports whether this resource supports the given remote locator (i.e., this resource
   * maps to a transport channel capable of sending to it).
   */
   bool CanSendToRemoteLocator(const Locator_t& remote);
   
   /**
   * Resources can only be transfered through move semantics. Copy, assignment, and 
   * construction outside of the factory are forbidden.
   */
   SenderResource(SenderResource&&);
   ~SenderResource();

private:
   SenderResource()                                 = delete;
   SenderResource(const SenderResource&)            = delete;
   SenderResource& operator=(const SenderResource&) = delete;

   SenderResource(TransportInterface&, Locator_t&);
   std::function<void()> Cleanup;
   std::function<bool(const octet* data, uint32_t dataLength, const Locator_t&)> SendThroughAssociatedChannel;
   std::function<bool(const Locator_t&)> LocatorMapsToManagedChannel;
   std::function<bool(const Locator_t&)> ManagedChannelMapsToRemote;
   bool mValid; // Post-construction validity check for the NetworkFactory
};

} // namespace rtps
} // namespace fastrtps
} // namespace eprosima

#endif
