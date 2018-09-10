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

#ifndef NETWORK_FACTORY_HPP
#define NETWORK_FACTORY_HPP

#include <fastrtps/transport/TransportInterface.h>
#include <fastrtps/rtps/network/ReceiverResource.h>
#include <fastrtps/rtps/network/SenderResource.h>
#include <vector>
#include <memory>

namespace eprosima{
namespace fastrtps{
namespace rtps{
/**
 * Provides the FastRTPS library with abstract resources, which
 * in turn manage the SEND and RECEIVE operations over some transport.
 * Once a transport is registered, it becomes invisible to the library
 * and is abstracted away for good.
 * @ingroup NETWORK_MODULE.
 */
class NetworkFactory
{
    public:

        NetworkFactory();

        /**
         * Allows registration of a transport statically, by specifying the transport type and
         * its associated descriptor type. This is particularly useful for user-defined transports.
         */
        template<class T, class D>
            void RegisterTransport(const D& descriptor)
            {
                std::unique_ptr<T> transport(new T(descriptor));
                if(transport->init())
                    mRegisteredTransports.emplace_back(std::move(transport));
            }

        /**
         * Allows registration of a transport dynamically. Only the transports built into FastRTPS
         * are supported here (although it can be easily extended at NetworkFactory.cpp)
         * @param descriptor Structure that defines all initial configuration for a given transport.
         */
        void RegisterTransport(const TransportDescriptorInterface* descriptor);

        /**
         * Walks over the list of transports, opening every possible channel that can send through 
         * the given locator and returning a vector of Sender Resources associated with it.
         * @param local Locator through which to send.
         */
        std::vector<SenderResource>   BuildSenderResources                 (Locator_t& local);
        /**
         * Walks over the list of transports, opening every possible channel that can send to the 
         * given remote locator and returning a vector of Sender Resources associated with it.
         * @param local Destination locator that we intend to send to.
         */
        std::vector<SenderResource>   BuildSenderResourcesForRemoteLocator (const Locator_t& remote);
        /**
         * Walks over the list of transports, opening every possible channel that we can listen to
         * from the given locator, and returns a vector of Receiver Resources for this goal.
         * @param local Locator from which to listen.
         */
        bool BuildReceiverResources (const Locator_t& local, std::vector<ReceiverResource>& returned_resources_list);

        void NormalizeLocators(LocatorList_t& locators);

        LocatorList_t ShrinkLocatorLists(const std::vector<LocatorList_t>& locatorLists);

        bool is_local_locator(const Locator_t& locator) const;

        size_t numberOfRegisteredTransports() const;

        uint32_t get_max_message_size_between_transports() { return maxMessageSizeBetweenTransports_; }

        uint32_t get_min_send_buffer_size() { return minSendBufferSize_; }

    private:

        std::vector<std::unique_ptr<TransportInterface> > mRegisteredTransports;

        uint32_t maxMessageSizeBetweenTransports_;

        uint32_t minSendBufferSize_;
};

} // namespace rtps
} // namespace fastrtps
} // namespace eprosima

#endif
