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

#ifndef UDPV4_TRANSPORT_H
#define UDPV4_TRANSPORT_H

#include <asio.hpp>
#include <thread>

#include "TransportInterface.h"
#include "UDPv4TransportDescriptor.h"
#include "../utils/IPFinder.h"

#include <vector>
#include <memory>
#include <map>
#include <mutex>

namespace eprosima{
namespace fastrtps{
namespace rtps{

/**
 * This is a default UDPv4 implementation.
 *    - Opening an output channel by passing a locator will open a socket per interface on the given port.
 *       This collection of sockets constitute the "outbound channel". In other words, a channel corresponds
 *       to a port + a direction.
 *
 *    - It is possible to provide a white list at construction, which limits the interfaces the transport 
 *       will ever be able to interact with. If left empty, all interfaces are allowed.
 *
 *    - Opening an input channel by passing a locator will open a socket listening on the given port on every
 *       whitelisted interface, and join the multicast channel specified by the locator address. Hence, any locator 
 *       that does not correspond to the multicast range will simply open the port without a subsequent join. Joining
 *       multicast groups late is supported by attempting to open the channel again with the same port + a 
 *       multicast address (the OpenInputChannel function will fail, however, because no new channel has been
 *       opened in a strict sense).
 * @ingroup TRANSPORT_MODULE
 */
class UDPv4Transport : public TransportInterface
{
    class SocketInfo
    {
        public:

#if defined(ASIO_HAS_MOVE)
            SocketInfo(asio::ip::udp::socket& socket) :
                socket_(std::move(socket)), only_multicast_purpose_(false)
#else
            SocketInfo(std::shared_ptr<asio::ip::udp::socket> socket) :
                socket_(socket), only_multicast_purpose_(false)
#endif
            {
            }

            SocketInfo(SocketInfo&& socketInfo) :
#if defined(ASIO_HAS_MOVE)
                socket_(std::move(socketInfo.socket_)),
#else
                socket_(socketInfo.socket_),
#endif
                only_multicast_purpose_(socketInfo.only_multicast_purpose_)
            {
            }

            SocketInfo& operator=(SocketInfo&& socketInfo)
            {
#if defined(ASIO_HAS_MOVE)
                socket_ = std::move(socketInfo.socket_);
#else
                socket_ = socketInfo.socket_;
#endif
                only_multicast_purpose_ = socketInfo.only_multicast_purpose_;
                return *this;
            }

            void only_multicast_purpose(const bool value)
            {
                only_multicast_purpose_ = value;
            };

            bool& only_multicast_purpose()
            {
                return only_multicast_purpose_;
            }

            bool only_multicast_purpose() const
            {
                return only_multicast_purpose_;
            }

#if defined(ASIO_HAS_MOVE)
            asio::ip::udp::socket socket_;
#else
            std::shared_ptr<asio::ip::udp::socket> socket_;
#endif
            bool only_multicast_purpose_;

        private:

            SocketInfo(const SocketInfo&) = delete;
            SocketInfo& operator=(const SocketInfo&) = delete;
    };

public:

   RTPS_DllAPI UDPv4Transport(const UDPv4TransportDescriptor&);

   virtual ~UDPv4Transport();

   bool init() override;

   //! Checks whether there are open and bound sockets for the given port.
   virtual bool IsInputChannelOpen(const Locator_t&) const override;

   /**
    * Checks whether there are open and bound sockets for the given port.
    */
   virtual bool IsOutputChannelOpen(const Locator_t&) const override;

   //! Checks for UDPv4 kind.
   virtual bool IsLocatorSupported(const Locator_t&) const override;

   //! Reports whether Locators correspond to the same port.
   virtual bool DoLocatorsMatch(const Locator_t&, const Locator_t&) const override;

   /**
    * Converts a given remote locator (that is, a locator referring to a remote
    * destination) to the main local locator whose channel can write to that
    * destination. In this case it will return a 0.0.0.0 address on that port.
    */
   virtual Locator_t RemoteToMainLocal(const Locator_t&) const override;

   /**
    * Starts listening on the specified port, and if the specified address is in the
    * multicast range, it joins the specified multicast group,
    */
   virtual bool OpenInputChannel(const Locator_t&) override;

   /**
    * Opens a socket on the given address and port (as long as they are white listed).
    */
   virtual bool OpenOutputChannel(Locator_t&) override;

   //! Removes the listening socket for the specified port.
   virtual bool CloseInputChannel(const Locator_t&) override;

   //! Removes all outbound sockets on the given port.
   virtual bool CloseOutputChannel(const Locator_t&) override;

   /**
    * Blocking Send through the specified channel. In both modes, using a localLocator of 0.0.0.0 will
    * send through all whitelisted interfaces provided the channel is open.
    * @param sendBuffer Slice into the raw data to send.
    * @param sendBufferSize Size of the raw data. It will be used as a bounds check for the previous argument.
    * It must not exceed the sendBufferSize fed to this class during construction.
    * @param localLocator Locator mapping to the channel we're sending from.
    * @param remoteLocator Locator describing the remote destination we're sending to.
    */
   virtual bool Send(const octet* sendBuffer, uint32_t sendBufferSize, const Locator_t& localLocator,
                     const Locator_t& remoteLocator) override;
   /**
    * Blocking Receive from the specified channel.
    * @param receiveBuffer vector with enough capacity (not size) to accomodate a full receive buffer. That
    * capacity must not be less than the receiveBufferSize supplied to this class during construction.
    * @param localLocator Locator mapping to the local channel we're listening to.
    * @param[out] remoteLocator Locator describing the remote restination we received a packet from.
    */
   virtual bool Receive(octet* receiveBuffer, uint32_t receiveBufferCapacity, uint32_t& receiveBufferSize,
                        const Locator_t& localLocator, Locator_t& remoteLocator) override;

   virtual LocatorList_t NormalizeLocator(const Locator_t& locator) override;

   virtual LocatorList_t ShrinkLocatorLists(const std::vector<LocatorList_t>& locatorLists) override;

   virtual bool is_local_locator(const Locator_t& locator) const override;

   UDPv4TransportDescriptor get_configuration() { return mConfiguration_; }

protected:

   //! Constructor with no descriptor is necessary for implementations derived from this class.
   UDPv4Transport();
   UDPv4TransportDescriptor mConfiguration_;
   uint32_t mSendBufferSize;
   uint32_t mReceiveBufferSize;

   asio::io_service mService;
   std::unique_ptr<std::thread> ioServiceThread;

   mutable std::recursive_mutex mOutputMapMutex;
   mutable std::recursive_mutex mInputMapMutex;

   //! The notion of output channel corresponds to a port.
   std::map<uint32_t, std::vector<SocketInfo> > mOutputSockets;

   std::vector<IPFinder::info_IP> currentInterfaces;

   struct LocatorCompare{ bool operator()(const Locator_t& lhs, const Locator_t& rhs) const
                        {return (memcmp(&lhs, &rhs, sizeof(Locator_t)) < 0); } };

   //! For both modes, an input channel corresponds to a port.
#if defined(ASIO_HAS_MOVE)
   std::map<uint32_t, asio::ip::udp::socket> mInputSockets;
#else
   std::map<uint32_t, std::shared_ptr<asio::ip::udp::socket>> mInputSockets;
#endif

   bool IsInterfaceAllowed(const asio::ip::address_v4& ip);
   std::vector<asio::ip::address_v4> mInterfaceWhiteList;

   bool OpenAndBindOutputSockets(Locator_t& locator);
   bool OpenAndBindInputSockets(uint32_t port, bool is_multicast);

#if defined(ASIO_HAS_MOVE)
   asio::ip::udp::socket OpenAndBindUnicastOutputSocket(const asio::ip::address_v4&, uint32_t& port);
   asio::ip::udp::socket OpenAndBindInputSocket(uint32_t port, bool is_multicast);

   bool SendThroughSocket(const octet* sendBuffer,
                          uint32_t sendBufferSize,
                          const Locator_t& remoteLocator,
                          asio::ip::udp::socket& socket);
#else
   std::shared_ptr<asio::ip::udp::socket> OpenAndBindUnicastOutputSocket(const asio::ip::address_v4&, uint32_t& port);
   std::shared_ptr<asio::ip::udp::socket> OpenAndBindInputSocket(uint32_t port, bool is_multicast);

   bool SendThroughSocket(const octet* sendBuffer,
                          uint32_t sendBufferSize,
                          const Locator_t& remoteLocator,
                          std::shared_ptr<asio::ip::udp::socket> socket);
#endif
};

} // namespace rtps
} // namespace fastrtps
} // namespace eprosima

#endif
