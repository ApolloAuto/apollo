// MIT License
//
// Copyright(c) 2019 ZVISION. All rights reserved.
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files(the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and / or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions :
//
// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.

#pragma once

#include <string>
namespace apollo
{
  namespace drivers
  {
    namespace zvision
    {

      /** \brief Convert the ip string in the standard IPv4 dotted decimal notation to 4 bytes char array in big endian(network byte order).
       * \param[in] ip    string which is in the standard IPv4 dotted decimal notation
       * \param[in] addr  destination address for the char array
       * \return true for success, false for failure.
       */
      bool AssembleIpString(std::string ip, char *addr);

      /** \brief Convert the port to 4 bytes char array in big endian(network byte order).
       * \param[in] port  port number
       * \param[in] addr  destination address for the char array
       */
      void AssemblePort(int port, char *addr);

      /** \brief Convert the mac address string("%hhx-%hhx-%hhx-%hhx-%hhx-%hhx") to 6 bytes char array.
       * \param[in] mac   mac address in the string format
       * \param[in] addr  destination address for the char array
       * \return true for success, false for failure.
       */
      bool AssembleMacAddress(std::string mac, char *addr);

      /** \brief Convert the ip address in big-endian(network byte order) char array format to "%u.%u.%u.%u" string.
       * \param[in] addr  ip address in big-endian(network byte order) char array format
       * \param[in] ip    "%u.%u.%u.%u" string
       */
      void ResolveIpString(const unsigned char *addr, std::string &ip);

      /** \brief Convert the port number in big-endian(network byte order) char array format to int.
       * \param[in] addr  port number in big-endian(network byte order) char array format
       * \param[in] port  port number
       */
      void ResolvePort(const unsigned char *addr, int &port);

      /** \brief Convert the mac address in char array format to string.
       * \param[in] addr  mac address data in char array format
       * \param[in] mac   mac address in string format
       */
      void ResolveMacAddress(const unsigned char *addr, std::string &mac);

      /** \brief Convert the ip string in the standard IPv4 dotted decimal notation to integer(host byte order).
       * \param[in] ip    string which is in the standard IPv4 dotted decimal notation
       * \param[in] iip   ip address in integer format
       * \return true for success, false for failure.
       */
      bool StringToIp(std::string ip, unsigned int &iip);

      /** \brief Convert the  to integer(host byte order) to ip string in the standard IPv4 dotted decimal notation.
       * \param[in] ip    ip address in integer format
       * \return the ip string.
       */
      std::string IpToString(int ip);

      /** \brief Convert network byte order to host byte order. 4 bytes cycle.
       * \param[in] net    raw data
       * \param[in] host   destination address
       * \param[in] len    how many bytes to convert
       */
      void NetworkToHost(const unsigned char *net, char *host, int len = 4);

      /** \brief Convert network byte order to host byte order. 2 bytes cycle.
       * \param[in] net    raw data
       * \param[in] host   destination address
       * \param[in] len    how many bytes to convert
       */
      void NetworkToHostShort(const unsigned char *net, char *host, int len = 2);

      /** \brief Convert host byte order to network byte order. 4 bytes cycle.
       * \param[in] host   raw data
       * \param[in] net    destination address
       * \param[in] len    how many bytes to convert
       */
      void HostToNetwork(const unsigned char *host, char *net, int len = 4);

      /** \brief Swap the byte order. 4 bytes cycle.
       * \param[in] src    raw data
       * \param[in] dst    destination address
       * \param[in] len    how many bytes to convert
       */
      void SwapByteOrder(char *src, char *dst, int len = 4);

      /** \brief Get system error code.
       * \param[in] len    how many bytes to convert
       * \return error code.
       */
      int GetSysErrorCode();

      //////////////////////////////////////////////////////////////////////////////////////////////
      /** \brief Env represents the socket library init status.
       * \author zvision
       */
      class Env
      {
      public:
        /** \brief get socket library init ok or not.
         * \If not, init it and return the status.
         * \return true for init ok, false for error.
         */
        static bool Ok();

        Env(const Env &) = delete;
        Env &operator=(const Env &) = delete;

      protected:
        /** \brief private constructor. */
        Env();
      };

      //////////////////////////////////////////////////////////////////////////////////////////////
      /** \brief Tcpclient represents the tcp client which wraps socket windows and linux.
       * \author zvision
       */
      class TcpClient
      {
      public:
        /** \brief zvision TcpClient constructor.
         * \param[in] connect_timeout timeout in ms for Connect function
         * \param[in] send_timeout    timeout in ms for SyncSend function
         * \param[in] recv_timeout    timeout in ms for SyncRecv function
         */
        TcpClient(int connect_timeout, int send_timeout, int recv_timeout);

        /** \brief Empty destructor */
        virtual ~TcpClient();

        /** \brief Calls the Connect method to connect to the server.
         * \param[in] dst_ip   the server's ip address
         * \param[in] dst_port server's listening port
         * \return 0 for success, others for failure.
         */
        int Connect(std::string dst_ip, int dst_port = 3000);

        /** \brief Calls the SyncSend method to send data to server.
         * \param[in] data the data send to server
         * \param[in] len  the length of data
         * \return 0 for success, others for failure.
         */
        int SyncSend(std::string &data, int len);

        /** \brief Calls the SyncRecv method to reveive data from server.
         * \param[in] data the buffer to store the data from server
         * \param[in] len  the length to receive
         * \return 0 for success, others for failure.
         */
        int SyncRecv(std::string &data, int len);

        /** \brief Calls the Close method to close the connection.
         * \return 0 for success, others for failure.
         */
        int Close();

        int GetSysErrorCode();

      protected:
        std::string RecordErrorInfo();

      private:
        /** \brief Server ip address. */
        std::string server_ip_;

        /** \brief Server listening port. */
        int server_port_;

        /** \brief timeout(ms) for connection. */
        int conn_timeout_ms_;

        /** \brief timeout(ms) for send. */
        int send_timeout_ms_;

        /** \brief timeout(ms) for recv. */
        int recv_timeout_ms_;

        /** \brief socket which represents the socket resource. */
        int socket_;

        /** \brief connnection is established or not.
         *  false connection error
         *  true  connection is ok
         */
        bool conn_ok_;

        /** \brief store the system error code when something wrong happen. */
        int error_code_;

        /** \brief store the internal error string when something wrong happen. */
        std::string error_str_;
      };

    } // namespace driver
  } // namespace zvision
} // namespace apollo
