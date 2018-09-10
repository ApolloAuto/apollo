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

#ifndef _RTPS_SECURITY_EXCEPTIONS_SECURITYEXCEPTION_H_
#define _RTPS_SECURITY_EXCEPTIONS_SECURITYEXCEPTION_H_

#include "../../exceptions/Exception.h"

namespace eprosima {
namespace fastrtps {
namespace rtps {
namespace security {

/**
 * @brief This class is thrown as an exception when there is an error in security module.
 * @ingroup EXCEPTIONMODULE
 */
class SecurityException : public Exception
{
    public:

        RTPS_DllAPI SecurityException() {}

        /**
         * @brief Default constructor.
         * @param message An error message. This message is copied.
         */
        RTPS_DllAPI SecurityException(const std::string& message) : Exception(message.c_str(), 1) {}

        /**
         * @brief Default copy constructor.
         * @param ex SecurityException that will be copied.
         */
        RTPS_DllAPI SecurityException(const SecurityException &ex);

        /**
         * @brief Default move constructor.
         * @param ex SecurityException that will be moved.
         */
        RTPS_DllAPI SecurityException(SecurityException&& ex);

        /**
         * @brief Assigment operation.
         * @param ex SecurityException that will be copied.
         */
        RTPS_DllAPI SecurityException& operator=(const SecurityException &ex);

        /**
         * @brief Assigment operation.
         * @param ex SecurityException that will be moved.
         */
        RTPS_DllAPI SecurityException& operator=(SecurityException&& ex);

        /// \brief Default constructor
        virtual RTPS_DllAPI ~SecurityException() throw();

        /// \brief This function throws the object as an exception.
        virtual RTPS_DllAPI void raise() const;
};
} // namespace security
} // namespace rtps
} // namespace fastrtps
} // namespace eprosima

#endif // _RTPS_SECURITY_EXCEPTIONS_SECURITYEXCEPTION_H_
