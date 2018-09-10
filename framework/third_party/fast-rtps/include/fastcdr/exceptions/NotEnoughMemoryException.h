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

#ifndef _FASTCDR_EXCEPTIONS_NOTENOUGHMEMORYEXCEPTION_H_
#define _FASTCDR_EXCEPTIONS_NOTENOUGHMEMORYEXCEPTION_H_

#include "Exception.h"

namespace eprosima
{
    namespace fastcdr
    {
        namespace exception
        {
            /*!
             * @brief This class is thrown as an exception when the buffer's internal memory reachs its size limit.
             * @ingroup EXCEPTIONMODULE
             */
            class NotEnoughMemoryException : public Exception
            {
                public:

                    /*!
                     * @brief Default constructor.
                     *
                     * @param message A error message. This message is copied.
                     */
                    Cdr_DllAPI NotEnoughMemoryException(const char* const &message);

                    /*!
                     * @brief Default copy constructor.
                     *
                     * @param ex NotEnoughMemoryException that will be copied.
                     */
                    Cdr_DllAPI NotEnoughMemoryException(const NotEnoughMemoryException &ex);

#if HAVE_CXX0X
                    /*!
                     * @brief Default move constructor.
                     *
                     * @param ex NotEnoughMemoryException that will be moved.
                     */
                    Cdr_DllAPI NotEnoughMemoryException(NotEnoughMemoryException&& ex);
#endif

                    /*!
                     * @brief Assigment operation.
                     *
                     * @param ex NotEnoughMemoryException that will be copied.
                     */
                    Cdr_DllAPI NotEnoughMemoryException& operator=(const NotEnoughMemoryException &ex);

#if HAVE_CXX0X
                    /*!
                     * @brief Assigment operation.
                     *
                     * @param ex NotEnoughMemoryException that will be moved.
                     */
                    Cdr_DllAPI NotEnoughMemoryException& operator=(NotEnoughMemoryException&& ex);
#endif

                    //! @brief Default constructor
                    virtual Cdr_DllAPI ~NotEnoughMemoryException() throw();

                    //! @brief This function throws the object as exception.
                    virtual Cdr_DllAPI void raise() const;

                    //! @brief Default message used in the library.
                    static Cdr_DllAPI const char* const NOT_ENOUGH_MEMORY_MESSAGE_DEFAULT;
            };
        } //namespace exception
    } //namespace fastcdr
} //namespace eprosima
#endif // _FASTCDR_EXCEPTIONS_NOTENOUGHMEMORYEXCEPTION_H_
