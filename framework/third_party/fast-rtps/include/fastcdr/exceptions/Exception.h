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

#ifndef _FASTCDR_EXCEPTIONS_EXCEPTION_H_
#define _FASTCDR_EXCEPTIONS_EXCEPTION_H_

#include "../fastcdr_dll.h"
#include <string>
#include <exception>

namespace eprosima
{
    namespace fastcdr
    {
        namespace exception
        {
            /*!
             * @brief This abstract class is used to create exceptions.
             * @ingroup EXCEPTIONMODULE
             */
            class Exception : public std::exception
            {
                public:

                    //! \brief Default destructor.
                    virtual Cdr_DllAPI ~Exception() throw();

                    //! \brief This function throws the object as exception.
                    virtual Cdr_DllAPI void raise() const = 0;

                    /*!
                     * @brief This function returns the error message.
                     *
                     * @return The error message.
                     */
                    virtual Cdr_DllAPI const char* what() const throw() ;

                protected:

                    /*!
                     * @brief Default constructor.
                     *
                     * @param message A error message. This message is copied.
                     */
                    Cdr_DllAPI Exception(const char* const &message);

                    /*!
                     * @brief Default copy constructor.
                     *
                     * @param ex Exception that will be copied.
                     */
                    Cdr_DllAPI Exception(const Exception &ex);

#if HAVE_CXX0X
                    /*!
                     * @brief Default move constructor.
                     *
                     * @param ex Exception that will be moved.
                     */
                    Cdr_DllAPI Exception(Exception&& ex);
#endif

                    /*!
                     * @brief Assigment operation.
                     *
                     * @param ex Exception that will be copied.
                     */
                    Cdr_DllAPI Exception& operator=(const Exception &ex);

#if HAVE_CXX0X
                    /*!
                     * @brief Assigment operation.
                     *
                     * @param ex Exception that will be moved.
                     */
                    Cdr_DllAPI Exception& operator=(Exception&&);
#endif

                private:

                    std::string m_message;
            };
        } //namespace exception
    } //namespace fastcdr
} //namespace eprosima

#endif // _FASTCDR_EXCEPTIONS_EXCEPTION_H_
