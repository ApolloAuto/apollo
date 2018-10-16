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

#ifndef _FASTCDR_CDRBUFFER_H_
#define _FASTCDR_CDRBUFFER_H_

#include "fastcdr_dll.h"
#include <stdint.h>
#include <cstdio>
#include <string.h>
#include <cstddef>

namespace eprosima
{
    namespace fastcdr
    {
        /*!
         * @brief This class implements the iterator used to go through a FastBuffer.
         */
        class Cdr_DllAPI _FastBuffer_iterator
        {
            public:

                /*!
                 * @brief Default constructor.
                 * The iterator points any position.
                 */
                _FastBuffer_iterator() : m_buffer(NULL), m_currentPosition(NULL) {}

                /*!
                 * @brief Constructor.
                 * The iterator points to the indicated position.
                 * @param buffer Pointer to the raw buffer.
                 * @param index Position of the raw buffer where the iterator will point.
                 */
                explicit _FastBuffer_iterator(char *buffer, size_t index) : m_buffer(buffer), m_currentPosition(&m_buffer[index]){}

                /*!
                 * @brief This operator changes the iterator's raw buffer.
                 * This operator makes the iterator point to the same position but in another raw buffer.
                 * The new raw buffer is the same than the source iterator's.
                 * @param iterator The source iterator. The iterator will use the source iterator's raw buffer after this operation.
                 */
                inline
                    void operator<<(const _FastBuffer_iterator &iterator)
                    {
                        ptrdiff_t diff = m_currentPosition - m_buffer;
                        m_buffer = iterator.m_buffer;
                        m_currentPosition = m_buffer + diff;
                    }

                /*!
                 * @brief This operator changes the position where the iterator points.
                 * This operator takes the index of the source iterator, but the iterator continues using its raw buffer.
                 * @param The source iterator. The iterator will use the source's iterator index to point to its own raw buffer.
                 */
                inline
                    void operator>>(const _FastBuffer_iterator &iterator)
                    {
                        ptrdiff_t diff = iterator.m_currentPosition - iterator.m_buffer;
                        m_currentPosition = m_buffer + diff;
                    }

                /*!
                 * @brief This operator copies a data in the raw buffer.
                 * The copy uses the size of the data type.
                 * @param data Data to be copied. Cannot be NULL.
                 */
                template<typename _T>
                    inline
                    void operator<<(const _T &data)
                    {
                        *((_T*)m_currentPosition) = data;
                    }

                /*!
                 * @brief This operator copies data from the raw buffer to a variable.
                 * The copy uses the size of the data type.
                 * @param data Data to be filled.
                 */
                template<typename _T>
                    inline
                    void operator>>(_T &data)
                    {
                        data = *((_T*)m_currentPosition);
                    }

                /*!
                 * @brief This function copies a buffer into the raw buffer.
                 * @param src The source buffer.
                 * @param size The number of bytes to be copied.
                 */
                inline
                    void memcopy(const void* src, const size_t size)
                    {
                        memcpy(m_currentPosition, src, size);
                    }

                /*!
                 * @brief This function copies from the raw buffer to a external buffer.
                 * @param dst The destination buffer.
                 * @param size The size of bytes to be copied.
                 */
                inline
                    void rmemcopy(void* dst, const size_t size)
                    {
                        memcpy(dst, m_currentPosition, size);
                    }

                /*!
                 * @brief This function increments the position where the iterator points.
                 * @param numBytes Number of bytes the iterator moves the position.
                 */
                inline
                    void operator+=(size_t numBytes)
                    {
                        m_currentPosition += numBytes;
                    }

                /*!
                 * @brief This operator returns the subtraction of the current interator's position and the source iterator's position.
                 * @iterator Source iterator whose position is subtracted to the current iterator's position.
                 * @return The result of subtract the current iterator's position and the source iterator's position.
                 */
                inline
                    size_t operator-(const _FastBuffer_iterator &it) const
                    {
                        return m_currentPosition - it.m_currentPosition;
                    }

                /*!
                 * @brief This function increments the iterator in one the position.
                 * @return The current iterator.
                 */
                inline
                    _FastBuffer_iterator operator++()
                    {
                        ++m_currentPosition;
                        return *this;
                    }

                /*!
                 * @brief This function increments the iterator in one the position.
                 * @return The current iterator.
                 */
                inline
                    _FastBuffer_iterator operator++(int)
                    {
                        _FastBuffer_iterator tmp = *this;
                        ++*this;
                        return tmp;
                    }

                /*!
                 * @brief This function returns the current position in the raw buffer.
                 * @return The current position in the raw buffer.
                 */
                inline 
                    char* operator&()
                    {
                        return m_currentPosition;
                    }

            private:

                //! Pointer to the raw buffer.
                char *m_buffer;

                //! Current position in the raw buffer.
                char *m_currentPosition;
        };

        /*!
         * @brief This class represents a stream of bytes that contains (or will contain)
         * serialized data. This class is used by the serializers to serialize
         * or deserialize using their representation.
         * @ingroup FASTCDRAPIREFERENCE
         */
        class Cdr_DllAPI FastBuffer
        {
            public:
                typedef _FastBuffer_iterator iterator;

                /*!
                 * @brief This constructor creates an internal stream and assigns it to the eprosima::fastcdr::FastBuffers object.
                 * The user can obtain this internal stream using the function eprosima::fastcdr::FastBuffers::getBuffer(). Be careful because this internal stream
                 * is deleted in the destruction of the eprosima::fastcdr::FastBuffers object.
                 */
                FastBuffer();

                /*!
                 * @brief This constructor assigns the user's stream of bytes to the eprosima::fastcdr::FastBuffers object.
                 * The user's stream will be used to serialize.
                 *
                 * @param buffer The user's buffer that will be used. This buffer is not deallocated in the object's destruction. Cannot be NULL.
                 * @param bufferSize The length of user's buffer.
                 */
                FastBuffer(char* const buffer, const size_t bufferSize);

                /*!
                 * @brief Default destructor.
                 */
                virtual ~FastBuffer();

                /*!
                 * @brief This function returns the stream that the eprosima::fastcdr::FastBuffers uses to serialize data.
                 * @return The stream used by eprosima::fastcdr::FastBuffers to serialize data.
                 */
                inline char* getBuffer() const { return m_buffer;}

                /*!
                 * @brief This function returns the size of the allocated memory of the stream that the eprosima::fastcdr::FastBuffers uses to serialize data.
                 * @return The size of the allocated memory of the stream used by the eprosima::fastcdr::FastBuffers to serialize data.
                 */
                inline size_t getBufferSize() const { return m_bufferSize;}

                /*!
                 * @brief This function returns a iterator that points to the begining of the stream.
                 * @return The new iterator.
                 */
                inline
                    iterator begin()
                    {
                        return (iterator(m_buffer, 0));
                    }

                /*!
                 * @brief This function returns a iterator that points to the end of the stream.
                 * @return The new iterator.
                 */
                inline
                    iterator end()
                    {
                        return (iterator(m_buffer, m_bufferSize));
                    }

                /*!
                 * @brief This function resizes the raw buffer. It will call the user's defined function for this purpose.
                 * @param minSizeInc The minimun growth expected of the current raw buffer.
                 * @return True if the operation works. False if it does not.
                 */
                bool resize(size_t minSizeInc);

            private:

                //! @brief Pointer to the stream of bytes that contains the serialized data.
                char *m_buffer;

                //! @brief The total size of the user's buffer.
                size_t m_bufferSize;

                //! @brief This variable indicates if the managed buffer is internal or is from the user.
                bool m_internalBuffer;
        };
    } //namespace fastcdr
} //namespace eprosima

#endif // _FASTCDR_FASTCDRBUFFER_H_
