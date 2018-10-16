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

/*!
 * @file Handle.h
 */
#ifndef _RTPS_SECURITY_COMMON_HANDLE_H_
#define _RTPS_SECURITY_COMMON_HANDLE_H_

#include <memory>
#include <string>

namespace eprosima {
namespace fastrtps {
namespace rtps {
namespace security {


class Handle
{
    public:

        const std::string& get_class_id() const
        {
            return class_id_;
        }

        virtual bool nil() const = 0;

    protected:

        Handle(const std::string& class_id) : class_id_(class_id) {};

        virtual ~Handle(){}

    private:

        std::string class_id_;
};

template<typename T>
class HandleImpl : public Handle
{
    public:

        typedef T type;

        HandleImpl() : Handle(T::class_id_), impl_(new T) {}

        static HandleImpl<T>& narrow(Handle& handle)
        {
            if(handle.get_class_id().compare(T::class_id_) == 0)
                return reinterpret_cast<HandleImpl<T>&>(handle);

            return HandleImpl<T>::nil_handle;
        }

        static const HandleImpl<T>& narrow(const Handle& handle)
        {
            if(handle.get_class_id().compare(T::class_id_) == 0)
                return reinterpret_cast<const HandleImpl<T>&>(handle);

            return HandleImpl<T>::nil_handle;
        }

        bool nil() const
        {
            return impl_ ? false : true;
        }

        T* operator*()
        {
            return impl_.get();
        }

        const T* operator*() const
        {
            return impl_.get();
        }

        T* operator->()
        {
            return impl_.get();
        }

        const T* operator->() const
        {
            return impl_.get();
        }

        static HandleImpl<T> nil_handle;

    private:

        explicit HandleImpl(bool) : Handle(T::class_id_) {}

        std::unique_ptr<T> impl_;
};
template<typename T>
HandleImpl<T> HandleImpl<T>::nil_handle(true);

class NilHandle : public Handle
{
    public:

        NilHandle() : Handle("nil_handle") {}

        bool nil() const { return true; }
};


// Define common handlers
typedef Handle IdentityHandle;

typedef Handle PermissionsHandle;

typedef Handle ParticipantCryptoHandle;
typedef Handle DatawriterCryptoHandle;
typedef Handle DatareaderCryptoHandle;

} //namespace security
} //namespace rtps
} //namespace fastrtps
} //namespace eprosima

#endif // _RTPS_SECURITY_COMMON_HANDLE_H_
