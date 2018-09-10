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
 * @file Cryptography.h
 */

#ifndef _RTPS_SECURITY_CRYPTOGRAPHY_CRYPTOGRAPHY_H_
#define _RTPS_SECURITY_CRYPTOGRAPHY_CRYPTOGRAPHY_H_

#include "CryptoKeyExchange.h"
#include "CryptoKeyFactory.h"
#include "CryptoTransform.h"
#include "CryptoTypes.h"

namespace eprosima {
namespace fastrtps {
namespace rtps {
namespace security {

class Cryptography
{
public:

    Cryptography(): m_cryptokeyexchange(nullptr), m_cryptokeyfactory(nullptr),
    m_cryptotransform(nullptr) {}

    virtual ~Cryptography() {}

    /* Specializations should add functions to access the private members */
    CryptoKeyExchange* cryptkeyexchange() { return m_cryptokeyexchange; }

    CryptoKeyFactory* cryptokeyfactory() { return m_cryptokeyfactory; }

    CryptoTransform* cryptotransform() { return m_cryptotransform; }

protected:

    CryptoKeyExchange *m_cryptokeyexchange;
    CryptoKeyFactory *m_cryptokeyfactory;
    CryptoTransform *m_cryptotransform;
};

} //namespace security
} //namespace rtps
} //namespace fastrtps
} //namespace eprosima

#endif //_RTPS_SECURITY_CRYPTOGRAPHY_CRYPTOGRAPHY_H_
