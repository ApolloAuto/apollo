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
 * @file Handshake.h
 */
#ifndef _RTPS_SECURITY_AUTHENTICATION_HANDSHAKE_H_
#define _RTPS_SECURITY_AUTHENTICATION_HANDSHAKE_H_

#include "../common/Handle.h"
#include "../../common/Token.h"

namespace eprosima {
namespace fastrtps {
namespace rtps {
namespace security {

typedef Handle HandshakeHandle;

typedef Token HandshakeMessageToken;

} //namespace eprosima
} //namespace fastrtps
} //namespace rtps
} //namespace security

#endif // _RTPS_SECURITY_AUTHENTICATION_HANDSHAKE_H_
