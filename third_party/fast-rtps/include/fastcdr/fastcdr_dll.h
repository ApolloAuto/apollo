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

#ifndef _FASTCDR_FASTCDR_DLL_H_
#define _FASTCDR_FASTCDR_DLL_H_

#include "config.h"

// normalize macros
#if !defined(FASTCDR_DYN_LINK) && !defined(FASTCDR_STATIC_LINK) \
    && !defined(EPROSIMA_ALL_DYN_LINK) && !defined(EPROSIMA_ALL_STATIC_LINK)
#define FASTCDR_STATIC_LINK
#endif

#if defined(EPROSIMA_ALL_DYN_LINK) && !defined(FASTCDR_DYN_LINK)
#define FASTCDR_DYN_LINK
#endif

#if defined(FASTCDR_DYN_LINK) && defined(FASTCDR_STATIC_LINK)
#error Must not define both FASTCDR_DYN_LINK and FASTCDR_STATIC_LINK
#endif

#if defined(EPROSIMA_ALL_NO_LIB) && !defined(FASTCDR_NO_LIB)
#define FASTCDR_NO_LIB
#endif

// enable dynamic linking

#if defined(_WIN32)
#if defined(EPROSIMA_ALL_DYN_LINK) || defined(FASTCDR_DYN_LINK)
#if defined(FASTCDR_SOURCE)
#define Cdr_DllAPI __declspec( dllexport )
#else
#define Cdr_DllAPI __declspec( dllimport )
#endif // FASTCDR_SOURCE
#else
#define Cdr_DllAPI
#endif
#else
#define Cdr_DllAPI
#endif // _WIN32

// Auto linking.

#if !defined(FASTCDR_SOURCE) && !defined(EPROSIMA_ALL_NO_LIB) \
    && !defined(FASTCDR_NO_LIB)

// Set properties.
#define EPROSIMA_LIB_NAME fastcdr

#if defined(EPROSIMA_ALL_DYN_LINK) || defined(FASTCDR_DYN_LINK)
#define EPROSIMA_DYN_LINK
#endif

#include "eProsima_auto_link.h"
#endif // auto-linking disabled

#endif // _FASTCDR_FASTCDR_DLL_H_
