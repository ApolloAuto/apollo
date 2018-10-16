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

#ifndef _FASTCDR_CONFIG_H_
#define _FASTCDR_CONFIG_H_

#ifndef FASTCDR_VERSION_MAJOR
#define FASTCDR_VERSION_MAJOR 1
#endif
#ifndef FASTCDR_VERSION_MINOR
#define FASTCDR_VERSION_MINOR 0
#endif
#ifndef FASTCDR_VERSION_MICRO
#define FASTCDR_VERSION_MICRO 7
#endif
#ifndef FASTCDR_VERSION_STR
#define FASTCDR_VERSION_STR "1.0.7"
#endif

// C++11 support defines
#ifndef HAVE_CXX11
#define HAVE_CXX11 1
#endif

// C++0x support defines
#ifndef HAVE_CXX0X
#define HAVE_CXX0X 1
#endif

// Endianness defines
#ifndef __BIG_ENDIAN__
#define __BIG_ENDIAN__ 0
#endif

// C++11 Non-copyable feature
#ifndef NON_COPYABLE_CXX11
#if !defined(HAVE_CXX0X) || (defined(_MSC_VER) && _MSC_VER <= 1600)
#define NON_COPYABLE_CXX11
#else
#define NON_COPYABLE_CXX11 = delete
#endif
#endif

#endif // _FASTCDR_CONFIG_H_
