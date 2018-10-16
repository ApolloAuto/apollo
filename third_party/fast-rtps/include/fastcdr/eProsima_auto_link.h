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

/*
 Expected defines.

 - EPROSIMA_LIB_NAME
 - FASTCDR_VERSION_MAJOR
 - FASTCDR_VERSION_MINOR
*/

#if defined(_MSC_VER)
    #define EPROSIMA_STRINGIZE(X) EPROSIMA_DO_STRINGIZE(X)
    #define EPROSIMA_DO_STRINGIZE(X) #X

    #if defined(_DEBUG)
        #define EPROSIMA_LIB_DEBUG_TAG "d"
    #else
        #define EPROSIMA_LIB_DEBUG_TAG
    #endif // _DEBUG

    // Select linkage option.
    #if (defined(_DLL) || defined(_RTLDLL)) && defined(EPROSIMA_DYN_LINK)
        #define EPROSIMA_LIB_PREFIX
    #elif defined(EPROSIMA_DYN_LINK)
        #error "Mixing a dll eprosima library with a static runtime is a bad idea"
    #else
        #define EPROSIMA_LIB_PREFIX "lib"
    #endif

    // Include library
    #if defined(EPROSIMA_LIB_NAME) \
	&& defined(EPROSIMA_LIB_PREFIX) \
	&& defined(EPROSIMA_LIB_DEBUG_TAG) \
	&& defined(FASTCDR_VERSION_MAJOR) \
	&& defined(FASTCDR_VERSION_MINOR)
        #pragma comment(lib, EPROSIMA_LIB_PREFIX EPROSIMA_STRINGIZE(EPROSIMA_LIB_NAME) EPROSIMA_LIB_DEBUG_TAG "-" EPROSIMA_STRINGIZE(FASTCDR_VERSION_MAJOR) "." EPROSIMA_STRINGIZE(FASTCDR_VERSION_MINOR) ".lib")
    #else
    #error "Some required macros where not defined"
    #endif

#endif // _MSC_VER
    
// Undef macros
#ifdef EPROSIMA_LIB_PREFIX
#undef EPROSIMA_LIB_PREFIX
#endif

#ifdef EPROSIMA_LIB_NAME
#undef EPROSIMA_LIB_NAME
#endif

#ifdef EPROSIMA_LIB_DEBUG_TAG
#undef EPROSIMA_LIB_DEBUG_TAG
#endif
