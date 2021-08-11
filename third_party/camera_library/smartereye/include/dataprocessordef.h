/******************************************************************************
 * Copyright 2020 The Beijing Smarter Eye Technology Co.Ltd Authors. All
 * Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the License);
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an AS IS BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *****************************************************************************/
#ifndef DATAPROCESSOR_H
#define DATAPROCESSOR_H

#ifdef _WIN64
#   define DATAPROCESSOR_EXPORT     __declspec(dllexport)
#   define DATAPROCESSOR_IMPORT     __declspec(dllimport)
#else
#   define DATAPROCESSOR_EXPORT     __attribute__((visibility("default")))
#   define DATAPROCESSOR_IMPORT     __attribute__((visibility("default")))
#   define DATAPROCESSOR_HIDDEN     __attribute__((visibility("hidden")))
#endif

#if defined(DATAPROCESSOR_LIBRARY)
#  define DATAPROCESSOR_SHARED_EXPORT DATAPROCESSOR_EXPORT
#else
#  define DATAPROCESSOR_SHARED_EXPORT DATAPROCESSOR_IMPORT
#endif

#endif // DATAPROCESSOR_H
