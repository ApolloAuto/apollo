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
#ifndef STEREOCAMERADEF_H
#define STEREOCAMERADEF_H

#ifdef _WIN64
#   define STEREO_EXPORT     __declspec(dllexport)
#   define STEREO_IMPORT     __declspec(dllimport)
#else
#   define STEREO_EXPORT     __attribute__((visibility("default")))
#   define STEREO_IMPORT     __attribute__((visibility("default")))
#   define STEREO_HIDDEN     __attribute__((visibility("hidden")))
#endif

#if defined(STEREOCAMERA_LIBRARY)
#  define STEREO_SHARED_EXPORT STEREO_EXPORT
#else
#  define STEREO_SHARED_EXPORT STEREO_IMPORT
#endif

#endif // STEREOCAMERADEF_H
