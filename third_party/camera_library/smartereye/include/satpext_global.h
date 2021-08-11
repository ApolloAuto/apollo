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
#ifndef SATPEXT_GLOBAL_H
#define SATPEXT_GLOBAL_H

#include <QtCore/qglobal.h>

#if defined(SATPEXT_LIBRARY)
#  define SATPEXTSHARED_EXPORT Q_DECL_EXPORT
#else
#  define SATPEXTSHARED_EXPORT Q_DECL_IMPORT
#endif

#endif // SATPEXT_GLOBAL_H
