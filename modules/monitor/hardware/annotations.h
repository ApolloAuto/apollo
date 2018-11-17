/******************************************************************************
 * Copyright 2017 The Apollo Authors. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *****************************************************************************/

#ifndef MODULES_MONITOR_HARDWARE_ANNOTATIONS_H_
#define MODULES_MONITOR_HARDWARE_ANNOTATIONS_H_

/// Indicating a pointer class member is not owned by an object of the class; it
/// is the responsibility of the programmer to make sure the given pointer is
/// valid during the life time of the object.
#define PTR_NOT_OWNED

/// Indicating a pointer function argument is used by the object of the function
/// for the lifetime of the object; it is the responsibility of the programmer
/// to make sure the given pointer is valid during the life time of the object.
#define PTR_LIFETIME

/// Indicating ownership of a pointer argument is transferred to the callee.
#define PTR_OWNER_XFR

/// Indicating a pointer function argument is used only in the scope of this
/// function, will not be used after this function is done (e.g., not saved for
/// future use).
/// This is the default behavior of any pointer argument of a function.
#define PTR_USE_ONCE

/// Indicating access to the given (data) member is lock-protected by the given
/// lock.
#define XLOCK_BY(lock)

/// Indicating this function will acquire the given exclusive lock(s).
#define ACQ_LOCK(locks...)

/// Indicating this function should only be called when the given exclusive
/// lock(s) is/are locked.
#define WITH_LOCK(locks...)

#define THREAD_SAFE

/// Indicating private global variable that is implementation specific and
/// should not be used directly.
#define PRIVATE

#endif  // MODULES_MONITOR_HARDWARE_ANNOTATIONS_H_
