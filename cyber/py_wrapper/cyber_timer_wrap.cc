/******************************************************************************
 * Copyright 2019 The Apollo Authors. All Rights Reserved.
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

#include <Python.h>
#include <set>
#include <string>

#include "cyber/py_wrapper/py_timer.h"

#define PYOBJECT_NULL_STRING PyString_FromStringAndSize("", 0)

template <typename T>
T PyObjectToPtr(PyObject* pyobj, const std::string& type_ptr) {
  T obj_ptr = (T)PyCapsule_GetPointer(pyobj, type_ptr.c_str());
  if (obj_ptr == nullptr) {
    AERROR << "PyObjectToPtr failed,type->" << type_ptr << "pyobj: " << pyobj;
  }
  return obj_ptr;
}

PyObject* cyber_new_PyTimer(PyObject* self, PyObject* args) {
  uint32_t period = 0;
  PyObject* pyobj_regist_fun = 0;
  unsigned int oneshot = 0;
  if (!PyArg_ParseTuple(args, const_cast<char*>("kOI:cyber_new_PyTimer"),
                        &period, &pyobj_regist_fun, &oneshot)) {
    AERROR << "cyber_new_PyTimer parsetuple failed!";
    Py_INCREF(Py_None);
    return Py_None;
  }

  void (*callback_fun)() = (void (*)())0;
  callback_fun = (void (*)())PyInt_AsLong(pyobj_regist_fun);
  apollo::cyber::PyTimer* pytimer =
      new apollo::cyber::PyTimer(period, callback_fun, oneshot != 0);
  PyObject* pyobj_timer =
      PyCapsule_New(pytimer, "apollo_cybertron_pytimer", nullptr);
  return pyobj_timer;
}

PyObject* cyber_new_PyTimer_noparam(PyObject* self, PyObject* args) {
  apollo::cyber::PyTimer* pytimer = new apollo::cyber::PyTimer();
  PyObject* pyobj_timer =
      PyCapsule_New(pytimer, "apollo_cybertron_pytimer", nullptr);
  return pyobj_timer;
}

PyObject* cyber_delete_PyTimer(PyObject* self, PyObject* args) {
  PyObject* pyobj_timer = nullptr;
  if (!PyArg_ParseTuple(args, const_cast<char*>("O:cyber_delete_PyTimer"),
                        &pyobj_timer)) {
    Py_INCREF(Py_None);
    return Py_None;
  }

  auto pytimer = (apollo::cyber::PyTimer*)PyCapsule_GetPointer(
      pyobj_timer, "apollo_cybertron_pytimer");
  if (nullptr == pytimer) {
    AERROR << "cyber_delete_PyTimer:timer ptr is null!";
    Py_INCREF(Py_None);
    return Py_None;
  }
  delete pytimer;
  Py_INCREF(Py_None);
  return Py_None;
}

PyObject* cyber_PyTimer_start(PyObject* self, PyObject* args) {
  PyObject* pyobj_timer = nullptr;
  if (!PyArg_ParseTuple(args, const_cast<char*>("O:cyber_delete_PyTimer"),
                        &pyobj_timer)) {
    Py_INCREF(Py_None);
    return Py_None;
  }

  auto pytimer = (apollo::cyber::PyTimer*)PyCapsule_GetPointer(
      pyobj_timer, "apollo_cybertron_pytimer");
  if (nullptr == pytimer) {
    AERROR << "cyber_delete_PyTimer:timer ptr is null!";
    Py_INCREF(Py_None);
    return Py_None;
  }
  pytimer->start();
  Py_INCREF(Py_None);
  return Py_None;
}

PyObject* cyber_PyTimer_stop(PyObject* self, PyObject* args) {
  PyObject* pyobj_timer = nullptr;
  if (!PyArg_ParseTuple(args, const_cast<char*>("O:cyber_delete_PyTimer"),
                        &pyobj_timer)) {
    Py_INCREF(Py_None);
    return Py_None;
  }

  auto pytimer = (apollo::cyber::PyTimer*)PyCapsule_GetPointer(
      pyobj_timer, "apollo_cybertron_pytimer");
  if (nullptr == pytimer) {
    AERROR << "cyber_delete_PyTimer:timer ptr is null!";
    Py_INCREF(Py_None);
    return Py_None;
  }
  pytimer->stop();
  Py_INCREF(Py_None);
  return Py_None;
}

PyObject* cyber_PyTimer_set_option(PyObject* self, PyObject* args) {
  PyObject* pyobj_timer = 0;
  uint32_t period = 0;
  PyObject* pyobj_regist_fun = 0;
  unsigned int oneshot = 0;

  void (*callback_fun)() = (void (*)())0;

  if (!PyArg_ParseTuple(args,
                        const_cast<char*>("OkOI:cyber_PyTimer_set_option"),
                        &pyobj_timer, &period, &pyobj_regist_fun, &oneshot)) {
    Py_INCREF(Py_None);
    return Py_None;
  }

  apollo::cyber::PyTimer* pytimer = PyObjectToPtr<apollo::cyber::PyTimer*>(
      pyobj_timer, "apollo_cybertron_pytimer");
  callback_fun = (void (*)())PyInt_AsLong(pyobj_regist_fun);
  if (nullptr == pytimer) {
    AERROR << "cyber_PyTimer_set_option ptr is null!";
    Py_INCREF(Py_None);
    return Py_None;
  }

  pytimer->set_option(period, callback_fun, oneshot != 0);

  Py_INCREF(Py_None);
  return Py_None;
}

static PyMethodDef _cyber_timer_methods[] = {
    {"new_PyTimer_noparam", cyber_new_PyTimer_noparam, METH_NOARGS, ""},
    {"new_PyTimer", cyber_new_PyTimer, METH_VARARGS, ""},
    {"delete_PyTimer", cyber_delete_PyTimer, METH_VARARGS, ""},
    {"PyTimer_start", cyber_PyTimer_start, METH_VARARGS, ""},
    {"PyTimer_stop", cyber_PyTimer_stop, METH_VARARGS, ""},
    {"PyTimer_set_option", cyber_PyTimer_set_option, METH_VARARGS, ""},

    {NULL, NULL, 0, NULL} /* sentinel */
};

/// Init function of this module
PyMODINIT_FUNC init_cyber_timer(void) {
  AINFO << "init _cyber_timer";
  Py_InitModule("_cyber_timer", _cyber_timer_methods);
}
