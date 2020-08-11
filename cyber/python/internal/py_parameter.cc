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

#include "cyber/python/internal/py_parameter.h"

#include <set>
#include <string>

#include <Python.h>
#include "cyber/python/internal/py_cyber.h"

using apollo::cyber::Parameter;
using apollo::cyber::PyNode;
using apollo::cyber::PyParameter;
using apollo::cyber::PyParameterClient;
using apollo::cyber::PyParameterServer;

#define PYOBJECT_NULL_STRING PyBytes_FromStringAndSize("", 0)
#define C_STR_TO_PY_BYTES(cstr) \
  PyBytes_FromStringAndSize(cstr.c_str(), cstr.size())

template <typename T>
T PyObjectToPtr(PyObject* pyobj, const std::string& type_ptr) {
  T obj_ptr = (T)PyCapsule_GetPointer(pyobj, type_ptr.c_str());
  if (obj_ptr == nullptr) {
    AERROR << "PyObjectToPtr failed,type->" << type_ptr << "pyobj: " << pyobj;
  }
  return obj_ptr;
}

PyObject* cyber_new_PyParameter_noparam(PyObject* self, PyObject* args) {
  PyParameter* pyparameter = new PyParameter();
  return PyCapsule_New(pyparameter, "apollo_cybertron_pyparameter", nullptr);
}

PyObject* cyber_delete_PyParameter(PyObject* self, PyObject* args) {
  PyObject* pyobj_param = nullptr;
  if (!PyArg_ParseTuple(args, const_cast<char*>("O:cyber_delete_PyParameter"),
                        &pyobj_param)) {
    Py_INCREF(Py_None);
    return Py_None;
  }

  auto* pyparameter = reinterpret_cast<PyParameter*>(
      PyCapsule_GetPointer(pyobj_param, "apollo_cybertron_pyparameter"));
  if (nullptr == pyparameter) {
    AERROR << "cyber_delete_PyParameter:parameter ptr is null!";
    Py_INCREF(Py_None);
    return Py_None;
  }
  delete pyparameter;
  Py_INCREF(Py_None);
  return Py_None;
}

PyObject* cyber_new_PyParameter_int(PyObject* self, PyObject* args) {
  char* name = nullptr;
  Py_ssize_t len = 0;
  int64_t int_value = 0;
  if (!PyArg_ParseTuple(args,
                        const_cast<char*>("s#L:cyber_new_PyParameter_int"),
                        &name, &len, &int_value)) {
    AERROR << "cyber_new_PyParameter_int parsetuple failed!";
    Py_INCREF(Py_None);
    return Py_None;
  }

  PyParameter* pyparameter = new PyParameter(std::string(name, len), int_value);
  return PyCapsule_New(pyparameter, "apollo_cybertron_pyparameter", nullptr);
}

PyObject* cyber_new_PyParameter_double(PyObject* self, PyObject* args) {
  char* name = nullptr;
  Py_ssize_t len = 0;
  double double_value = 0;
  if (!PyArg_ParseTuple(args,
                        const_cast<char*>("s#d:cyber_new_PyParameter_double"),
                        &name, &len, &double_value)) {
    AERROR << "cyber_new_PyParameter_double parsetuple failed!";
    Py_INCREF(Py_None);
    return Py_None;
  }

  PyParameter* pyparameter =
      new PyParameter(std::string(name, len), double_value);
  return PyCapsule_New(pyparameter, "apollo_cybertron_pyparameter", nullptr);
}

PyObject* cyber_new_PyParameter_string(PyObject* self, PyObject* args) {
  char* name = nullptr;
  char* string_param = nullptr;
  if (!PyArg_ParseTuple(args,
                        const_cast<char*>("ss:cyber_new_PyParameter_string"),
                        &name, &string_param)) {
    AERROR << "cyber_new_PyParameter_string parsetuple failed!";
    Py_INCREF(Py_None);
    return Py_None;
  }

  PyParameter* pyparameter =
      new PyParameter(std::string(name), std::string(string_param));
  return PyCapsule_New(pyparameter, "apollo_cybertron_pyparameter", nullptr);
}

PyObject* cyber_PyParameter_type_name(PyObject* self, PyObject* args) {
  PyObject* pyobj_param = nullptr;
  if (!PyArg_ParseTuple(args,
                        const_cast<char*>("O:cyber_PyParameter_type_name"),
                        &pyobj_param)) {
    AERROR << "cyber_PyParameter_type_name failed!";
    return PYOBJECT_NULL_STRING;
  }

  auto* param = reinterpret_cast<PyParameter*>(
      PyCapsule_GetPointer(pyobj_param, "apollo_cybertron_pyparameter"));
  if (nullptr == param) {
    AERROR << "cyber_PyParameter_type_name ptr is null!";
    return PYOBJECT_NULL_STRING;
  }

  const std::string type = param->type_name();
  return C_STR_TO_PY_BYTES(type);
}

PyObject* cyber_PyParameter_descriptor(PyObject* self, PyObject* args) {
  PyObject* pyobj_param = nullptr;
  if (!PyArg_ParseTuple(args,
                        const_cast<char*>("O:cyber_PyParameter_descriptor"),
                        &pyobj_param)) {
    AERROR << "cyber_PyParameter_descriptor failed!";
    return PYOBJECT_NULL_STRING;
  }

  auto* param = reinterpret_cast<PyParameter*>(
      PyCapsule_GetPointer(pyobj_param, "apollo_cybertron_pyparameter"));
  if (nullptr == param) {
    AERROR << "cyber_PyParameter_descriptor ptr is null!";
    return PYOBJECT_NULL_STRING;
  }

  const std::string desc = param->descriptor();
  return C_STR_TO_PY_BYTES(desc);
}

PyObject* cyber_PyParameter_name(PyObject* self, PyObject* args) {
  PyObject* pyobj_param = nullptr;
  if (!PyArg_ParseTuple(args, const_cast<char*>("O:cyber_PyParameter_name"),
                        &pyobj_param)) {
    AERROR << "cyber_PyParameter_name failed!";
    return PYOBJECT_NULL_STRING;
  }

  auto* param = reinterpret_cast<PyParameter*>(
      PyCapsule_GetPointer(pyobj_param, "apollo_cybertron_pyparameter"));
  if (nullptr == param) {
    AERROR << "cyber_PyParameter_name ptr is null!";
    return PYOBJECT_NULL_STRING;
  }

  const std::string name = param->name();
  return C_STR_TO_PY_BYTES(name);
}

PyObject* cyber_PyParameter_debug_string(PyObject* self, PyObject* args) {
  PyObject* pyobj_param = nullptr;
  if (!PyArg_ParseTuple(args,
                        const_cast<char*>("O:cyber_PyParameter_debug_string"),
                        &pyobj_param)) {
    AERROR << "cyber_PyParameter_debug_string failed!";
    return PYOBJECT_NULL_STRING;
  }

  auto* param = reinterpret_cast<PyParameter*>(
      PyCapsule_GetPointer(pyobj_param, "apollo_cybertron_pyparameter"));
  if (nullptr == param) {
    AERROR << "cyber_PyParameter_debug_string ptr is null!";
    return PYOBJECT_NULL_STRING;
  }

  const std::string debug_string = param->debug_string();
  return C_STR_TO_PY_BYTES(debug_string);
}

PyObject* cyber_PyParameter_as_string(PyObject* self, PyObject* args) {
  PyObject* pyobj_param = nullptr;
  if (!PyArg_ParseTuple(args,
                        const_cast<char*>("O:cyber_PyParameter_as_string"),
                        &pyobj_param)) {
    AERROR << "cyber_PyParameter_as_string failed!";
    return PYOBJECT_NULL_STRING;
  }

  auto* param = reinterpret_cast<PyParameter*>(
      PyCapsule_GetPointer(pyobj_param, "apollo_cybertron_pyparameter"));
  if (nullptr == param) {
    AERROR << "cyber_PyParameter_as_string ptr is null!";
    return PYOBJECT_NULL_STRING;
  }

  std::string debug_string = param->as_string();
  return C_STR_TO_PY_BYTES(debug_string);
}

PyObject* cyber_PyParameter_as_double(PyObject* self, PyObject* args) {
  PyObject* pyobj_param = nullptr;
  if (!PyArg_ParseTuple(args,
                        const_cast<char*>("O:cyber_PyParameter_as_double"),
                        &pyobj_param)) {
    AERROR << "cyber_PyParameter_as_double failed!";
    return PyFloat_FromDouble(0.0);
  }

  auto* param = reinterpret_cast<PyParameter*>(
      PyCapsule_GetPointer(pyobj_param, "apollo_cybertron_pyparameter"));
  if (nullptr == param) {
    AERROR << "cyber_PyParameter_as_double ptr is null!";
    return PyFloat_FromDouble(0.0);
  }

  double dTemp = param->as_double();
  return PyFloat_FromDouble(dTemp);
}

PyObject* cyber_PyParameter_as_int64(PyObject* self, PyObject* args) {
  PyObject* pyobj_param = nullptr;
  if (!PyArg_ParseTuple(args, const_cast<char*>("O:cyber_PyParameter_as_int64"),
                        &pyobj_param)) {
    AERROR << "cyber_PyParameter_as_int64 failed!";
    return PyLong_FromLongLong(0);
  }

  auto* param = reinterpret_cast<PyParameter*>(
      PyCapsule_GetPointer(pyobj_param, "apollo_cybertron_pyparameter"));
  if (nullptr == param) {
    AERROR << "cyber_PyParameter_as_int64 ptr is null!";
    return PyLong_FromLongLong(0);
  }

  int64_t lTemp = param->as_int64();
  return PyLong_FromLongLong(lTemp);
}

// PyParameterClient
PyObject* cyber_new_PyParameterClient(PyObject* self, PyObject* args) {
  PyObject* pyobj_node = nullptr;
  char* service_node_name = nullptr;
  Py_ssize_t len = 0;
  if (!PyArg_ParseTuple(args,
                        const_cast<char*>("Os#:cyber_new_PyParameterClient"),
                        &pyobj_node, &service_node_name, &len)) {
    AERROR << "cyber_new_PyParameterClient parsetuple failed!";
    Py_INCREF(Py_None);
    return Py_None;
  }

  PyNode* pynode = PyObjectToPtr<PyNode*>(pyobj_node, "apollo_cyber_pynode");
  if (nullptr == pynode) {
    AERROR << "pynode ptr is null!";
    Py_INCREF(Py_None);
    return Py_None;
  }

  auto node = pynode->get_node();
  if (nullptr == node) {
    AERROR << "node ptr is null!";
    Py_INCREF(Py_None);
    return Py_None;
  }

  PyParameterClient* pyparameter_clt =
      new PyParameterClient(node, std::string(service_node_name, len));
  return PyCapsule_New(pyparameter_clt, "apollo_cybertron_pyparameterclient",
                       nullptr);
}

PyObject* cyber_delete_PyParameterClient(PyObject* self, PyObject* args) {
  PyObject* pyobj_param = nullptr;
  if (!PyArg_ParseTuple(args,
                        const_cast<char*>("O:cyber_delete_PyParameterClient"),
                        &pyobj_param)) {
    AERROR << "cyber_delete_PyParameterClient parsetuple failed!";
    Py_INCREF(Py_None);
    return Py_None;
  }

  PyParameterClient* pyparameter_clt = reinterpret_cast<PyParameterClient*>(
      PyCapsule_GetPointer(pyobj_param, "apollo_cybertron_pyparameterclient"));
  if (nullptr == pyparameter_clt) {
    AERROR << "cyber_delete_PyParameterClient:pyparameter_clt ptr is null!";
    Py_INCREF(Py_None);
    return Py_None;
  }
  delete pyparameter_clt;
  Py_INCREF(Py_None);
  return Py_None;
}

PyObject* cyber_PyParameter_clt_set_parameter(PyObject* self, PyObject* args) {
  PyObject* pyobj_param_clt = nullptr;
  PyObject* pyobj_param = nullptr;
  if (!PyArg_ParseTuple(args,
                        const_cast<char*>("OO:cyber_PyParameter_set_parameter"),
                        &pyobj_param_clt, &pyobj_param)) {
    AERROR << "cyber_PyParameter_set_parameter parsetuple failed!";
    Py_RETURN_FALSE;
  }

  PyParameterClient* pyparam_clt = PyObjectToPtr<PyParameterClient*>(
      pyobj_param_clt, "apollo_cybertron_pyparameterclient");
  if (nullptr == pyparam_clt) {
    AERROR << "pyparam_clt ptr is null!";
    Py_RETURN_FALSE;
  }

  PyParameter* pyparam =
      PyObjectToPtr<PyParameter*>(pyobj_param, "apollo_cybertron_pyparameter");
  if (nullptr == pyparam) {
    AERROR << "pyparam ptr is null!";
    Py_RETURN_FALSE;
  }

  if (!pyparam_clt->set_parameter(pyparam->get_param())) {
    Py_RETURN_FALSE;
  } else {
    Py_RETURN_TRUE;
  }
}

PyObject* cyber_PyParameter_clt_get_parameter(PyObject* self, PyObject* args) {
  char* name = nullptr;
  Py_ssize_t len = 0;
  PyObject* pyobj_param_clt = nullptr;
  if (!PyArg_ParseTuple(
          args, const_cast<char*>("Os#:cyber_PyParameter_get_parameter"),
          &pyobj_param_clt, &name, &len)) {
    AERROR << "cyber_PyParameter_get_parameter parsetuple failed!";
    Py_INCREF(Py_None);
    return Py_None;
  }
  PyParameterClient* pyparam_clt = PyObjectToPtr<PyParameterClient*>(
      pyobj_param_clt, "apollo_cybertron_pyparameterclient");
  if (nullptr == pyparam_clt) {
    AERROR << "pyparam_clt ptr is null!";
    Py_INCREF(Py_None);
    return Py_None;
  }

  Parameter* param = new Parameter();
  std::string str_param = std::string(name, len);
  if (!pyparam_clt->get_parameter(str_param, param)) {
    AERROR << "pyparam_clt get_parameter is false!";
    Py_INCREF(Py_None);
    return Py_None;
  }

  PyParameter* pyparameter = new PyParameter(param);
  return PyCapsule_New(pyparameter, "apollo_cybertron_pyparameter", nullptr);
}

PyObject* cyber_PyParameter_clt_get_parameter_list(PyObject* self,
                                                   PyObject* args) {
  PyObject* pyobj_param_clt = nullptr;
  if (!PyArg_ParseTuple(
          args, const_cast<char*>("O:cyber_PyParameter_clt_get_parameter_list"),
          &pyobj_param_clt)) {
    AERROR
        << "cyber_PyParameter_clt_get_parameter_list:PyArg_ParseTuple failed!";
    Py_INCREF(Py_None);
    return Py_None;
  }

  auto* pyparam_clt = reinterpret_cast<PyParameterClient*>(PyCapsule_GetPointer(
      pyobj_param_clt, "apollo_cybertron_pyparameterclient"));
  if (nullptr == pyparam_clt) {
    AERROR << "cyber_PyParameter_clt_get_parameter_list pyparam_clt is null!";
    Py_INCREF(Py_None);
    return Py_None;
  }

  std::vector<Parameter> param_list;
  pyparam_clt->list_parameters(&param_list);

  PyObject* pyobj_list = PyList_New(param_list.size());
  size_t pos = 0;
  for (auto& param : param_list) {
    Parameter* param_ptr = new Parameter(param);
    PyParameter* pyparameter = new PyParameter(param_ptr);
    PyObject* pyobj_param =
        PyCapsule_New(pyparameter, "apollo_cybertron_pyparameter", nullptr);
    PyList_SetItem(pyobj_list, pos, pyobj_param);
    pos++;
  }

  return pyobj_list;
}

// PyParameterServer
PyObject* cyber_new_PyParameterServer(PyObject* self, PyObject* args) {
  PyObject* pyobj_node = nullptr;
  if (!PyArg_ParseTuple(args,
                        const_cast<char*>("O:cyber_new_PyParameterServer"),
                        &pyobj_node)) {
    AERROR << "cyber_new_PyParameterServer parsetuple failed!";
    Py_INCREF(Py_None);
    return Py_None;
  }

  PyNode* pynode = PyObjectToPtr<PyNode*>(pyobj_node, "apollo_cyber_pynode");
  if (nullptr == pynode) {
    AERROR << "pynode ptr is null!";
    Py_INCREF(Py_None);
    return Py_None;
  }

  auto node = pynode->get_node();
  if (nullptr == node) {
    AERROR << "node ptr is null!";
    Py_INCREF(Py_None);
    return Py_None;
  }

  PyParameterServer* pyparameter_srv = new PyParameterServer(node);
  return PyCapsule_New(pyparameter_srv, "apollo_cybertron_pyparameterserver",
                       nullptr);
}

PyObject* cyber_delete_PyParameterServer(PyObject* self, PyObject* args) {
  PyObject* pyobj_param = nullptr;
  if (!PyArg_ParseTuple(args,
                        const_cast<char*>("O:cyber_delete_PyParameterServer"),
                        &pyobj_param)) {
    AERROR << "cyber_delete_PyParameterServer parsetuple failed!";
    Py_INCREF(Py_None);
    return Py_None;
  }

  PyParameterServer* pyparameter_srv = reinterpret_cast<PyParameterServer*>(
      PyCapsule_GetPointer(pyobj_param, "apollo_cybertron_pyparameterserver"));
  if (nullptr == pyparameter_srv) {
    AERROR << "cyber_delete_PyParameterServer:pyparameter_srv ptr is null!";
    Py_INCREF(Py_None);
    return Py_None;
  }
  delete pyparameter_srv;
  Py_INCREF(Py_None);
  return Py_None;
}

PyObject* cyber_PyParameter_srv_set_parameter(PyObject* self, PyObject* args) {
  PyObject* pyobj_param_srv = nullptr;
  PyObject* pyobj_param = nullptr;
  if (!PyArg_ParseTuple(args,
                        const_cast<char*>("OO:cyber_PyParameter_set_parameter"),
                        &pyobj_param_srv, &pyobj_param)) {
    AERROR << "cyber_PyParameter_set_parameter parsetuple failed!";
    Py_INCREF(Py_None);
    return Py_None;
  }

  PyParameterServer* pyparam_srv = PyObjectToPtr<PyParameterServer*>(
      pyobj_param_srv, "apollo_cybertron_pyparameterserver");
  if (nullptr == pyparam_srv) {
    AERROR << "pyparam_srv ptr is null!";
    Py_INCREF(Py_None);
    return Py_None;
  }

  PyParameter* pyparam =
      PyObjectToPtr<PyParameter*>(pyobj_param, "apollo_cybertron_pyparameter");
  if (nullptr == pyparam) {
    AERROR << "pyparam ptr is null!";
    Py_INCREF(Py_None);
    return Py_None;
  }

  pyparam_srv->set_parameter(pyparam->get_param());
  Py_INCREF(Py_None);
  return Py_None;
}

PyObject* cyber_PyParameter_srv_get_parameter(PyObject* self, PyObject* args) {
  char* name = nullptr;
  Py_ssize_t len = 0;
  PyObject* pyobj_param_srv = nullptr;
  if (!PyArg_ParseTuple(
          args, const_cast<char*>("Os#:cyber_PyParameter_get_parameter"),
          &pyobj_param_srv, &name, &len)) {
    AERROR << "cyber_PyParameter_get_parameter parsetuple failed!";
    Py_INCREF(Py_None);
    return Py_None;
  }
  PyParameterServer* pyparam_srv = PyObjectToPtr<PyParameterServer*>(
      pyobj_param_srv, "apollo_cybertron_pyparameterserver");
  if (nullptr == pyparam_srv) {
    AERROR << "pyparam_srv ptr is null!";
    Py_INCREF(Py_None);
    return Py_None;
  }

  Parameter* param = new Parameter();
  std::string str_param = std::string(name, len);
  if (!pyparam_srv->get_parameter(str_param, param)) {
    AERROR << "pyparam_srv get_parameter is false!";
    Py_INCREF(Py_None);
    return Py_None;
  }

  PyParameter* pyparameter = new PyParameter(param);
  return PyCapsule_New(pyparameter, "apollo_cybertron_pyparameter", nullptr);
}

PyObject* cyber_PyParameter_srv_get_parameter_list(PyObject* self,
                                                   PyObject* args) {
  PyObject* pyobj_param_srv = nullptr;
  if (!PyArg_ParseTuple(
          args, const_cast<char*>("O:cyber_PyParameter_srv_get_parameter_list"),
          &pyobj_param_srv)) {
    AERROR
        << "cyber_PyParameter_srv_get_parameter_list:PyArg_ParseTuple failed!";
    Py_INCREF(Py_None);
    return Py_None;
  }

  auto* pyparam_srv = reinterpret_cast<PyParameterServer*>(PyCapsule_GetPointer(
      pyobj_param_srv, "apollo_cybertron_pyparameterserver"));
  if (nullptr == pyparam_srv) {
    AERROR << "cyber_PyParameter_srv_get_parameter_list pyparam_srv is null!";
    Py_INCREF(Py_None);
    return Py_None;
  }

  std::vector<Parameter> param_list;
  pyparam_srv->list_parameters(&param_list);

  PyObject* pyobj_list = PyList_New(param_list.size());
  size_t pos = 0;
  for (auto& param : param_list) {
    Parameter* param_ptr = new Parameter(param);
    PyParameter* pyparameter = new PyParameter(param_ptr);
    PyObject* pyobj_param =
        PyCapsule_New(pyparameter, "apollo_cybertron_pyparameter", nullptr);
    PyList_SetItem(pyobj_list, pos, pyobj_param);
    pos++;
  }

  return pyobj_list;
}

static PyMethodDef _cyber_parameter_methods[] = {
    {"new_PyParameter_noparam", cyber_new_PyParameter_noparam, METH_NOARGS, ""},
    {"delete_PyParameter", cyber_delete_PyParameter, METH_VARARGS, ""},
    {"new_PyParameter_int", cyber_new_PyParameter_int, METH_VARARGS, ""},
    {"new_PyParameter_double", cyber_new_PyParameter_double, METH_VARARGS, ""},
    {"new_PyParameter_string", cyber_new_PyParameter_string, METH_VARARGS, ""},
    {"PyParameter_type_name", cyber_PyParameter_type_name, METH_VARARGS, ""},
    {"PyParameter_descriptor", cyber_PyParameter_descriptor, METH_VARARGS, ""},
    {"PyParameter_name", cyber_PyParameter_name, METH_VARARGS, ""},
    {"PyParameter_debug_string", cyber_PyParameter_debug_string, METH_VARARGS,
     ""},
    {"PyParameter_as_string", cyber_PyParameter_as_string, METH_VARARGS, ""},
    {"PyParameter_as_double", cyber_PyParameter_as_double, METH_VARARGS, ""},
    {"PyParameter_as_int64", cyber_PyParameter_as_int64, METH_VARARGS, ""},

    {"new_PyParameterClient", cyber_new_PyParameterClient, METH_VARARGS, ""},
    {"delete_PyParameterClient", cyber_delete_PyParameterClient, METH_VARARGS,
     ""},
    {"PyParameter_clt_set_parameter", cyber_PyParameter_clt_set_parameter,
     METH_VARARGS, ""},
    {"PyParameter_clt_get_parameter", cyber_PyParameter_clt_get_parameter,
     METH_VARARGS, ""},
    {"PyParameter_clt_get_parameter_list",
     cyber_PyParameter_clt_get_parameter_list, METH_VARARGS, ""},

    {"new_PyParameterServer", cyber_new_PyParameterServer, METH_VARARGS, ""},
    {"delete_PyParameterServer", cyber_delete_PyParameterServer, METH_VARARGS,
     ""},
    {"PyParameter_srv_set_parameter", cyber_PyParameter_srv_set_parameter,
     METH_VARARGS, ""},
    {"PyParameter_srv_get_parameter", cyber_PyParameter_srv_get_parameter,
     METH_VARARGS, ""},
    {"PyParameter_srv_get_parameter_list",
     cyber_PyParameter_srv_get_parameter_list, METH_VARARGS, ""},

    {nullptr, nullptr, 0, nullptr} /* sentinel */
};

/// Init function of this module
PyMODINIT_FUNC PyInit__cyber_parameter_wrapper(void) {
  static struct PyModuleDef module_def = {
      PyModuleDef_HEAD_INIT,
      "_cyber_parameter_wrapper",  // Module name.
      "CyberParameter module",     // Module doc.
      -1,                          // Module size.
      _cyber_parameter_methods,    // Module methods.
      nullptr,
      nullptr,
      nullptr,
      nullptr,
  };

  return PyModule_Create(&module_def);
}
