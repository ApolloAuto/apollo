/******************************************************************************
 * Copyright 2018 The Apollo Authors. All Rights Reserved.
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
#include <string>
#include <vector>

#include "cyber/py_wrapper/py_topology_manager.h"

#define PYOBJECT_NULL_STRING PyString_FromStringAndSize("", 0)
using apollo::cyber::PyChannelManager;
using apollo::cyber::PyNodeManager;
template <typename T>
T PyObjectToPtr(PyObject *pyobj, const std::string &type_ptr) {
  T obj_ptr = (T)PyCapsule_GetPointer(pyobj, type_ptr.c_str());
  if (obj_ptr == nullptr) {
    AERROR << "PyObjectToPtr failed,type->" << type_ptr << "pyobj: " << pyobj;
  }
  return obj_ptr;
}
// Node Manage Interface
PyObject *cyber_new_Node_Manager(PyObject *self, PyObject *args) {
  PyNodeManager *pyobj_node_manager = new PyNodeManager();
  PyObject *py_node_manager;
  py_node_manager =
      PyCapsule_New(pyobj_node_manager, "apollo_cyber_node_manager", nullptr);
  return py_node_manager;
}

static PyObject *cyber_Py_HasNode(PyObject *self, PyObject *args) {
  char *node_name = nullptr;
  PyObject *py_node_manager_obj = 0;
  Py_ssize_t len = 0;
  PyNodeManager *py_node_manager = 0;
  if (!PyArg_ParseTuple(args, const_cast<char *>("Os#:cyber_Py_HasNode"),
                        &py_node_manager_obj, &node_name, &len)) {
    AERROR << "cyber_Py_HasNode:cyber_Py_HasNode failed!";
    Py_INCREF(Py_None);
    return Py_None;
  }
  std::string data_str(node_name, len);
  py_node_manager = reinterpret_cast<PyNodeManager *>(
      PyCapsule_GetPointer(py_node_manager_obj, "apollo_cyber_node_manager"));
  bool hasNode = py_node_manager->HasNode(data_str);
  if (hasNode) {
    Py_RETURN_TRUE;
  } else {
    Py_RETURN_FALSE;
  }
}

static PyObject *cyber_Py_GetNodeList(PyObject *self, PyObject *args) {
  PyObject *py_node_manager_obj = 0;
  PyNodeManager *py_node_manager = 0;
  if (!PyArg_ParseTuple(args, const_cast<char *>("O:cyber_Py_GetNodeList"),
                        &py_node_manager_obj)) {
    AERROR << "cyber_Py_GetNodeList:cyber_Py_GetNodeList failed!";
    Py_INCREF(Py_None);
    return Py_None;
  }
  py_node_manager = reinterpret_cast<PyNodeManager *>(
      PyCapsule_GetPointer(py_node_manager_obj, "apollo_cyber_node_manager"));
  PyObject *node_list = py_node_manager->GetNodeList();
  return node_list;
}

static PyObject *cyber_Py_ShowNodeInfo(PyObject *self, PyObject *args) {
  char *node_name = nullptr;
  PyObject *py_node_manager_obj = 0;
  Py_ssize_t len = 0;
  PyNodeManager *py_node_manager = 0;
  if (!PyArg_ParseTuple(args, const_cast<char *>("Os#:cyber_Py_ShowNodeInfo"),
                        &py_node_manager_obj, &node_name, &len)) {
    AERROR << "cyber_Py_ShowNodeInfo:cyber_Py_ShowNodeInfo failed!";
    Py_INCREF(Py_None);
    return Py_None;
  }
  std::string data_str(node_name, len);
  py_node_manager = reinterpret_cast<PyNodeManager *>(
      PyCapsule_GetPointer(py_node_manager_obj, "apollo_cyber_node_manager"));
  PyObject *showNodeInfo = py_node_manager->ShowNodeInfo(data_str);
  return showNodeInfo;
}

PyObject *cyber_new_Channel_Manager(PyObject *self, PyObject *args) {
  PyChannelManager *pyobj_channel_manager = new PyChannelManager();
  PyObject *py_channel_manager;
  py_channel_manager = PyCapsule_New(pyobj_channel_manager,
                                     "apollo_cyber_channel_manager", nullptr);
  return py_channel_manager;
}

static PyObject *cyber_Py_GetChannelList(PyObject *self, PyObject *args) {
  PyObject *py_channel_manager_obj = 0;
  PyChannelManager *py_channel_manager = 0;
  if (!PyArg_ParseTuple(args, const_cast<char *>("O:cyber_Py_GetChannelList"),
                        &py_channel_manager_obj)) {
    AERROR << "cyber_Py_GetChannelList:cyber_Py_GetChannelList failed!";
    Py_INCREF(Py_None);
    return Py_None;
  }
  py_channel_manager =
      reinterpret_cast<PyChannelManager *>(PyCapsule_GetPointer(
          py_channel_manager_obj, "apollo_cyber_channel_manager"));
  PyObject *channel_list = py_channel_manager->GetChannelList();
  return channel_list;
}

static PyObject *cyber_Py_GetReaderList(PyObject *self, PyObject *args) {
  PyObject *py_channel_manager_obj = 0;
  PyChannelManager *py_channel_manager = 0;
  if (!PyArg_ParseTuple(args, const_cast<char *>("O:cyber_Py_GetReaderList"),
                        &py_channel_manager_obj)) {
    AERROR << "cyber_Py_GetReaderList:cyber_Py_GetReaderList failed!";
    Py_INCREF(Py_None);
    return Py_None;
  }
  py_channel_manager =
      reinterpret_cast<PyChannelManager *>(PyCapsule_GetPointer(
          py_channel_manager_obj, "apollo_cyber_channel_manager"));
  PyObject *reader_list = py_channel_manager->GetReaderList();
  return reader_list;
}

static PyObject *cyber_Py_GetWriterList(PyObject *self, PyObject *args) {
  PyObject *py_channel_manager_obj = 0;
  PyChannelManager *py_channel_manager = 0;
  if (!PyArg_ParseTuple(args, const_cast<char *>("O:cyber_Py_GetWriterList"),
                        &py_channel_manager_obj)) {
    AERROR << "cyber_Py_GetWriterList:cyber_Py_GetWriterList failed!";
    Py_INCREF(Py_None);
    return Py_None;
  }
  py_channel_manager =
      reinterpret_cast<PyChannelManager *>(PyCapsule_GetPointer(
          py_channel_manager_obj, "apollo_cyber_channel_manager"));
  PyObject *writer_list = py_channel_manager->GetWriterList();
  return writer_list;
}

static PyObject *cyber_Py_GetWritersOfNode(PyObject *self, PyObject *args) {
  PyObject *py_channel_manager_obj = 0;
  char *node_name = nullptr;
  Py_ssize_t len = 0;
  PyChannelManager *py_channel_manager = 0;
  if (!PyArg_ParseTuple(args, const_cast<char *>("Os#:cyber_Py_GetChannelList"),
                        &py_channel_manager_obj, &node_name, &len)) {
    AERROR << "cyber_Py_GetNodeWriters:cyber_Py_GetNodeWriters failed!";
    Py_INCREF(Py_None);
    return Py_None;
  }
  std::string data_str(node_name, len);
  py_channel_manager =
      reinterpret_cast<PyChannelManager *>(PyCapsule_GetPointer(
          py_channel_manager_obj, "apollo_cyber_channel_manager"));
  PyObject *node_writers = py_channel_manager->GetWritersOfNode(data_str);
  return node_writers;
}

static PyObject *cyber_Py_GetReadersOfNode(PyObject *self, PyObject *args) {
  PyObject *py_channel_manager_obj = 0;
  char *node_name = nullptr;
  Py_ssize_t len = 0;
  PyChannelManager *py_channel_manager = 0;
  if (!PyArg_ParseTuple(args, const_cast<char *>("Os#:cyber_Py_GetChannelList"),
                        &py_channel_manager_obj, &node_name, &len)) {
    AERROR << "cyber_Py_GetNodeReaders:cyber_Py_GetNodeReaders failed!";
    Py_INCREF(Py_None);
    return Py_None;
  }
  std::string data_str(node_name, len);
  py_channel_manager =
      reinterpret_cast<PyChannelManager *>(PyCapsule_GetPointer(
          py_channel_manager_obj, "apollo_cyber_channel_manager"));
  PyObject *node_readers = py_channel_manager->GetReadersOfNode(data_str);
  return node_readers;
}

static PyObject *cyber_Py_ShowChannelInfo(PyObject *self, PyObject *args) {
  char *channel_name = nullptr;
  PyObject *py_channel_manager_obj = 0;
  Py_ssize_t len = 0;
  PyChannelManager *py_channel_manager = 0;
  if (!PyArg_ParseTuple(args,
                        const_cast<char *>("Os#:cyber_Py_ShowChannelInfo"),
                        &py_channel_manager_obj, &channel_name, &len)) {
    AERROR << "cyber_Py_ShowChannelInfo:cyber_Py_ShowChannelInfo failed!";
    Py_INCREF(Py_None);
    return Py_None;
  }
  std::string data_str(channel_name, len);
  py_channel_manager =
      reinterpret_cast<PyChannelManager *>(PyCapsule_GetPointer(
          py_channel_manager_obj, "apollo_cyber_channel_manager"));
  PyObject *showChannelInfo = py_channel_manager->ShowChannelInfo(data_str);
  return showChannelInfo;
}
/////////////////////////////////////////////////////////////////////
//// global for whole page, init module
/////////////////////////////////////////////////////////////////////
static PyMethodDef _topology_manager_methods[] = {
    // PyWriter fun
    {"new_Node_Manager", cyber_new_Node_Manager, METH_VARARGS, ""},
    {"Py_HasNode", cyber_Py_HasNode, METH_VARARGS, ""},
    {"Py_GetNodeList", cyber_Py_GetNodeList, METH_VARARGS, ""},
    {"Py_ShowNodeInfo", cyber_Py_ShowNodeInfo, METH_VARARGS, ""},
    {"new_Channel_Manager", cyber_new_Channel_Manager, METH_VARARGS, ""},
    {"Py_GetChannelList", cyber_Py_GetChannelList, METH_VARARGS, ""},
    {"Py_GetReaderList", cyber_Py_GetReaderList, METH_VARARGS, ""},
    {"Py_GetWriterList", cyber_Py_GetWriterList, METH_VARARGS, ""},
    {"Py_GetWritersOfNode", cyber_Py_GetWritersOfNode, METH_VARARGS, ""},
    {"Py_GetReadersOfNode", cyber_Py_GetReadersOfNode, METH_VARARGS, ""},
    {"Py_ShowChannelInfo", cyber_Py_ShowChannelInfo, METH_VARARGS, ""},
    {NULL, NULL, 0, NULL} /* sentinel */
};

/// Init function of this module
PyMODINIT_FUNC init_cyber_topology_manager(void) {
  Py_InitModule("_cyber_topology_manager", _topology_manager_methods);
}
