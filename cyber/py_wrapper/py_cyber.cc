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

#include "cyber/py_wrapper/py_cyber.h"

#if PY_MAJOR_VERSION >= 3
#include <python3.6m/Python.h>
#else
#include <python2.7/Python.h>
#endif

#include <string>
#include <vector>

using apollo::cyber::Node;
using apollo::cyber::PyChannelUtils;
using apollo::cyber::PyClient;
using apollo::cyber::PyNode;
using apollo::cyber::PyReader;
using apollo::cyber::PyService;
using apollo::cyber::PyServiceUtils;
using apollo::cyber::PyWriter;

#if PY_MAJOR_VERSION >= 3
#define PyInt_AsLong PyLong_AsLong
#define PyInt_FromLong PyLong_FromLong
#define PYOBJECT_NULL_STRING PyBytes_FromStringAndSize("", 0)
#define C_STR_TO_PY_BYTES(cstr) \
  PyBytes_FromStringAndSize(cstr.c_str(), cstr.size())
#else
#define PYOBJECT_NULL_STRING PyString_FromStringAndSize("", 0)
#define C_STR_TO_PY_BYTES(cstr) \
  PyString_FromStringAndSize(cstr.c_str(), cstr.size())
#endif

google::protobuf::Message *PyChannelUtils::raw_msg_class_ = nullptr;

static PyObject *cyber_py_init(PyObject *self, PyObject *args) {
  char *data = nullptr;
  Py_ssize_t len = 0;
  if (!PyArg_ParseTuple(args, const_cast<char *>("s#:cyber_py_init"), &data,
                        &len)) {
    AERROR << "cyber_py_init:PyArg_ParseTuple failed!";
    Py_RETURN_FALSE;
  }
  std::string module_name(data, len);
  bool is_init = apollo::cyber::py_init(module_name);
  if (is_init) {
    Py_RETURN_TRUE;
  } else {
    Py_RETURN_FALSE;
  }
}

static PyObject *cyber_py_ok(PyObject *self, PyObject *args) {
  bool is_ok = apollo::cyber::py_ok();
  if (is_ok) {
    Py_RETURN_TRUE;
  } else {
    Py_RETURN_FALSE;
  }
}

static PyObject *cyber_py_shutdown(PyObject *self, PyObject *args) {
  apollo::cyber::py_shutdown();
  Py_INCREF(Py_None);
  return Py_None;
}

static PyObject *cyber_py_is_shutdown(PyObject *self, PyObject *args) {
  bool is_shutdown = apollo::cyber::py_is_shutdown();
  if (is_shutdown) {
    Py_RETURN_TRUE;
  } else {
    Py_RETURN_FALSE;
  }
}

static PyObject *cyber_py_waitforshutdown(PyObject *self, PyObject *args) {
  apollo::cyber::py_waitforshutdown();
  Py_INCREF(Py_None);
  return Py_None;
}

template <typename T>
T PyObjectToPtr(PyObject *pyobj, const std::string &type_ptr) {
  T obj_ptr = (T)PyCapsule_GetPointer(pyobj, type_ptr.c_str());
  if (obj_ptr == nullptr) {
    AERROR << "PyObjectToPtr failed,type->" << type_ptr << "pyobj: " << pyobj;
  }
  return obj_ptr;
}

PyObject *cyber_new_PyWriter(PyObject *self, PyObject *args) {
  char *channel_name = nullptr;
  char *data_type = nullptr;
  uint32_t qos_depth = 1;

  PyObject *node_pyobj = nullptr;
  if (!PyArg_ParseTuple(args, const_cast<char *>("ssIO:new_PyWriter"),
                        &channel_name, &data_type, &qos_depth, &node_pyobj)) {
    Py_INCREF(Py_None);
    return Py_None;
  }

  Node *node = reinterpret_cast<Node *>(
      PyCapsule_GetPointer(node_pyobj, "apollo_cyber_pynode"));
  if (nullptr == node) {
    AERROR << "node is null";
    Py_INCREF(Py_None);
    return Py_None;
  }

  PyWriter *writer =
      new PyWriter((std::string const &)*channel_name,
                   (std::string const &)*data_type, qos_depth, node);
  PyObject *pyobj_writer =
      PyCapsule_New(writer, "apollo_cyber_pywriter", nullptr);
  return pyobj_writer;
}

PyObject *cyber_delete_PyWriter(PyObject *self, PyObject *args) {
  PyObject *writer_py = nullptr;
  if (!PyArg_ParseTuple(args, const_cast<char *>("O:delete_PyWriter"),
                        &writer_py)) {
    Py_INCREF(Py_None);
    return Py_None;
  }

  PyWriter *writer = reinterpret_cast<PyWriter *>(
      PyCapsule_GetPointer(writer_py, "apollo_cyber_pywriter"));
  delete writer;
  Py_INCREF(Py_None);
  return Py_None;
}

PyObject *cyber_PyWriter_write(PyObject *self, PyObject *args) {
  PyObject *pyobj_writer = nullptr;
  char *data = nullptr;
  Py_ssize_t len = 0;
  if (!PyArg_ParseTuple(args, const_cast<char *>("Os#:cyber_PyWriter_write"),
                        &pyobj_writer, &data, &len)) {
    AERROR << "cyber_PyWriter_write:cyber_PyWriter_write failed!";
    return PyInt_FromLong(1);
  }

  PyWriter *writer =
      PyObjectToPtr<PyWriter *>(pyobj_writer, "apollo_cyber_pywriter");

  if (nullptr == writer) {
    AERROR << "cyber_PyWriter_write:writer ptr is null!";
    return PyInt_FromLong(1);
  }

  std::string data_str(data, len);
  int ret = writer->write(data_str);
  return PyInt_FromLong(ret);
}

PyObject *cyber_new_PyReader(PyObject *self, PyObject *args) {
  char *channel_name = nullptr;
  char *data_type = nullptr;
  PyObject *node_pyobj = nullptr;
  if (!PyArg_ParseTuple(args, const_cast<char *>("ssO:new_PyReader"),
                        &channel_name, &data_type, &node_pyobj)) {
    Py_INCREF(Py_None);
    return Py_None;
  }

  Node *node = reinterpret_cast<Node *>(
      PyCapsule_GetPointer(node_pyobj, "apollo_cyber_pynode"));
  if (!node) {
    AERROR << "node is null";
    Py_INCREF(Py_None);
    return Py_None;
  }

  PyReader *reader = new PyReader((std::string const &)*channel_name,
                                  (std::string const &)*data_type, node);
  PyObject *pyobj_reader =
      PyCapsule_New(reader, "apollo_cyber_pyreader", nullptr);
  return pyobj_reader;
}

PyObject *cyber_delete_PyReader(PyObject *self, PyObject *args) {
  PyObject *reader_py = nullptr;
  if (!PyArg_ParseTuple(args, const_cast<char *>("O:delete_PyReader"),
                        &reader_py)) {
    Py_INCREF(Py_None);
    return Py_None;
  }

  PyReader *reader = reinterpret_cast<PyReader *>(
      PyCapsule_GetPointer(reader_py, "apollo_cyber_pyreader"));
  delete reader;
  Py_INCREF(Py_None);
  return Py_None;
}

PyObject *cyber_PyReader_read(PyObject *self, PyObject *args) {
  PyObject *pyobj_reader = nullptr;
  PyObject *pyobj_iswait = nullptr;

  if (!PyArg_ParseTuple(args, const_cast<char *>("OO:cyber_PyReader_read"),
                        &pyobj_reader, &pyobj_iswait)) {
    AERROR << "cyber_PyReader_read:PyArg_ParseTuple failed!";
    Py_INCREF(Py_None);
    return Py_None;
  }
  PyReader *reader =
      PyObjectToPtr<PyReader *>(pyobj_reader, "apollo_cyber_pyreader");
  if (nullptr == reader) {
    AERROR << "cyber_PyReader_read:PyReader ptr is null!";
    Py_INCREF(Py_None);
    return Py_None;
  }

  int r = PyObject_IsTrue(pyobj_iswait);
  if (r == -1) {
    AERROR << "cyber_PyReader_read:pyobj_iswait is error!";
    Py_INCREF(Py_None);
    return Py_None;
  }

  bool wait = (r == 1);

  const std::string reader_ret = reader->read(wait);
  return C_STR_TO_PY_BYTES(reader_ret);
}

PyObject *cyber_PyReader_register_func(PyObject *self, PyObject *args) {
  PyObject *pyobj_regist_fun = nullptr;
  PyObject *pyobj_reader = nullptr;

  int (*callback_fun)(char const *) = (int (*)(char const *))0;

  if (!PyArg_ParseTuple(args, const_cast<char *>("OO:PyReader_register_func"),
                        &pyobj_reader, &pyobj_regist_fun)) {
    Py_INCREF(Py_None);
    return Py_None;
  }

  PyReader *reader =
      PyObjectToPtr<PyReader *>(pyobj_reader, "apollo_cyber_pyreader");
  callback_fun = (int (*)(const char *i))PyInt_AsLong(pyobj_regist_fun);
  if (reader) {
    AINFO << "reader regist fun";
    reader->register_func(callback_fun);
  }

  Py_INCREF(Py_None);
  return Py_None;
}

PyObject *cyber_new_PyClient(PyObject *self, PyObject *args) {
  char *channel_name = nullptr;
  char *data_type = nullptr;
  PyObject *node_pyobj = nullptr;
  if (!PyArg_ParseTuple(args, const_cast<char *>("ssO:new_PyClient"),
                        &channel_name, &data_type, &node_pyobj)) {
    Py_INCREF(Py_None);
    return Py_None;
  }

  Node *node = reinterpret_cast<Node *>(
      PyCapsule_GetPointer(node_pyobj, "apollo_cyber_pynode"));
  if (!node) {
    AERROR << "node is null";
    Py_INCREF(Py_None);
    return Py_None;
  }

  PyClient *client = new PyClient((std::string const &)*channel_name,
                                  (std::string const &)*data_type, node);
  PyObject *pyobj_client =
      PyCapsule_New(client, "apollo_cyber_pyclient", nullptr);
  return pyobj_client;
}

PyObject *cyber_delete_PyClient(PyObject *self, PyObject *args) {
  PyObject *client_py = nullptr;
  if (!PyArg_ParseTuple(args, const_cast<char *>("O:delete_PyClient"),
                        &client_py)) {
    Py_INCREF(Py_None);
    return Py_None;
  }

  PyClient *client = reinterpret_cast<PyClient *>(
      PyCapsule_GetPointer(client_py, "apollo_cyber_pyclient"));
  delete client;
  Py_INCREF(Py_None);
  return Py_None;
}

PyObject *cyber_PyClient_send_request(PyObject *self, PyObject *args) {
  PyObject *pyobj_client = nullptr;
  char *data = nullptr;
  Py_ssize_t len = 0;
  if (!PyArg_ParseTuple(args, const_cast<char *>("Os#:PyClient_send_request"),
                        &pyobj_client, &data, &len)) {
    AERROR << "cyber_PyClient_send_request:PyArg_ParseTuple failed!";
    return PYOBJECT_NULL_STRING;
  }

  PyClient *client =
      PyObjectToPtr<PyClient *>(pyobj_client, "apollo_cyber_pyclient");

  if (nullptr == client) {
    AERROR << "cyber_PyClient_send_request:client ptr is null!";
    return PYOBJECT_NULL_STRING;
  }

  std::string data_str(data, len);
  ADEBUG << "c++:PyClient_send_request data->[ " << data_str << "]";
  const std::string response_str =
      client->send_request((std::string const &)data_str);
  ADEBUG << "c++:response data->[ " << response_str << "]";
  return C_STR_TO_PY_BYTES(response_str);
}

PyObject *cyber_new_PyService(PyObject *self, PyObject *args) {
  char *channel_name = nullptr;
  char *data_type = nullptr;
  PyObject *node_pyobj = nullptr;
  if (!PyArg_ParseTuple(args, const_cast<char *>("ssO:new_PyService"),
                        &channel_name, &data_type, &node_pyobj)) {
    Py_INCREF(Py_None);
    return Py_None;
  }

  Node *node = reinterpret_cast<Node *>(
      PyCapsule_GetPointer(node_pyobj, "apollo_cyber_pynode"));
  if (!node) {
    AERROR << "node is null";
    Py_INCREF(Py_None);
    return Py_None;
  }

  PyService *service = new PyService((std::string const &)*channel_name,
                                     (std::string const &)*data_type, node);
  PyObject *pyobj_service =
      PyCapsule_New(service, "apollo_cyber_pyservice", nullptr);
  return pyobj_service;
}

PyObject *cyber_delete_PyService(PyObject *self, PyObject *args) {
  PyObject *pyobj_service = nullptr;
  if (!PyArg_ParseTuple(args, const_cast<char *>("O:delete_PyService"),
                        &pyobj_service)) {
    Py_INCREF(Py_None);
    return Py_None;
  }

  PyService *service = reinterpret_cast<PyService *>(
      PyCapsule_GetPointer(pyobj_service, "apollo_cyber_pyservice"));
  delete service;
  Py_INCREF(Py_None);
  return Py_None;
}

PyObject *cyber_PyService_register_func(PyObject *self, PyObject *args) {
  PyObject *pyobj_regist_fun = nullptr;
  PyObject *pyobj_service = nullptr;

  int (*callback_fun)(char const *) = (int (*)(char const *))0;

  if (!PyArg_ParseTuple(args, const_cast<char *>("OO:PyService_register_func"),
                        &pyobj_service, &pyobj_regist_fun)) {
    Py_INCREF(Py_None);
    return Py_None;
  }

  PyService *service =
      PyObjectToPtr<PyService *>(pyobj_service, "apollo_cyber_pyservice");
  callback_fun = (int (*)(const char *i))PyInt_AsLong(pyobj_regist_fun);
  if (service) {
    AINFO << "service regist fun";
    service->register_func(callback_fun);
  }

  Py_INCREF(Py_None);
  return Py_None;
}

PyObject *cyber_PyService_read(PyObject *self, PyObject *args) {
  PyObject *pyobj_service = nullptr;
  if (!PyArg_ParseTuple(args, const_cast<char *>("O:cyber_PyService_read"),
                        &pyobj_service)) {
    AERROR << "cyber_PyService_read:PyArg_ParseTuple failed!";
    return PYOBJECT_NULL_STRING;
  }
  PyService *service =
      PyObjectToPtr<PyService *>(pyobj_service, "apollo_cyber_pyservice");
  if (nullptr == service) {
    AERROR << "cyber_PyService_read:service ptr is null!";
    return PYOBJECT_NULL_STRING;
  }

  const std::string reader_ret = service->read();
  ADEBUG << "c++:PyService_read -> " << reader_ret;
  return C_STR_TO_PY_BYTES(reader_ret);
}

PyObject *cyber_PyService_write(PyObject *self, PyObject *args) {
  PyObject *pyobj_service = nullptr;
  char *data = nullptr;
  Py_ssize_t len = 0;
  if (!PyArg_ParseTuple(args, const_cast<char *>("Os#:cyber_PyService_write"),
                        &pyobj_service, &data, &len)) {
    AERROR << "cyber_PyService_write:PyArg_ParseTuple failed!";
    return PyInt_FromLong(1);
  }

  PyService *service =
      PyObjectToPtr<PyService *>(pyobj_service, "apollo_cyber_pyservice");

  if (nullptr == service) {
    AERROR << "cyber_PyService_write:writer ptr is null!";
    return PyInt_FromLong(1);
  }

  std::string data_str(data, len);
  ADEBUG << "c++:PyService_write data->[ " << data_str << "]";
  int ret = service->write((std::string const &)data_str);
  return PyInt_FromLong(ret);
}

PyObject *cyber_new_PyNode(PyObject *self, PyObject *args) {
  char *node_name = nullptr;
  if (!PyArg_ParseTuple(args, const_cast<char *>("s:new_PyNode"), &node_name)) {
    Py_INCREF(Py_None);
    return Py_None;
  }

  PyNode *node = new PyNode((std::string const &)node_name);
  PyObject *pyobj_node = PyCapsule_New(node, "apollo_cyber_pynode", nullptr);
  return pyobj_node;
}

PyObject *cyber_delete_PyNode(PyObject *self, PyObject *args) {
  PyObject *pyobj_node = nullptr;
  if (!PyArg_ParseTuple(args, const_cast<char *>("O:delete_PyNode"),
                        &pyobj_node)) {
    Py_INCREF(Py_None);
    return Py_None;
  }

  PyNode *node = reinterpret_cast<PyNode *>(
      PyCapsule_GetPointer(pyobj_node, "apollo_cyber_pynode"));
  delete node;
  Py_INCREF(Py_None);
  return Py_None;
}

PyObject *cyber_PyNode_create_writer(PyObject *self, PyObject *args) {
  PyObject *pyobj_node = nullptr;
  char *channel_name = nullptr;
  char *type_name = nullptr;
  uint32_t qos_depth = 1;

  if (!PyArg_ParseTuple(args, const_cast<char *>("OssI:PyNode_create_writer"),
                        &pyobj_node, &channel_name, &type_name, &qos_depth)) {
    AERROR << "cyber_PyNode_create_writer:PyArg_ParseTuple failed!";
    Py_INCREF(Py_None);
    return Py_None;
  }

  PyNode *node = PyObjectToPtr<PyNode *>(pyobj_node, "apollo_cyber_pynode");
  if (nullptr == node) {
    AERROR << "cyber_PyNode_create_writer:node ptr is null!";
    Py_INCREF(Py_None);
    return Py_None;
  }

  PyWriter *writer = reinterpret_cast<PyWriter *>(
      (node->create_writer((std::string const &)channel_name,
                           (std::string const &)type_name, qos_depth)));

  if (nullptr == writer) {
    AERROR << "cyber_PyNode_create_writer:writer is null!";
    Py_INCREF(Py_None);
    return Py_None;
  }
  PyObject *pyobj_writer =
      PyCapsule_New(writer, "apollo_cyber_pywriter", nullptr);
  return pyobj_writer;
}

PyObject *cyber_PyNode_create_reader(PyObject *self, PyObject *args) {
  char *channel_name = nullptr;
  char *type_name = nullptr;
  PyObject *pyobj_node = nullptr;

  if (!PyArg_ParseTuple(args, const_cast<char *>("Oss:PyNode_create_reader"),
                        &pyobj_node, &channel_name, &type_name)) {
    AERROR << "PyNode_create_reader:PyArg_ParseTuple failed!";
    Py_INCREF(Py_None);
    return Py_None;
  }

  PyNode *node = PyObjectToPtr<PyNode *>(pyobj_node, "apollo_cyber_pynode");
  if (nullptr == node) {
    AERROR << "PyNode_create_reader:node ptr is null!";
    Py_INCREF(Py_None);
    return Py_None;
  }

  PyReader *reader = reinterpret_cast<PyReader *>((node->create_reader(
      (std::string const &)channel_name, (std::string const &)type_name)));
  ACHECK(reader) << "PyReader is NULL!";

  PyObject *pyobj_reader =
      PyCapsule_New(reader, "apollo_cyber_pyreader", nullptr);
  return pyobj_reader;
}

PyObject *cyber_PyNode_create_client(PyObject *self, PyObject *args) {
  char *channel_name = nullptr;
  char *type_name = nullptr;
  PyObject *pyobj_node = nullptr;

  if (!PyArg_ParseTuple(args, const_cast<char *>("Oss:PyNode_create_client"),
                        &pyobj_node, &channel_name, &type_name)) {
    AERROR << "PyNode_create_client:PyArg_ParseTuple failed!";
    Py_INCREF(Py_None);
    return Py_None;
  }

  PyNode *node = PyObjectToPtr<PyNode *>(pyobj_node, "apollo_cyber_pynode");
  if (nullptr == node) {
    AERROR << "PyNode_create_client:node ptr is null!";
    Py_INCREF(Py_None);
    return Py_None;
  }

  PyClient *client = reinterpret_cast<PyClient *>((node->create_client(
      (std::string const &)channel_name, (std::string const &)type_name)));
  PyObject *pyobj_client =
      PyCapsule_New(client, "apollo_cyber_pyclient", nullptr);

  return pyobj_client;
}

PyObject *cyber_PyNode_create_service(PyObject *self, PyObject *args) {
  char *channel_name = nullptr;
  char *type_name = nullptr;
  PyObject *pyobj_node = nullptr;

  if (!PyArg_ParseTuple(args,
                        const_cast<char *>("Oss:cyber_PyNode_create_service"),
                        &pyobj_node, &channel_name, &type_name)) {
    AERROR << "cyber_PyNode_create_service:PyArg_ParseTuple failed!";
    Py_INCREF(Py_None);
    return Py_None;
  }

  PyNode *node = PyObjectToPtr<PyNode *>(pyobj_node, "apollo_cyber_pynode");
  if (nullptr == node) {
    AERROR << "cyber_PyNode_create_service:node ptr is null!";
    Py_INCREF(Py_None);
    return Py_None;
  }

  PyService *service = reinterpret_cast<PyService *>((node->create_service(
      (std::string const &)channel_name, (std::string const &)type_name)));
  PyObject *pyobj_service =
      PyCapsule_New(service, "apollo_cyber_pyservice", nullptr);
  return pyobj_service;
}

PyObject *cyber_PyNode_shutdown(PyObject *self, PyObject *args) {
  PyObject *pyobj_node = nullptr;

  if (!PyArg_ParseTuple(args, const_cast<char *>("O:PyNode_shutdown"),
                        &pyobj_node)) {
    AERROR << "cyber_PyNode_shutdown:PyNode_shutdown failed!";
    Py_INCREF(Py_None);
    return Py_None;
  }

  PyNode *node = PyObjectToPtr<PyNode *>(pyobj_node, "apollo_cyber_pynode");
  if (nullptr == node) {
    AERROR << "cyber_PyNode_shutdown:node ptr is null!";
    Py_INCREF(Py_None);
    return Py_None;
  }
  node->shutdown();
  Py_INCREF(Py_None);
  return Py_None;
}

PyObject *cyber_PyNode_register_message(PyObject *self, PyObject *args) {
  PyObject *pyobj_node = nullptr;
  char *desc = nullptr;
  int len = 0;
  if (!PyArg_ParseTuple(args,
                        const_cast<char *>("Os#:cyber_PyNode_register_message"),
                        &pyobj_node, &desc, &len)) {
    AERROR << "cyber_PyNode_register_message: failed!";
    Py_INCREF(Py_None);
    return Py_None;
  }
  PyNode *node = PyObjectToPtr<PyNode *>(pyobj_node, "apollo_cyber_pynode");
  if (nullptr == node) {
    AERROR << "cyber_PyNode_register_message:node ptr is null! desc->" << desc;
    Py_INCREF(Py_None);
    return Py_None;
  }
  std::string desc_str(desc, len);
  node->register_message((std::string const &)desc_str);
  Py_INCREF(Py_None);
  return Py_None;
}

PyObject *cyber_PyChannelUtils_get_msg_type(PyObject *self, PyObject *args) {
  char *channel_name = nullptr;
  Py_ssize_t len = 0;
  unsigned char sleep_s = 0;
  if (!PyArg_ParseTuple(
          args, const_cast<char *>("s#B:cyber_PyChannelUtils_get_msg_type"),
          &channel_name, &len, &sleep_s)) {
    AERROR << "cyber_PyChannelUtils_get_msg_type failed!";
    return PYOBJECT_NULL_STRING;
  }
  std::string channel(channel_name, len);
  const std::string msg_type =
      PyChannelUtils::get_msgtype_by_channelname(channel, sleep_s);
  return C_STR_TO_PY_BYTES(msg_type);
}

PyObject *cyber_PyChannelUtils_get_debugstring_by_msgtype_rawmsgdata(
    PyObject *self, PyObject *args) {
  char *msgtype = nullptr;
  char *rawdata = nullptr;
  Py_ssize_t len = 0;
  if (!PyArg_ParseTuple(
          args,
          const_cast<char *>(
              "ss#:cyber_PyChannelUtils_get_debugstring_by_msgtype_rawmsgdata"),
          &msgtype, &rawdata, &len)) {
    AERROR
        << "cyber_PyChannelUtils_get_debugstring_by_msgtype_rawmsgdata failed!";
    return PYOBJECT_NULL_STRING;
  }
  std::string raw_data(rawdata, len);
  const std::string debug_string =
      PyChannelUtils::get_debugstring_by_msgtype_rawmsgdata(msgtype, raw_data);
  return C_STR_TO_PY_BYTES(debug_string);
}

static PyObject *cyber_PyChannelUtils_get_active_channels(PyObject *self,
                                                          PyObject *args) {
  unsigned char sleep_s = 0;
  if (!PyArg_ParseTuple(
          args,
          const_cast<char *>("B:cyber_PyChannelUtils_get_active_channels"),
          &sleep_s)) {
    AERROR << "cyber_PyChannelUtils_get_active_channels failed!";
    Py_INCREF(Py_None);
    return Py_None;
  }

  std::vector<std::string> channel_list =
      PyChannelUtils::get_active_channels(sleep_s);
  PyObject *pyobj_list = PyList_New(channel_list.size());
  size_t pos = 0;
  for (const std::string &channel : channel_list) {
    PyList_SetItem(pyobj_list, pos, Py_BuildValue("s", channel.c_str()));
    pos++;
  }

  return pyobj_list;
}

// return dict value look like:
// {  'channel1':[atrr1, atrr2, atrr3],
//    'channel2':[atrr1, atrr2]
// }
static PyObject *cyber_PyChannelUtils_get_channels_info(PyObject *self,
                                                        PyObject *args) {
  auto channelsinfo = PyChannelUtils::get_channels_info();
  PyObject *pyobj_channelinfo_dict = PyDict_New();
  for (auto &channelinfo : channelsinfo) {
    std::string channel_name = channelinfo.first;
    PyObject *bld_name = Py_BuildValue("s", channel_name.c_str());
    std::vector<std::string> &roleAttr_list = channelinfo.second;
    PyObject *pyobj_list = PyList_New(roleAttr_list.size());

    size_t pos = 0;
    for (auto &attr : roleAttr_list) {
      PyList_SetItem(pyobj_list, pos,
                     Py_BuildValue("s#", attr.c_str(), attr.size()));
      pos++;
    }
    PyDict_SetItem(pyobj_channelinfo_dict, bld_name, pyobj_list);
    Py_DECREF(bld_name);
  }
  return pyobj_channelinfo_dict;
}

PyObject *cyber_PyNodeUtils_get_active_nodes(PyObject *self, PyObject *args) {
  unsigned char sleep_s = 0;
  if (!PyArg_ParseTuple(
          args, const_cast<char *>("B:cyber_PyNodeUtils_get_active_nodes"),
          &sleep_s)) {
    AERROR << "cyber_PyNodeUtils_get_active_nodes failed!";
    Py_INCREF(Py_None);
    return Py_None;
  }

  std::vector<std::string> nodes_name =
      apollo::cyber::PyNodeUtils::get_active_nodes(sleep_s);
  PyObject *pyobj_list = PyList_New(nodes_name.size());
  size_t pos = 0;
  for (const std::string &name : nodes_name) {
    PyList_SetItem(pyobj_list, pos, Py_BuildValue("s", name.c_str()));
    pos++;
  }

  return pyobj_list;
}

PyObject *cyber_PyNodeUtils_get_node_attr(PyObject *self, PyObject *args) {
  char *node_name = nullptr;
  Py_ssize_t len = 0;
  unsigned char sleep_s = 0;
  if (!PyArg_ParseTuple(
          args, const_cast<char *>("s#B:cyber_PyNodeUtils_get_node_attr"),
          &node_name, &len, &sleep_s)) {
    AERROR << "cyber_PyNodeUtils_get_node_attr failed!";
    Py_INCREF(Py_None);
    return Py_None;
  }
  std::string name(node_name, len);
  const std::string node_attr =
      apollo::cyber::PyNodeUtils::get_node_attr(name, sleep_s);
  return C_STR_TO_PY_BYTES(node_attr);
}

PyObject *cyber_PyNodeUtils_get_readersofnode(PyObject *self, PyObject *args) {
  char *node_name = nullptr;
  Py_ssize_t len = 0;
  unsigned char sleep_s = 0;
  if (!PyArg_ParseTuple(
          args, const_cast<char *>("s#B:cyber_PyNodeUtils_get_readersofnode"),
          &node_name, &len, &sleep_s)) {
    AERROR << "cyber_PyNodeUtils_get_readersofnode failed!";
    Py_INCREF(Py_None);
    return Py_None;
  }
  std::string name(node_name, len);
  std::vector<std::string> readers_channel =
      apollo::cyber::PyNodeUtils::get_readersofnode(name, sleep_s);
  PyObject *pyobj_list = PyList_New(readers_channel.size());
  size_t pos = 0;
  for (const std::string &channel : readers_channel) {
    PyList_SetItem(pyobj_list, pos, Py_BuildValue("s", channel.c_str()));
    pos++;
  }

  return pyobj_list;
}

PyObject *cyber_PyNodeUtils_get_writersofnode(PyObject *self, PyObject *args) {
  char *node_name = nullptr;
  Py_ssize_t len = 0;
  unsigned char sleep_s = 0;
  if (!PyArg_ParseTuple(
          args, const_cast<char *>("s#B:cyber_PyNodeUtils_get_writersofnode"),
          &node_name, &len, &sleep_s)) {
    AERROR << "cyber_PyNodeUtils_get_writersofnode failed!";
    Py_INCREF(Py_None);
    return Py_None;
  }
  std::string name(node_name, len);
  std::vector<std::string> writers_channel =
      apollo::cyber::PyNodeUtils::get_writersofnode(name, sleep_s);
  PyObject *pyobj_list = PyList_New(writers_channel.size());
  size_t pos = 0;
  for (const std::string &channel : writers_channel) {
    PyList_SetItem(pyobj_list, pos, Py_BuildValue("s", channel.c_str()));
    pos++;
  }

  return pyobj_list;
}

PyObject *cyber_PyServiceUtils_get_active_services(PyObject *self,
                                                   PyObject *args) {
  unsigned char sleep_s = 0;
  if (!PyArg_ParseTuple(
          args,
          const_cast<char *>("B:cyber_PyServiceUtils_get_active_services"),
          &sleep_s)) {
    AERROR << "cyber_PyServiceUtils_get_active_services failed!";
    Py_INCREF(Py_None);
    return Py_None;
  }

  std::vector<std::string> services_name =
      PyServiceUtils::get_active_services(sleep_s);
  PyObject *pyobj_list = PyList_New(services_name.size());
  size_t pos = 0;
  for (const std::string &name : services_name) {
    PyList_SetItem(pyobj_list, pos, Py_BuildValue("s", name.c_str()));
    pos++;
  }

  return pyobj_list;
}

PyObject *cyber_PyServiceUtils_get_service_attr(PyObject *self,
                                                PyObject *args) {
  char *srv_name = nullptr;
  Py_ssize_t len = 0;
  unsigned char sleep_s = 0;
  if (!PyArg_ParseTuple(
          args, const_cast<char *>("s#B:cyber_PyServiceUtils_get_service_attr"),
          &srv_name, &len, &sleep_s)) {
    AERROR << "cyber_PyServiceUtils_get_service_attr failed!";
    Py_INCREF(Py_None);
    return Py_None;
  }
  std::string name(srv_name, len);
  const std::string srv_attr = PyServiceUtils::get_service_attr(name, sleep_s);
  return C_STR_TO_PY_BYTES(srv_attr);
}

/////////////////////////////////////////////////////////////////////
//// global for whole page, init module
/////////////////////////////////////////////////////////////////////
static PyMethodDef _cyber_methods[] = {
    // PyInit fun
    {"py_init", cyber_py_init, METH_VARARGS, ""},
    {"py_ok", cyber_py_ok, METH_NOARGS, ""},
    {"py_shutdown", cyber_py_shutdown, METH_NOARGS, ""},
    {"py_is_shutdown", cyber_py_is_shutdown, METH_NOARGS, ""},
    {"py_waitforshutdown", cyber_py_waitforshutdown, METH_NOARGS, ""},

    // PyWriter fun
    {"new_PyWriter", cyber_new_PyWriter, METH_VARARGS, ""},
    {"delete_PyWriter", cyber_delete_PyWriter, METH_VARARGS, ""},
    {"PyWriter_write", cyber_PyWriter_write, METH_VARARGS, ""},

    // PyReader fun
    {"new_PyReader", cyber_new_PyReader, METH_VARARGS, ""},
    {"delete_PyReader", cyber_delete_PyReader, METH_VARARGS, ""},
    {"PyReader_register_func", cyber_PyReader_register_func, METH_VARARGS, ""},
    {"PyReader_read", cyber_PyReader_read, METH_VARARGS, ""},

    // PyClient fun
    {"new_PyClient", cyber_new_PyClient, METH_VARARGS, ""},
    {"delete_PyClient", cyber_delete_PyClient, METH_VARARGS, ""},
    {"PyClient_send_request", cyber_PyClient_send_request, METH_VARARGS, ""},
    // PyService fun
    {"new_PyService", cyber_new_PyService, METH_VARARGS, ""},
    {"delete_PyService", cyber_delete_PyService, METH_VARARGS, ""},
    {"PyService_register_func", cyber_PyService_register_func, METH_VARARGS,
     ""},
    {"PyService_read", cyber_PyService_read, METH_VARARGS, ""},
    {"PyService_write", cyber_PyService_write, METH_VARARGS, ""},
    // PyNode fun
    {"new_PyNode", cyber_new_PyNode, METH_VARARGS, ""},
    {"delete_PyNode", cyber_delete_PyNode, METH_VARARGS, ""},
    {"PyNode_shutdown", cyber_PyNode_shutdown, METH_VARARGS, ""},
    {"PyNode_create_writer", cyber_PyNode_create_writer, METH_VARARGS, ""},
    {"PyNode_register_message", cyber_PyNode_register_message, METH_VARARGS,
     ""},
    {"PyNode_create_reader", cyber_PyNode_create_reader, METH_VARARGS, ""},
    {"PyNode_create_client", cyber_PyNode_create_client, METH_VARARGS, ""},
    {"PyNode_create_service", cyber_PyNode_create_service, METH_VARARGS, ""},

    {"PyChannelUtils_get_msg_type", cyber_PyChannelUtils_get_msg_type,
     METH_VARARGS, ""},
    {"PyChannelUtils_get_debugstring_by_msgtype_rawmsgdata",
     cyber_PyChannelUtils_get_debugstring_by_msgtype_rawmsgdata, METH_VARARGS,
     ""},
    {"PyChannelUtils_get_active_channels",
     cyber_PyChannelUtils_get_active_channels, METH_VARARGS, ""},
    {"PyChannelUtils_get_channels_info", cyber_PyChannelUtils_get_channels_info,
     METH_VARARGS, ""},

    {"PyNodeUtils_get_active_nodes", cyber_PyNodeUtils_get_active_nodes,
     METH_VARARGS, ""},
    {"PyNodeUtils_get_node_attr", cyber_PyNodeUtils_get_node_attr, METH_VARARGS,
     ""},
    {"PyNodeUtils_get_readersofnode", cyber_PyNodeUtils_get_readersofnode,
     METH_VARARGS, ""},
    {"PyNodeUtils_get_writersofnode", cyber_PyNodeUtils_get_writersofnode,
     METH_VARARGS, ""},

    {"PyServiceUtils_get_active_services",
     cyber_PyServiceUtils_get_active_services, METH_VARARGS, ""},
    {"PyServiceUtils_get_service_attr", cyber_PyServiceUtils_get_service_attr,
     METH_VARARGS, ""},

    {nullptr, nullptr, 0, nullptr} /* sentinel */
};

/// Init function of this module
#if PY_MAJOR_VERSION >= 3
PyMODINIT_FUNC PyInit__cyber_py3(void) {
  static struct PyModuleDef module_def = {
      PyModuleDef_HEAD_INIT,
      "_cyber_py3",    // Module name.
      "Cyber module",  // Module doc.
      -1,              // Module size.
      _cyber_methods,  // Module methods.
      nullptr,
      nullptr,
      nullptr,
      nullptr,
  };

  return PyModule_Create(&module_def);
}
#else
PyMODINIT_FUNC init_cyber(void) { Py_InitModule("_cyber", _cyber_methods); }
#endif
