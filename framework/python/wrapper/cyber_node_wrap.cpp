#include <Python.h>
#include <string.h>

#include "py_node.h"

static PyObject *cyber_py_is_shutdown(PyObject *self, PyObject *args) {
  bool is_shutdown = apollo::cybertron::py_is_shutdown();
  if (is_shutdown) {
    Py_RETURN_TRUE;
  } else {
    Py_RETURN_FALSE;
  }
}

static PyObject *cyber_py_init(PyObject *self, PyObject *args) {
  bool is_init = apollo::cybertron::py_init();
  if (is_init) {
    Py_RETURN_TRUE;
  } else {
    Py_RETURN_FALSE;
  }
}

static PyObject *cyber_py_ok(PyObject *self, PyObject *args) {
  bool is_ok = apollo::cybertron::OK();
  if (is_ok) {
    Py_RETURN_TRUE;
  } else {
    Py_RETURN_FALSE;
  }
}

/////////////////general functions////////////////////////////////////
#define PYOBJECT_NULL_STRING PyString_FromStringAndSize("", 0)

template <typename T>
T PyObjectToPtr(PyObject *pyobj, const std::string &type_ptr) {
  T obj_ptr = (T)PyCapsule_GetPointer(pyobj, type_ptr.c_str());
  if (obj_ptr == nullptr) {
    AINFO << "PyObjectToPtr failed,type->" << type_ptr << "pyobj: " << pyobj;
  }
  return obj_ptr;
}

///////////pywriter methed////////////////////////////
PyObject *cyber_new_PyWriter(PyObject *self, PyObject *args) {
  char *channel_name = nullptr;
  char *data_type = nullptr;
  apollo::cybertron::Node *node = nullptr;

  PyObject *node_pyobj = nullptr;
  if (!PyArg_ParseTuple(args, (char *)"ssO:new_PyWriter", &channel_name,
                        &data_type, &node_pyobj)) {
    return Py_None;
  }

  node = (apollo::cybertron::Node *)PyCapsule_GetPointer(
      node_pyobj, "apollo_cybertron_pynode");
  if (nullptr == node) {
    AINFO << "node is null";
    return Py_None;
  }

  apollo::cybertron::PyWriter *writer =
      new apollo::cybertron::PyWriter((std::string const &)*channel_name,
                                      (std::string const &)*data_type, node);
  PyObject *pyobj_writer =
      PyCapsule_New(writer, "apollo_cybertron_pywriter", NULL);
  return pyobj_writer;
}

PyObject *cyber_delete_PyWriter(PyObject *self, PyObject *args) {
  apollo::cybertron::PyWriter *writer = nullptr;

  PyObject *writer_py = nullptr;
  if (!PyArg_ParseTuple(args, (char *)"O:delete_PyWriter", &writer_py)) {
    return Py_None;
  }

  writer = (apollo::cybertron::PyWriter *)PyCapsule_GetPointer(
      writer_py, "apollo_cybertron_pywriter");
  delete writer;
  return Py_None;
}

PyObject *cyber_PyWriter_write(PyObject *self, PyObject *args) {
  PyObject *pyobj_writer = nullptr;
  char *data = nullptr;
  Py_ssize_t len = 0;
  if (!PyArg_ParseTuple(args, (char *)"Os#:cyber_PyWriter_write", &pyobj_writer,
                        &data, &len)) {
    AINFO << "cyber_PyWriter_write:cyber_PyWriter_write failed!";
    return PyInt_FromLong(1);
  }

  apollo::cybertron::PyWriter *writer =
      PyObjectToPtr<apollo::cybertron::PyWriter *>(pyobj_writer,
                                                   "apollo_cybertron_pywriter");

  if (nullptr == writer) {
    AINFO << "cyber_PyWriter_write:writer ptr is null!";
    return PyInt_FromLong(1);
  }

  std::string data_str(data, len);
  int ret = writer->write(data_str);
  return PyInt_FromLong((long)ret);
}

///////////pyreader methed////////////////////////////
PyObject *cyber_new_PyReader(PyObject *self, PyObject *args) {
  char *channel_name = nullptr;
  char *data_type = nullptr;
  apollo::cybertron::Node *node = 0;

  PyObject *node_pyobj = 0;
  if (!PyArg_ParseTuple(args, (char *)"ssO:new_PyReader", &channel_name,
                        &data_type, &node_pyobj)) {
    return Py_None;
  }

  node = (apollo::cybertron::Node *)PyCapsule_GetPointer(
      node_pyobj, "apollo_cybertron_pynode");
  if (!node) {
    AINFO << "node is null";
    return Py_None;
  }

  apollo::cybertron::PyReader *reader =
      new apollo::cybertron::PyReader((std::string const &)*channel_name,
                                      (std::string const &)*data_type, node);
  PyObject *pyobj_reader =
      PyCapsule_New(reader, "apollo_cybertron_pyreader", NULL);
  return pyobj_reader;
}

PyObject *cyber_delete_PyReader(PyObject *self, PyObject *args) {
  PyObject *reader_py = 0;
  if (!PyArg_ParseTuple(args, (char *)"O:delete_PyReader", &reader_py)) {
    return Py_None;
  }

  apollo::cybertron::PyReader *reader =
      (apollo::cybertron::PyReader *)PyCapsule_GetPointer(
          reader_py, "apollo_cybertron_pyreader");
  delete reader;
  return Py_None;
}

PyObject *cyber_PyReader_read(PyObject *self, PyObject *args) {
  PyObject *pyobj_reader = nullptr;
  PyObject *pyobj_iswait = nullptr;

  if (!PyArg_ParseTuple(args, (char *)"OO:cyber_PyReader_read", &pyobj_reader,
                        &pyobj_iswait)) {
    AINFO << "cyber_PyReader_read:PyArg_ParseTuple failed!";
    return Py_None;
  }
  apollo::cybertron::PyReader *reader =
      PyObjectToPtr<apollo::cybertron::PyReader *>(pyobj_reader,
                                                   "apollo_cybertron_pyreader");
  if (nullptr == reader) {
    AINFO << "cyber_PyReader_read:PyReader ptr is null!";
    return Py_None;
  }

  int r = PyObject_IsTrue(pyobj_iswait);
  if (r == -1) {
    AINFO << "cyber_PyReader_read:pyobj_iswait is error!";
    return Py_None;
  }

  bool wait = (r == 1 ? true : false);

  std::string reader_ret = reader->read(wait);
  // AINFO << "c++:PyReader_read -> " << reader_ret;
  return PyString_FromStringAndSize(reader_ret.c_str(), reader_ret.size());
}

PyObject *cyber_PyReader_register_func(PyObject *self, PyObject *args) {
  PyObject *pyobj_regist_fun = 0;
  PyObject *pyobj_reader = 0;

  int (*callback_fun)(char const *) = (int (*)(char const *))0;

  if (!PyArg_ParseTuple(args, (char *)"OO:PyReader_register_func",
                        &pyobj_reader, &pyobj_regist_fun)) {
    return Py_None;
  }

  apollo::cybertron::PyReader *reader =
      PyObjectToPtr<apollo::cybertron::PyReader *>(pyobj_reader,
                                                   "apollo_cybertron_pyreader");
  callback_fun = (int (*)(const char *i))PyInt_AsLong(pyobj_regist_fun);
  if (reader) {
    AINFO << "reader regist fun";
    reader->register_func(callback_fun);
  }

  return Py_None;
}
///////////pyclient methed////////////////////////////
PyObject *cyber_new_PyClient(PyObject *self, PyObject *args) {
  char *channel_name = 0;
  char *data_type = 0;
  apollo::cybertron::Node *node = 0;

  PyObject *node_pyobj = 0;
  if (!PyArg_ParseTuple(args, (char *)"ssO:new_PyClient", &channel_name,
                        &data_type, &node_pyobj)) {
    return Py_None;
  }

  node = (apollo::cybertron::Node *)PyCapsule_GetPointer(
      node_pyobj, "apollo_cybertron_pynode");
  if (!node) {
    AINFO << "node is null";
    return Py_None;
  }

  apollo::cybertron::PyClient *client =
      new apollo::cybertron::PyClient((std::string const &)*channel_name,
                                      (std::string const &)*data_type, node);
  PyObject *pyobj_client =
      PyCapsule_New(client, "apollo_cybertron_pyclient", NULL);
  return pyobj_client;
}

PyObject *cyber_delete_PyClient(PyObject *self, PyObject *args) {
  PyObject *client_py = 0;
  if (!PyArg_ParseTuple(args, (char *)"O:delete_PyClient", &client_py)) {
    return Py_None;
  }

  apollo::cybertron::PyClient *client =
      (apollo::cybertron::PyClient *)PyCapsule_GetPointer(
          client_py, "apollo_cybertron_pyclient");
  delete client;
  return Py_None;
}

PyObject *cyber_PyClient_send_request(PyObject *self, PyObject *args) {
  PyObject *pyobj_client = nullptr;
  char *data = nullptr;
  Py_ssize_t len = 0;
  if (!PyArg_ParseTuple(args, (char *)"Os#:PyClient_send_request",
                        &pyobj_client, &data, &len)) {
    AINFO << "cyber_PyClient_send_request:PyArg_ParseTuple failed!";
    return PYOBJECT_NULL_STRING;
  }

  apollo::cybertron::PyClient *client =
      PyObjectToPtr<apollo::cybertron::PyClient *>(pyobj_client,
                                                   "apollo_cybertron_pyclient");

  if (nullptr == client) {
    AINFO << "cyber_PyClient_send_request:client ptr is null!";
    return PYOBJECT_NULL_STRING;
  }

  std::string data_str(data, len);
  AINFO << "c++:PyClient_send_request data->[ " << data_str << "]";
  std::string response_str =
      client->send_request((std::string const &)data_str);
  AINFO << "c++:response data->[ " << response_str << "]";
  return PyString_FromStringAndSize(response_str.c_str(), response_str.size());
}

///////////pyservice methed/////////////////
/// apollo_cybertron_pyservice///////////
PyObject *cyber_new_PyService(PyObject *self, PyObject *args) {
  char *channel_name = 0;
  char *data_type = 0;
  apollo::cybertron::Node *node = 0;

  PyObject *node_pyobj = 0;
  if (!PyArg_ParseTuple(args, (char *)"ssO:new_PyService", &channel_name,
                        &data_type, &node_pyobj)) {
    return Py_None;
  }

  node = (apollo::cybertron::Node *)PyCapsule_GetPointer(
      node_pyobj, "apollo_cybertron_pynode");
  if (!node) {
    AINFO << "node is null";
    return Py_None;
  }

  apollo::cybertron::PyService *service =
      new apollo::cybertron::PyService((std::string const &)*channel_name,
                                       (std::string const &)*data_type, node);
  PyObject *pyobj_service =
      PyCapsule_New(service, "apollo_cybertron_pyservice", NULL);
  return pyobj_service;
}

PyObject *cyber_delete_PyService(PyObject *self, PyObject *args) {
  PyObject *pyobj_service = 0;
  if (!PyArg_ParseTuple(args, (char *)"O:delete_PyService", &pyobj_service)) {
    return Py_None;
  }

  apollo::cybertron::PyService *service =
      (apollo::cybertron::PyService *)PyCapsule_GetPointer(
          pyobj_service, "apollo_cybertron_pyservice");
  delete service;
  return Py_None;
}

PyObject *cyber_PyService_register_func(PyObject *self, PyObject *args) {
  PyObject *pyobj_regist_fun = nullptr;
  PyObject *pyobj_service = nullptr;

  int (*callback_fun)(char const *) = (int (*)(char const *))0;

  if (!PyArg_ParseTuple(args, (char *)"OO:PyService_register_func",
                        &pyobj_service, &pyobj_regist_fun)) {
    return Py_None;
  }

  apollo::cybertron::PyService *service =
      PyObjectToPtr<apollo::cybertron::PyService *>(
          pyobj_service, "apollo_cybertron_pyservice");
  callback_fun = (int (*)(const char *i))PyInt_AsLong(pyobj_regist_fun);
  if (service) {
    AINFO << "service regist fun";
    service->register_func(callback_fun);
  }

  return Py_None;
}

PyObject *cyber_PyService_read(PyObject *self, PyObject *args) {
  PyObject *pyobj_service = nullptr;
  char *data = nullptr;
  if (!PyArg_ParseTuple(args, (char *)"O:cyber_PyService_read",
                        &pyobj_service)) {
    AINFO << "cyber_PyService_read:PyArg_ParseTuple failed!";
    return PYOBJECT_NULL_STRING;
  }
  apollo::cybertron::PyService *service =
      PyObjectToPtr<apollo::cybertron::PyService *>(
          pyobj_service, "apollo_cybertron_pyservice");
  if (nullptr == service) {
    AINFO << "cyber_PyService_read:service ptr is null!";
    return PYOBJECT_NULL_STRING;
  }

  std::string reader_ret = service->read();
  AINFO << "c++:PyService_read -> " << reader_ret;
  return PyString_FromStringAndSize(reader_ret.c_str(), reader_ret.size());
}

PyObject *cyber_PyService_write(PyObject *self, PyObject *args) {
  PyObject *pyobj_service = nullptr;
  char *data = nullptr;
  Py_ssize_t len = 0;
  if (!PyArg_ParseTuple(args, (char *)"Os#:cyber_PyService_write",
                        &pyobj_service, &data, &len)) {
    AINFO << "cyber_PyService_write:PyArg_ParseTuple failed!";
    return PyInt_FromLong(1);
  }

  apollo::cybertron::PyService *service =
      PyObjectToPtr<apollo::cybertron::PyService *>(
          pyobj_service, "apollo_cybertron_pyservice");

  if (nullptr == service) {
    AINFO << "cyber_PyService_write:writer ptr is null!";
    return PyInt_FromLong(1);
  }

  std::string data_str(data, len);
  AINFO << "c++:PyService_write data->[ " << data_str << "]";
  int ret = service->write((std::string const &)data_str);
  return PyInt_FromLong((long)ret);
}

///////////pynode methed////////////////////////////
PyObject *cyber_new_PyNode(PyObject *self, PyObject *args) {
  char *node_name = nullptr;
  if (!PyArg_ParseTuple(args, (char *)"s:new_PyNode", &node_name)) {
    return Py_None;
  }

  apollo::cybertron::PyNode *node =
      new apollo::cybertron::PyNode((std::string const &)node_name);
  PyObject *pyobj_node = PyCapsule_New(node, "apollo_cybertron_pynode", NULL);
  return pyobj_node;
}

PyObject *cyber_delete_PyNode(PyObject *self, PyObject *args) {
  PyObject *pyobj_node = nullptr;
  if (!PyArg_ParseTuple(args, (char *)"O:delete_PyNode", &pyobj_node)) {
    return Py_None;
  }

  apollo::cybertron::PyNode *node =
      (apollo::cybertron::PyNode *)PyCapsule_GetPointer(
          pyobj_node, "apollo_cybertron_pynode");
  delete node;
  return Py_None;
}

PyObject *cyber_PyNode_create_writer(PyObject *self, PyObject *args) {
  char *channel_name = nullptr;
  char *type_name = nullptr;
  PyObject *pyobj_node = nullptr;

  if (!PyArg_ParseTuple(args, (char *)"Oss:PyNode_create_writer", &pyobj_node,
                        &channel_name, &type_name)) {
    AINFO << "cyber_PyNode_create_writer:PyArg_ParseTuple failed!";
    return Py_None;
  }

  apollo::cybertron::PyNode *node = PyObjectToPtr<apollo::cybertron::PyNode *>(
      pyobj_node, "apollo_cybertron_pynode");
  if (nullptr == node) {
    AINFO << "cyber_PyNode_create_writer:node ptr is null!";
    return Py_None;
  }

  apollo::cybertron::PyWriter *writer =
      (apollo::cybertron::PyWriter *)(node->create_writer(
          (std::string const &)channel_name, (std::string const &)type_name));

  PyObject *pyobj_writer =
      PyCapsule_New(writer, "apollo_cybertron_pywriter", NULL);
  return pyobj_writer;
}

PyObject *cyber_PyNode_create_reader(PyObject *self, PyObject *args) {
  char *channel_name = nullptr;
  char *type_name = nullptr;
  PyObject *pyobj_node = nullptr;

  if (!PyArg_ParseTuple(args, (char *)"Oss:PyNode_create_reader", &pyobj_node,
                        &channel_name, &type_name)) {
    AINFO << "PyNode_create_reader:PyArg_ParseTuple failed!";
    return Py_None;
  }

  apollo::cybertron::PyNode *node = PyObjectToPtr<apollo::cybertron::PyNode *>(
      pyobj_node, "apollo_cybertron_pynode");
  if (nullptr == node) {
    AINFO << "PyNode_create_reader:node ptr is null!";
    return Py_None;
  }

  apollo::cybertron::PyReader *reader =
      (apollo::cybertron::PyReader *)(node->create_reader(
          (std::string const &)channel_name, (std::string const &)type_name));

  PyObject *pyobj_reader =
      PyCapsule_New(reader, "apollo_cybertron_pyreader", NULL);
  return pyobj_reader;
}

PyObject *cyber_PyNode_create_client(PyObject *self, PyObject *args) {
  char *channel_name = nullptr;
  char *type_name = nullptr;
  PyObject *pyobj_node = nullptr;

  if (!PyArg_ParseTuple(args, (char *)"Oss:PyNode_create_client", &pyobj_node,
                        &channel_name, &type_name)) {
    AINFO << "PyNode_create_client:PyArg_ParseTuple failed!";
    return Py_None;
  }

  apollo::cybertron::PyNode *node = PyObjectToPtr<apollo::cybertron::PyNode *>(
      pyobj_node, "apollo_cybertron_pynode");
  if (nullptr == node) {
    AINFO << "PyNode_create_client:node ptr is null!";
    return Py_None;
  }

  apollo::cybertron::PyClient *client =
      (apollo::cybertron::PyClient *)(node->create_client(
          (std::string const &)channel_name, (std::string const &)type_name));
  PyObject *pyobj_client =
      PyCapsule_New(client, "apollo_cybertron_pyclient", NULL);

  return pyobj_client;
}

PyObject *cyber_PyNode_create_service(PyObject *self, PyObject *args) {
  char *channel_name = nullptr;
  char *type_name = nullptr;
  PyObject *pyobj_node = nullptr;

  if (!PyArg_ParseTuple(args, (char *)"Oss:cyber_PyNode_create_service",
                        &pyobj_node, &channel_name, &type_name)) {
    AINFO << "cyber_PyNode_create_service:PyArg_ParseTuple failed!";
    return Py_None;
  }

  apollo::cybertron::PyNode *node = PyObjectToPtr<apollo::cybertron::PyNode *>(
      pyobj_node, "apollo_cybertron_pynode");
  if (nullptr == node) {
    AINFO << "cyber_PyNode_create_service:node ptr is null!";
    return Py_None;
  }

  apollo::cybertron::PyService *service =
      (apollo::cybertron::PyService *)(node->create_service(
          (std::string const &)channel_name, (std::string const &)type_name));
  PyObject *pyobj_service =
      PyCapsule_New(service, "apollo_cybertron_pyservice", NULL);
  return pyobj_service;
}

PyObject *cyber_PyNode_shutdown(PyObject *self, PyObject *args) {
  PyObject *pyobj_node = nullptr;

  if (!PyArg_ParseTuple(args, (char *)"O:PyNode_shutdown", &pyobj_node)) {
    AINFO << "cyber_PyNode_shutdown:PyNode_shutdown failed!";
    return Py_None;
  }

  apollo::cybertron::PyNode *node = PyObjectToPtr<apollo::cybertron::PyNode *>(
      pyobj_node, "apollo_cybertron_pynode");
  if (nullptr == node) {
    AINFO << "cyber_PyNode_shutdown:node ptr is null!";
    return Py_None;
  }
  node->shutdown();
  return Py_None;
}

PyObject *cyber_PyNode_register_message(PyObject *self, PyObject *args) {
  PyObject *pyobj_node = nullptr;
  char *desc = nullptr;
  if (!PyArg_ParseTuple(args, (char *)"Os:cyber_PyNode_register_message",
                        &pyobj_node, &desc)) {
    AINFO << "cyber_PyNode_register_message: failed!";
    return Py_None;
  }
  apollo::cybertron::PyNode *node = PyObjectToPtr<apollo::cybertron::PyNode *>(
      pyobj_node, "apollo_cybertron_pynode");
  if (nullptr == node) {
    AINFO << "cyber_PyNode_register_message:node ptr is null! desc->" << desc;
    return Py_None;
  }
  node->register_message((std::string const &)desc);
  return Py_None;
}

/////////////////////////////////////////////////////////////////////
//// debug pyobject
/////////////////////////////////////////////////////////////////////

PyObject *cyber_test0(PyObject *self, PyObject *args) {
  int channel = 0;
  int data_type = 0;
  AINFO << "+++++++++++++++++++++begin";
  if (!PyArg_ParseTuple(args, "ii", &channel, &data_type)) {
    return Py_None;
  }

  AINFO << "channel, data_type->:" << channel << ":" << data_type;
  std::string ret_str = "good morning";
  return PyString_FromStringAndSize(ret_str.c_str(), ret_str.size());
  ;
}

struct student {
  std::string name;
  int age;
};

student *cyber_student() {
  student *stu1 = new student();
  stu1->name = "ywf";
  stu1->age = 22;
  return stu1;
}

using namespace std;
PyObject *cyber_test1(PyObject *self, PyObject *args) {
  char *channel = nullptr;
  char *data_type = nullptr;
  PyObject *pyboj_binstr = nullptr;
  char *s = 0;
  int len = 0;
  if (!PyArg_ParseTuple(args, "sss#", &channel, &data_type, &s, &len)) {
    return Py_None;
  }
  std::string str(s, len);

  AINFO << "p3: " << str;
  AINFO << "++++len: " << len;
  AINFO << str;

  AINFO << "channel, data_type->:" << channel << ":" << data_type;

  ////////test ptr
  student *stu = cyber_student();
  // ptr->pyobj
  PyObject *py_stu = PyCapsule_New(stu, "student", NULL);
  AINFO << "capsule name->" << PyCapsule_GetName(py_stu);

  AINFO << "===========================";
  // shared ptr
  std::vector<string> *strPtrV = new std::vector<string>;
  strPtrV->push_back("ywf");
  strPtrV->push_back("lj");
  PyObject *py_stu1 = PyCapsule_New(strPtrV, "studentptr", NULL);
  AINFO << "capsule name->" << PyCapsule_GetName(py_stu1);

  std::vector<string> *stu1_ptr =
      (std::vector<string> *)PyCapsule_GetPointer(py_stu1, "studentptr");
  if (stu1_ptr) {
    AINFO << "jiebao->" << (*stu1_ptr)[0] << ";" << (*stu1_ptr)[1];
  }

  return Py_None;
}
/////////////////////////////////////////////////////////////////////
//// global for whole page, init module
/////////////////////////////////////////////////////////////////////
static PyMethodDef _cyber_node_methods[] = {
    // global fun
    {"py_is_shutdown", cyber_py_is_shutdown, METH_NOARGS, ""},
    {"py_init", cyber_py_init, METH_NOARGS, ""},
    {"py_OK", cyber_py_ok, METH_NOARGS, ""},

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

    // for test
    {"cyber_test0", cyber_test0, METH_VARARGS, "test parms input."},
    {"cyber_test1", cyber_test1, METH_VARARGS, "test parms input."},

    {NULL, NULL, 0, NULL} /* sentinel */
};

/// Init function of this module
PyMODINIT_FUNC init_cyber_node(void) {
  AINFO << "init _cyber_node";
  Py_InitModule("_cyber_node", _cyber_node_methods);
}
