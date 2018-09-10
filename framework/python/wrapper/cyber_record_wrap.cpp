#include <Python.h>
#include <string.h>
#include <iostream>
#include <memory>

#include "py_record.h"

static PyObject *cyber_py_is_shutdown(PyObject *self, PyObject *args) {
  bool is_shutdown = apollo::cybertron::record::py_is_shutdown();
  if (is_shutdown) {
    Py_RETURN_TRUE;
  } else {
    Py_RETURN_FALSE;
  }
}

static PyObject *cyber_py_init(PyObject *self, PyObject *args) {
  bool is_init = apollo::cybertron::record::py_init();
  if (is_init) {
    Py_RETURN_TRUE;
  } else {
    Py_RETURN_FALSE;
  }
}

static PyObject *cyber_py_ok(PyObject *self, PyObject *args) {
  bool is_ok = apollo::cybertron::record::py_OK();
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

///////////PyRecordFileReader methed////////////////////////////
PyObject *cyber_new_PyRecordFileReader(PyObject *self, PyObject *args) {
  apollo::cybertron::record::PyRecordFileReader *reader =
      new apollo::cybertron::record::PyRecordFileReader();
  PyObject *pyobj_rec_reader =
      PyCapsule_New(reader, "apollo_cybertron_record_pyrecordfilereader", NULL);
  return pyobj_rec_reader;
}

PyObject *cyber_delete_PyRecordFileReader(PyObject *self, PyObject *args) {
  PyObject *pyobj_rec_reader = nullptr;
  if (!PyArg_ParseTuple(args, (char *)"O:delete_PyRecordFileReader", &pyobj_rec_reader)) {
    return Py_None;
  }

  auto reader = (apollo::cybertron::record::PyRecordFileReader *)PyCapsule_GetPointer(
      pyobj_rec_reader, "apollo_cybertron_record_pyrecordfilereader");
  if (nullptr == reader) {
    AINFO << "delete_PyRecordFileReader:reader ptr is null!";
    return Py_None;
  }
  delete reader;
  return Py_None;
}

PyObject *cyber_PyRecordFileReader_Open(PyObject *self, PyObject *args) {
  PyObject *pyobj_reader = nullptr;
  char *path = nullptr;
  Py_ssize_t len = 0;
  if (!PyArg_ParseTuple(args, (char *)"Os#:cyber_PyRecordFileReader_Open", &pyobj_reader,
                        &path, &len)) {
    AINFO << "cyber_PyRecordFileReader_Open:PyRecordFileReader failed!";
    Py_RETURN_FALSE;
  }
  
  apollo::cybertron::record::PyRecordFileReader *reader =
      PyObjectToPtr<apollo::cybertron::record::PyRecordFileReader *>(pyobj_reader,
                                              "apollo_cybertron_record_pyrecordfilereader");

  if (nullptr == reader) {
    AINFO << "PyRecordFileReader_Open:reader ptr is null!";
    Py_RETURN_FALSE;
  }

  std::string path_str(path, len);
  AINFO << "c++:PyRecordFileReader_Open path->[ " << path_str << "]";
  if(!reader->Open(path_str)) {
    Py_RETURN_FALSE;
  }
  Py_RETURN_TRUE;
}

PyObject *cyber_PyRecordFileReader_Close(PyObject *self, PyObject *args) {
  PyObject *pyobj_reader = nullptr;
  if (!PyArg_ParseTuple(args, (char *)"O:PyRecordFileReader_Close", &pyobj_reader)) {
    return Py_None;
  }

  auto reader = (apollo::cybertron::record::PyRecordFileReader *)PyCapsule_GetPointer(
      pyobj_reader, "apollo_cybertron_record_pyrecordfilereader");
  if (nullptr == reader) {
    AINFO << "cyber_PyRecordFileReader_Close:PyRecordFileReader ptr is null!";
    return Py_None;
  }
  reader->Close();
  return Py_None;
}

PyObject *cyber_PyRecordFileReader_ReadHeader(PyObject *self, PyObject *args) {
  PyObject *pyobj_reader = nullptr;
  if (!PyArg_ParseTuple(args, (char *)"O:PyRecordFileReader_ReadHeader", &pyobj_reader)) {
    Py_RETURN_FALSE;
  }

  auto reader = (apollo::cybertron::record::PyRecordFileReader *)PyCapsule_GetPointer(
      pyobj_reader, "apollo_cybertron_record_pyrecordfilereader");
  if (nullptr == reader) {
    AINFO << "PyRecordFileReader_ReadHeader ptr is null!";
    Py_RETURN_FALSE;
  }
  
  if(!reader->ReadHeader()) {
    Py_RETURN_FALSE;
  }
  Py_RETURN_TRUE;
}

PyObject *cyber_PyRecordFileReader_ReadIndex(PyObject *self, PyObject *args) {
  PyObject *pyobj_reader = nullptr;
  if (!PyArg_ParseTuple(args, (char *)"O:PyRecordFileReader_ReadIndex", &pyobj_reader)) {
    Py_RETURN_FALSE;
  }

  auto reader = (apollo::cybertron::record::PyRecordFileReader *)PyCapsule_GetPointer(
      pyobj_reader, "apollo_cybertron_record_pyrecordfilereader");
  if (nullptr == reader) {
    AINFO << "PyRecordFileReader_ReadIndex ptr is null!";
    Py_RETURN_FALSE;
  }
  
  if(!reader->ReadIndex()) {
    Py_RETURN_FALSE;
  }
  Py_RETURN_TRUE;
}

PyObject *cyber_PyRecordFileReader_EndOfFile(PyObject *self, PyObject *args) {
  PyObject *pyobj_reader = nullptr;
  if (!PyArg_ParseTuple(args, (char *)"O:PyRecordFileReader_EndOfFile", &pyobj_reader)) {
    Py_RETURN_FALSE;
  }

  auto reader = (apollo::cybertron::record::PyRecordFileReader *)PyCapsule_GetPointer(
      pyobj_reader, "apollo_cybertron_record_pyrecordfilereader");
  if (nullptr == reader) {
    AINFO << "PyRecordFileReader_EndOfFile ptr is null!";
    Py_RETURN_FALSE;
  }
  
  if(!reader->EndOfFile()) {
    Py_RETURN_FALSE;
  }
  Py_RETURN_TRUE;
}

///////////PyRecordFileWriter methed////////////////////////////
PyObject *cyber_new_PyRecordFileWriter(PyObject *self, PyObject *args) {
  apollo::cybertron::record::PyRecordFileWriter *writer =
      new apollo::cybertron::record::PyRecordFileWriter();
  PyObject *pyobj_rec_writer =
      PyCapsule_New(writer, "apollo_cybertron_record_pyrecordfilewriter", NULL);
  return pyobj_rec_writer;
}

PyObject *cyber_delete_PyRecordFileWriter(PyObject *self, PyObject *args) {
  PyObject *pyobj_rec_writer = nullptr;
  if (!PyArg_ParseTuple(args, (char *)"O:delete_PyRecordFileWriter", &pyobj_rec_writer)) {
    return Py_None;
  }

  auto writer = (apollo::cybertron::record::PyRecordFileWriter *)PyCapsule_GetPointer(
      pyobj_rec_writer, "apollo_cybertron_record_pyrecordfilewriter");
  if (nullptr == writer) {
    AINFO << "delete_PyRecordFileWriter:writer ptr is null!";
    return Py_None;
  }
  delete writer;
  return Py_None;
}

PyObject *cyber_PyRecordFileWriter_Open(PyObject *self, PyObject *args) {
  PyObject *pyobj_rec_writer = nullptr;
  char *path = nullptr;
  Py_ssize_t len = 0;
  if (!PyArg_ParseTuple(args, (char *)"Os#:cyber_PyRecordFileWriter_Open", &pyobj_rec_writer,
                        &pyobj_rec_writer, &len)) {
    AINFO << "cyber_PyRecordFileWriter_Open:PyRecordFileWriter_Open failed!";
    Py_RETURN_FALSE;
  }

  apollo::cybertron::record::PyRecordFileWriter *writer =
      PyObjectToPtr<apollo::cybertron::record::PyRecordFileWriter *>(pyobj_rec_writer,
                                              "apollo_cybertron_record_pyrecordfilewriter");

  if (nullptr == writer) {
    AINFO << "PyRecordFileWriter_Open:writer ptr is null!";
    Py_RETURN_FALSE;
  }

  std::string path_str(path, len);
  AINFO << "c++:PyRecordFileWriter_Open path->[ " << path_str << "]";
  if(!writer->Open(path_str)) {
    Py_RETURN_FALSE;
  }
  Py_RETURN_TRUE;
}

PyObject *cyber_PyRecordFileWriter_Close(PyObject *self, PyObject *args) {
  PyObject *pyobj_rec_writer = nullptr;
  if (!PyArg_ParseTuple(args, (char *)"O:delete_PyRecordFileWriter", &pyobj_rec_writer)) {
    return Py_None;
  }

  auto writer = (apollo::cybertron::record::PyRecordFileWriter *)PyCapsule_GetPointer(
      pyobj_rec_writer, "apollo_cybertron_record_pyrecordfilewriter");
  if (nullptr == writer) {
    AINFO << "cyber_PyRecordFileWriterer_Close: ptr is null!";
    return Py_None;
  }
  writer->Close();
  return Py_None;
}

PyObject *cyber_PyRecordFileWriter_WriteHeader(PyObject *self, PyObject *args) {
  PyObject *pyobj_rec_writer = nullptr;
  char *header = nullptr;
  Py_ssize_t len = 0;
  if (!PyArg_ParseTuple(args, (char *)"Os#:cyber_PyRecordFileWriter_WriteHeader", &pyobj_rec_writer,
                        &pyobj_rec_writer, &len)) {
    AINFO << "cyber_PyRecordFileWriter_WriteHeader failed!";
    Py_RETURN_FALSE;
  }

  auto writer =
      PyObjectToPtr<apollo::cybertron::record::PyRecordFileWriter *>(pyobj_rec_writer,
                                              "apollo_cybertron_record_pyrecordfilewriter");

  if (nullptr == writer) {
    AINFO << "cyber_PyRecordFileWriter_WriteHeader:writer ptr is null!";
    Py_RETURN_FALSE;
  }

  std::string header_str(header, len);
  AINFO << "c++:cyber_PyRecordFileWriter_WriteHeader header->[ " << header_str << "]";
  if(!writer->WriteHeader(header_str)) {
    Py_RETURN_FALSE;
  }
  Py_RETURN_TRUE;
}

PyObject *cyber_PyRecordFileWriter_WriteChannel(PyObject *self, PyObject *args) {
  PyObject *pyobj_rec_writer = nullptr;
  char *channel = nullptr;
  Py_ssize_t len = 0;
  if (!PyArg_ParseTuple(args, (char *)"Os#:cyber_PyRecordFileWriter_WriteChannel", &pyobj_rec_writer,
                        &pyobj_rec_writer, &len)) {
    AINFO << "cyber_PyRecordFileWriter_WriteChannel failed!";
    Py_RETURN_FALSE;
  }

  auto writer =
      PyObjectToPtr<apollo::cybertron::record::PyRecordFileWriter *>(pyobj_rec_writer,
                                              "apollo_cybertron_record_pyrecordfilewriter");

  if (nullptr == writer) {
    AINFO << "PyRecordFileWriter_Open:writer ptr is null!";
    Py_RETURN_FALSE;
  }

  std::string channel_str(channel, len);
  AINFO << "c++:PyRecordFileWriter_Open channel->[ " << channel_str << "]";
  if(!writer->WriteChannel(channel_str)) {
    Py_RETURN_FALSE;
  }
  Py_RETURN_TRUE;
}

PyObject *cyber_PyRecordFileWriter_AddSingleMessage(PyObject *self, PyObject *args) {
  PyObject *pyobj_rec_writer = nullptr;
  char *single = nullptr;
  Py_ssize_t len = 0;
  if (!PyArg_ParseTuple(args, (char *)"Os#:cyber_PyRecordFileWriter_AddSingleMessage", &pyobj_rec_writer,
                        &pyobj_rec_writer, &len)) {
    AINFO << "cyber_PyRecordFileWriter_AddSingleMessage parsetuple failed!";
    Py_RETURN_FALSE;
  }

  auto writer =
      PyObjectToPtr<apollo::cybertron::record::PyRecordFileWriter *>(pyobj_rec_writer,
                                              "apollo_cybertron_record_pyrecordfilewriter");

  if (nullptr == writer) {
    AINFO << "cyber_PyRecordFileWriter_AddSingleMessage:writer ptr is null!";
    Py_RETURN_FALSE;
  }

  std::string single_str(single, len);
  AINFO << "c++:cyber_PyRecordFileWriter_AddSingleMessage singlemsg->[ " << single_str << "]";
  if(!writer->AddSingleMessage(single_str)) {
    Py_RETURN_FALSE;
  }
  Py_RETURN_TRUE;
}
/////////////////////////////////////////////////////////////////////
//// global for whole page, init module
/////////////////////////////////////////////////////////////////////
static PyMethodDef _cyber_record_methods[] = {
    // global fun
    {"py_is_shutdown", cyber_py_is_shutdown, METH_NOARGS, ""},
    {"py_init", cyber_py_init, METH_NOARGS, ""},
    {"py_OK", cyber_py_ok, METH_NOARGS, ""},
    // PyRecordFileReader fun
    {"new_PyRecordFileReader", cyber_new_PyRecordFileReader, METH_VARARGS, ""},
    {"delete_PyRecordFileReader", cyber_delete_PyRecordFileReader, METH_VARARGS, ""},
    {"PyRecordFileReader_Open", cyber_PyRecordFileReader_Open, METH_VARARGS, ""},
    {"PyRecordFileReader_Close", cyber_PyRecordFileReader_Close, METH_VARARGS, ""},
    {"PyRecordFileReader_ReadHeader", cyber_PyRecordFileReader_ReadHeader, METH_VARARGS, ""},
    {"PyRecordFileReader_ReadIndex", cyber_PyRecordFileReader_ReadIndex, METH_VARARGS, ""},
    {"PyRecordFileReader_EndOfFile", cyber_PyRecordFileReader_EndOfFile, METH_VARARGS, ""},
    // PyRecordFileWriter fun
    {"new_PyRecordFileWriter", cyber_new_PyRecordFileWriter, METH_VARARGS, ""},
    {"delete_PyRecordFileWriter", cyber_delete_PyRecordFileWriter, METH_VARARGS, ""},
    {"PyRecordFileWriter_Open", cyber_PyRecordFileWriter_Open, METH_VARARGS, ""},
    {"PyRecordFileWriter_Close", cyber_PyRecordFileWriter_Close, METH_VARARGS, ""},
    {"PyRecordFileWriter_WriteHeader", cyber_PyRecordFileWriter_WriteHeader, METH_VARARGS, ""},
    {"PyRecordFileWriter_WriteChannel", cyber_PyRecordFileWriter_WriteChannel, METH_VARARGS, ""},
    {"PyRecordFileWriter_AddSingleMessage", cyber_PyRecordFileWriter_AddSingleMessage, METH_VARARGS, ""},
    
    {NULL, NULL, 0, NULL} /* sentinel */
};

/// Init function of this module
PyMODINIT_FUNC init_cyber_record(void) {
  AINFO << "init _cyber_record";
  Py_InitModule("_cyber_record", _cyber_record_methods);
}
