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
#include <set>
#include <string>

#include "cyber/py_wrapper/py_record.h"

#define PYOBJECT_NULL_STRING PyString_FromStringAndSize("", 0)

template <typename T>
T PyObjectToPtr(PyObject *pyobj, const std::string &type_ptr) {
  T obj_ptr = (T)PyCapsule_GetPointer(pyobj, type_ptr.c_str());
  if (obj_ptr == nullptr) {
    AERROR << "PyObjectToPtr failed,type->" << type_ptr << "pyobj: " << pyobj;
  }
  return obj_ptr;
}

PyObject *cyber_new_PyRecordReader(PyObject *self, PyObject *args) {
  char *filepath = nullptr;
  Py_ssize_t len = 0;
  if (!PyArg_ParseTuple(args, const_cast<char *>("s#:new_PyRecordReader"),
                        &filepath, &len)) {
    AERROR << "cyber_new_PyRecordReader parsetuple failed!";
    Py_INCREF(Py_None);
    return Py_None;
  }

  apollo::cyber::record::PyRecordReader *reader =
      new apollo::cyber::record::PyRecordReader(std::string(filepath, len));
  PyObject *pyobj_rec_reader =
      PyCapsule_New(reader, "apollo_cyber_record_pyrecordfilereader", nullptr);
  return pyobj_rec_reader;
}

PyObject *cyber_delete_PyRecordReader(PyObject *self, PyObject *args) {
  PyObject *pyobj_rec_reader = nullptr;
  if (!PyArg_ParseTuple(args, const_cast<char *>("O:delete_PyRecordReader"),
                        &pyobj_rec_reader)) {
    Py_INCREF(Py_None);
    return Py_None;
  }

  auto reader = (apollo::cyber::record::PyRecordReader *)PyCapsule_GetPointer(
      pyobj_rec_reader, "apollo_cyber_record_pyrecordfilereader");
  if (nullptr == reader) {
    AINFO << "delete_PyRecordReader:reader ptr is null!";
    Py_INCREF(Py_None);
    return Py_None;
  }
  delete reader;
  Py_INCREF(Py_None);
  return Py_None;
}

PyObject *cyber_PyRecordReader_ReadMessage(PyObject *self, PyObject *args) {
  PyObject *pyobj_reader = nullptr;
  uint64_t begin_time = 0;
  uint64_t end_time = UINT64_MAX;
  if (!PyArg_ParseTuple(args,
                        const_cast<char *>("OKK:PyRecordReader_ReadMessage"),
                        &pyobj_reader, &begin_time, &end_time)) {
    return nullptr;
  }

  auto reader = (apollo::cyber::record::PyRecordReader *)PyCapsule_GetPointer(
      pyobj_reader, "apollo_cyber_record_pyrecordfilereader");
  if (nullptr == reader) {
    AERROR << "PyRecordReader_ReadMessage ptr is null!";
    return nullptr;
  }

  apollo::cyber::record::BagMessage result;
  result = reader->ReadMessage(begin_time, end_time);
  PyObject *pyobj_bag_message = PyDict_New();

  PyObject *bld_name = Py_BuildValue("s", result.channel_name.c_str());
  PyDict_SetItemString(pyobj_bag_message, "channel_name", bld_name);
  Py_DECREF(bld_name);

  PyObject *bld_data =
      Py_BuildValue("s#", result.data.c_str(), result.data.length());
  PyDict_SetItemString(pyobj_bag_message, "data", bld_data);
  Py_DECREF(bld_data);

  PyObject *bld_type = Py_BuildValue("s", result.data_type.c_str());
  PyDict_SetItemString(pyobj_bag_message, "data_type", bld_type);
  Py_DECREF(bld_type);

  PyObject *bld_time = Py_BuildValue("s", "timestamp");
  PyObject *bld_rtime = Py_BuildValue("K", result.timestamp);
  PyDict_SetItem(pyobj_bag_message, bld_time, bld_rtime);
  Py_DECREF(bld_time);
  Py_DECREF(bld_rtime);

  PyObject *bld_end = Py_BuildValue("s", "end");
  PyDict_SetItem(pyobj_bag_message, bld_end, result.end ? Py_True : Py_False);
  Py_DECREF(bld_end);

  return pyobj_bag_message;
}

PyObject *cyber_PyRecordReader_GetMessageNumber(PyObject *self,
                                                PyObject *args) {
  PyObject *pyobj_reader = nullptr;
  char *channel_name = nullptr;
  if (!PyArg_ParseTuple(
          args, const_cast<char *>("Os:PyRecordReader_GetMessageNumber"),
          &pyobj_reader, &channel_name)) {
    AERROR << "PyRecordReader_GetMessageNumber:PyRecordReader failed!";
    return PyLong_FromUnsignedLongLong(0);
  }

  auto reader = (apollo::cyber::record::PyRecordReader *)PyCapsule_GetPointer(
      pyobj_reader, "apollo_cyber_record_pyrecordfilereader");
  if (nullptr == reader) {
    AINFO << "PyRecordReader_GetMessageNumber ptr is null!";
    return PyLong_FromUnsignedLongLong(0);
  }

  uint64_t num = reader->GetMessageNumber(channel_name);
  return PyLong_FromUnsignedLongLong(num);
}

PyObject *cyber_PyRecordReader_GetMessageType(PyObject *self, PyObject *args) {
  PyObject *pyobj_reader = nullptr;
  char *channel_name = nullptr;
  if (!PyArg_ParseTuple(
          args, const_cast<char *>("Os:cyber_PyRecordReader_GetMessageType"),
          &pyobj_reader, &channel_name)) {
    AINFO << "PyRecordReader_GetMessageType:PyRecordReader failed!";
    return PYOBJECT_NULL_STRING;
  }

  auto reader = (apollo::cyber::record::PyRecordReader *)PyCapsule_GetPointer(
      pyobj_reader, "apollo_cyber_record_pyrecordfilereader");
  if (nullptr == reader) {
    AINFO << "PyRecordReader_GetMessageType ptr is null!";
    return PYOBJECT_NULL_STRING;
  }

  std::string msg_type = reader->GetMessageType(channel_name);
  return PyString_FromStringAndSize(msg_type.c_str(), msg_type.size());
}

PyObject *cyber_PyRecordReader_GetProtoDesc(PyObject *self, PyObject *args) {
  PyObject *pyobj_reader = nullptr;
  char *channel_name = nullptr;
  if (!PyArg_ParseTuple(args,
                        const_cast<char *>("Os:PyRecordReader_GetProtoDesc"),
                        &pyobj_reader, &channel_name)) {
    AERROR << "PyRecordReader_GetProtoDesc failed!";
    return PYOBJECT_NULL_STRING;
  }

  auto reader = (apollo::cyber::record::PyRecordReader *)PyCapsule_GetPointer(
      pyobj_reader, "apollo_cyber_record_pyrecordfilereader");
  if (nullptr == reader) {
    AINFO << "PyRecordReader_GetProtoDesc ptr is null!";
    return PYOBJECT_NULL_STRING;
  }

  std::string pb_desc = reader->GetProtoDesc(channel_name);
  return PyString_FromStringAndSize(pb_desc.c_str(), pb_desc.size());
}

PyObject *cyber_PyRecordReader_GetHeaderString(PyObject *self, PyObject *args) {
  PyObject *pyobj_reader = nullptr;
  if (!PyArg_ParseTuple(
          args, const_cast<char *>("O:cyber_PyRecordReader_GetHeaderString"),
          &pyobj_reader)) {
    return PYOBJECT_NULL_STRING;
  }

  auto reader = (apollo::cyber::record::PyRecordReader *)PyCapsule_GetPointer(
      pyobj_reader, "apollo_cyber_record_pyrecordfilereader");
  if (nullptr == reader) {
    AINFO << "PyRecordReader_GetHeaderString ptr is null!";
    return PYOBJECT_NULL_STRING;
  }

  std::string header_string = reader->GetHeaderString();
  return PyString_FromStringAndSize(header_string.c_str(),
                                    header_string.size());
}

PyObject *cyber_PyRecordReader_Reset(PyObject *self, PyObject *args) {
  PyObject *pyobj_reader = nullptr;
  if (!PyArg_ParseTuple(args,
                        const_cast<char *>("O:cyber_PyRecordReader_Reset"),
                        &pyobj_reader)) {
    AERROR << "cyber_PyRecordReader_Reset:PyArg_ParseTuple failed!";
    Py_INCREF(Py_None);
    return Py_None;
  }

  auto reader = (apollo::cyber::record::PyRecordReader *)PyCapsule_GetPointer(
      pyobj_reader, "apollo_cyber_record_pyrecordfilereader");
  if (nullptr == reader) {
    AERROR << "PyRecordReader_Reset reader is null!";
    Py_INCREF(Py_None);
    return Py_None;
  }

  reader->Reset();
  Py_INCREF(Py_None);
  return Py_None;
}

PyObject *cyber_PyRecordReader_GetChannelList(PyObject *self, PyObject *args) {
  PyObject *pyobj_reader = nullptr;
  if (!PyArg_ParseTuple(
          args, const_cast<char *>("O:cyber_PyRecordReader_GetChannelList"),
          &pyobj_reader)) {
    AERROR << "cyber_PyRecordReader_GetChannelList:PyArg_ParseTuple failed!";
    Py_INCREF(Py_None);
    return Py_None;
  }

  auto reader = (apollo::cyber::record::PyRecordReader *)PyCapsule_GetPointer(
      pyobj_reader, "apollo_cyber_record_pyrecordfilereader");
  if (nullptr == reader) {
    AERROR << "PyRecordReader_GetChannelList reader is null!";
    Py_INCREF(Py_None);
    return Py_None;
  }

  std::set<std::string> channel_list = reader->GetChannelList();
  PyObject *pyobj_list = PyList_New(channel_list.size());
  size_t pos = 0;
  for (const std::string &channel : channel_list) {
    PyList_SetItem(pyobj_list, pos, Py_BuildValue("s", channel.c_str()));
    pos++;
  }

  return pyobj_list;
}

PyObject *cyber_new_PyRecordWriter(PyObject *self, PyObject *args) {
  apollo::cyber::record::PyRecordWriter *writer =
      new apollo::cyber::record::PyRecordWriter();
  PyObject *pyobj_rec_writer =
      PyCapsule_New(writer, "apollo_cyber_record_pyrecordfilewriter", nullptr);
  return pyobj_rec_writer;
}

PyObject *cyber_delete_PyRecordWriter(PyObject *self, PyObject *args) {
  PyObject *pyobj_rec_writer = nullptr;
  if (!PyArg_ParseTuple(args, const_cast<char *>("O:delete_PyRecordWriter"),
                        &pyobj_rec_writer)) {
    Py_INCREF(Py_None);
    return Py_None;
  }

  auto writer = (apollo::cyber::record::PyRecordWriter *)PyCapsule_GetPointer(
      pyobj_rec_writer, "apollo_cyber_record_pyrecordfilewriter");
  if (nullptr == writer) {
    AERROR << "delete_PyRecordWriter:writer is null!";
    Py_INCREF(Py_None);
    return Py_None;
  }
  delete writer;
  Py_INCREF(Py_None);
  return Py_None;
}

PyObject *cyber_PyRecordWriter_Open(PyObject *self, PyObject *args) {
  PyObject *pyobj_rec_writer = nullptr;
  char *path = nullptr;
  Py_ssize_t len = 0;
  if (!PyArg_ParseTuple(args,
                        const_cast<char *>("Os#:cyber_PyRecordWriter_Open"),
                        &pyobj_rec_writer, &path, &len)) {
    AERROR << "cyber_PyRecordWriter_Open:PyArg_ParseTuple failed!";
    Py_RETURN_FALSE;
  }

  apollo::cyber::record::PyRecordWriter *writer =
      PyObjectToPtr<apollo::cyber::record::PyRecordWriter *>(
          pyobj_rec_writer, "apollo_cyber_record_pyrecordfilewriter");

  if (nullptr == writer) {
    AERROR << "PyRecordWriter_Open:writer is null!";
    Py_RETURN_FALSE;
  }

  std::string path_str(path, len);
  if (!writer->Open(path_str)) {
    Py_RETURN_FALSE;
  }
  Py_RETURN_TRUE;
}

PyObject *cyber_PyRecordWriter_Close(PyObject *self, PyObject *args) {
  PyObject *pyobj_rec_writer = nullptr;
  if (!PyArg_ParseTuple(args, const_cast<char *>("O:delete_PyRecordWriter"),
                        &pyobj_rec_writer)) {
    Py_INCREF(Py_None);
    return Py_None;
  }

  auto writer = (apollo::cyber::record::PyRecordWriter *)PyCapsule_GetPointer(
      pyobj_rec_writer, "apollo_cyber_record_pyrecordfilewriter");
  if (nullptr == writer) {
    AERROR << "cyber_PyRecordWriter_Close: writer is null!";
    Py_INCREF(Py_None);
    return Py_None;
  }
  writer->Close();
  Py_INCREF(Py_None);
  return Py_None;
}

PyObject *cyber_PyRecordWriter_WriteChannel(PyObject *self, PyObject *args) {
  PyObject *pyobj_rec_writer = nullptr;
  char *channel = nullptr;
  char *type = nullptr;
  char *proto_desc = nullptr;
  Py_ssize_t len = 0;
  if (!PyArg_ParseTuple(
          args, const_cast<char *>("Osss#:cyber_PyRecordWriter_WriteChannel"),
          &pyobj_rec_writer, &channel, &type, &proto_desc, &len)) {
    AERROR << "cyber_PyRecordWriter_WriteChannel parsetuple failed!";
    Py_RETURN_FALSE;
  }

  auto writer = PyObjectToPtr<apollo::cyber::record::PyRecordWriter *>(
      pyobj_rec_writer, "apollo_cyber_record_pyrecordfilewriter");

  if (nullptr == writer) {
    AINFO << "cyber_PyRecordWriter_WriteChannel:writer ptr is null!";
    Py_RETURN_FALSE;
  }

  std::string proto_desc_str(proto_desc, len);
  if (!writer->WriteChannel(channel, type, proto_desc_str)) {
    Py_RETURN_FALSE;
  }
  Py_RETURN_TRUE;
}

PyObject *cyber_PyRecordWriter_WriteMessage(PyObject *self, PyObject *args) {
  PyObject *pyobj_rec_writer = nullptr;
  char *channel_name = nullptr;
  char *rawmessage = nullptr;
  Py_ssize_t len = 0;
  uint64_t time = 0;
  char *proto_desc = nullptr;
  Py_ssize_t len_desc = 0;

  if (!PyArg_ParseTuple(
          args, const_cast<char *>("Oss#Ks#:cyber_PyRecordWriter_WriteMessage"),
          &pyobj_rec_writer, &channel_name, &rawmessage, &len, &time,
          &proto_desc, &len_desc)) {
    AERROR << "cyber_PyRecordWriter_WriteMessage parsetuple failed!";
    Py_RETURN_FALSE;
  }

  auto writer = PyObjectToPtr<apollo::cyber::record::PyRecordWriter *>(
      pyobj_rec_writer, "apollo_cyber_record_pyrecordfilewriter");

  if (nullptr == writer) {
    AERROR << "cyber_PyRecordWriter_WriteMessage:writer ptr is null!";
    Py_RETURN_FALSE;
  }

  std::string rawmessage_str(rawmessage, len);
  std::string desc_str(proto_desc, len_desc);
  if (!writer->WriteMessage(channel_name, rawmessage_str, time, desc_str)) {
    AERROR << "cyber_PyRecordWriter_WriteMessage:WriteMessage failed!";
    Py_RETURN_FALSE;
  }
  Py_RETURN_TRUE;
}

PyObject *cyber_PyRecordWriter_SetSizeOfFileSegmentation(PyObject *self,
                                                         PyObject *args) {
  PyObject *pyobj_rec_writer = nullptr;
  uint64_t size_kilobytes = 0;

  if (!PyArg_ParseTuple(
          args,
          const_cast<char *>(
              "OK:cyber_PyRecordWriter_SetSizeOfFileSegmentation"),
          &pyobj_rec_writer, &size_kilobytes)) {
    AERROR
        << "cyber_PyRecordWriter_SetSizeOfFileSegmentation parsetuple failed!";
    Py_RETURN_FALSE;
  }

  auto writer = PyObjectToPtr<apollo::cyber::record::PyRecordWriter *>(
      pyobj_rec_writer, "apollo_cyber_record_pyrecordfilewriter");

  if (nullptr == writer) {
    AERROR
        << "cyber_PyRecordWriter_SetSizeOfFileSegmentation:writer ptr is null!";
    Py_RETURN_FALSE;
  }

  if (!writer->SetSizeOfFileSegmentation(size_kilobytes)) {
    AERROR << "cyber_PyRecordWriter_SetSizeOfFileSegmentation failed!";
    Py_RETURN_FALSE;
  }
  Py_RETURN_TRUE;
}

PyObject *cyber_PyRecordWriter_SetIntervalOfFileSegmentation(PyObject *self,
                                                             PyObject *args) {
  PyObject *pyobj_rec_writer = nullptr;
  uint64_t time_sec = 0;

  if (!PyArg_ParseTuple(
          args,
          const_cast<char *>(
              "OK:cyber_PyRecordWriter_SetIntervalOfFileSegmentation"),
          &pyobj_rec_writer, &time_sec)) {
    AERROR << "cyber_PyRecordWriter_SetIntervalOfFileSegmentation parsetuple "
              "failed!";
    Py_RETURN_FALSE;
  }

  auto writer = PyObjectToPtr<apollo::cyber::record::PyRecordWriter *>(
      pyobj_rec_writer, "apollo_cyber_record_pyrecordfilewriter");

  if (nullptr == writer) {
    AERROR << "cyber_PyRecordWriter_SetIntervalOfFileSegmentation:writer ptr "
              "is null!";
    Py_RETURN_FALSE;
  }

  if (!writer->SetIntervalOfFileSegmentation(time_sec)) {
    AERROR << "cyber_PyRecordWriter_SetIntervalOfFileSegmentation failed!";
    Py_RETURN_FALSE;
  }
  Py_RETURN_TRUE;
}

PyObject *cyber_PyRecordWriter_GetMessageNumber(PyObject *self,
                                                PyObject *args) {
  PyObject *pyobj_rec_writer = nullptr;
  char *channel_name = nullptr;
  if (!PyArg_ParseTuple(
          args, const_cast<char *>("Os:PyRecordWriter_GetMessageNumber"),
          &pyobj_rec_writer, &channel_name)) {
    AERROR << "PyRecordWriter_GetMessageNumber:PyRecordWriter_GetMessageNumber "
              "failed!";
    return PyLong_FromUnsignedLongLong(0);
  }

  auto writer = (apollo::cyber::record::PyRecordWriter *)PyCapsule_GetPointer(
      pyobj_rec_writer, "apollo_cyber_record_pyrecordfilewriter");
  if (nullptr == writer) {
    AERROR << "PyRecordWriter_GetMessageNumber ptr is null!";
    return PyLong_FromUnsignedLongLong(0);
  }

  uint64_t num = writer->GetMessageNumber(channel_name);
  return PyLong_FromUnsignedLongLong(num);
}

PyObject *cyber_PyRecordWriter_GetMessageType(PyObject *self, PyObject *args) {
  PyObject *pyobj_rec_writer = nullptr;
  char *channel_name = nullptr;
  if (!PyArg_ParseTuple(args,
                        const_cast<char *>("Os:PyRecordWriter_GetMessageType"),
                        &pyobj_rec_writer, &channel_name)) {
    AERROR << "PyRecordWriter_GetMessageType failed!";
    return PYOBJECT_NULL_STRING;
  }

  auto writer = (apollo::cyber::record::PyRecordWriter *)PyCapsule_GetPointer(
      pyobj_rec_writer, "apollo_cyber_record_pyrecordfilewriter");
  if (nullptr == writer) {
    AERROR << "PyRecordWriter_GetMessageType ptr is null!";
    return PYOBJECT_NULL_STRING;
  }

  std::string msg_type = writer->GetMessageType(channel_name);
  return PyString_FromStringAndSize(msg_type.c_str(), msg_type.size());
}

PyObject *cyber_PyRecordWriter_GetProtoDesc(PyObject *self, PyObject *args) {
  PyObject *pyobj_rec_writer = nullptr;
  char *channel_name = nullptr;
  if (!PyArg_ParseTuple(args,
                        const_cast<char *>("Os:PyRecordWriter_GetProtoDesc"),
                        &pyobj_rec_writer, &channel_name)) {
    AERROR << "PyRecordWriter_GetProtoDesc failed!";
    return PYOBJECT_NULL_STRING;
  }

  auto writer = (apollo::cyber::record::PyRecordWriter *)PyCapsule_GetPointer(
      pyobj_rec_writer, "apollo_cyber_record_pyrecordfilewriter");
  if (nullptr == writer) {
    AERROR << "PyRecordWriter_GetProtoDesc ptr is null!";
    return PYOBJECT_NULL_STRING;
  }

  std::string proto_desc_str = writer->GetProtoDesc(channel_name);
  return PyString_FromStringAndSize(proto_desc_str.c_str(),
                                    proto_desc_str.size());
}

static PyMethodDef _cyber_record_methods[] = {
    // PyRecordReader fun
    {"new_PyRecordReader", cyber_new_PyRecordReader, METH_VARARGS, ""},
    {"delete_PyRecordReader", cyber_delete_PyRecordReader, METH_VARARGS, ""},
    {"PyRecordReader_ReadMessage", cyber_PyRecordReader_ReadMessage,
     METH_VARARGS, ""},
    {"PyRecordReader_GetMessageNumber", cyber_PyRecordReader_GetMessageNumber,
     METH_VARARGS, ""},
    {"PyRecordReader_GetMessageType", cyber_PyRecordReader_GetMessageType,
     METH_VARARGS, ""},
    {"PyRecordReader_GetProtoDesc", cyber_PyRecordReader_GetProtoDesc,
     METH_VARARGS, ""},
    {"PyRecordReader_GetHeaderString", cyber_PyRecordReader_GetHeaderString,
     METH_VARARGS, ""},
    {"PyRecordReader_Reset", cyber_PyRecordReader_Reset, METH_VARARGS, ""},
    {"PyRecordReader_GetChannelList", cyber_PyRecordReader_GetChannelList,
     METH_VARARGS, ""},

    // PyRecordWriter fun
    {"new_PyRecordWriter", cyber_new_PyRecordWriter, METH_VARARGS, ""},
    {"delete_PyRecordWriter", cyber_delete_PyRecordWriter, METH_VARARGS, ""},
    {"PyRecordWriter_Open", cyber_PyRecordWriter_Open, METH_VARARGS, ""},
    {"PyRecordWriter_Close", cyber_PyRecordWriter_Close, METH_VARARGS, ""},
    {"PyRecordWriter_WriteChannel", cyber_PyRecordWriter_WriteChannel,
     METH_VARARGS, ""},
    {"PyRecordWriter_WriteMessage", cyber_PyRecordWriter_WriteMessage,
     METH_VARARGS, ""},
    {"PyRecordWriter_SetSizeOfFileSegmentation",
     cyber_PyRecordWriter_SetSizeOfFileSegmentation, METH_VARARGS, ""},
    {"PyRecordWriter_SetIntervalOfFileSegmentation",
     cyber_PyRecordWriter_SetIntervalOfFileSegmentation, METH_VARARGS, ""},
    {"PyRecordWriter_GetMessageNumber", cyber_PyRecordWriter_GetMessageNumber,
     METH_VARARGS, ""},
    {"PyRecordWriter_GetMessageType", cyber_PyRecordWriter_GetMessageType,
     METH_VARARGS, ""},
    {"PyRecordWriter_GetProtoDesc", cyber_PyRecordWriter_GetProtoDesc,
     METH_VARARGS, ""},

    {NULL, NULL, 0, NULL} /* sentinel */
};

/// Init function of this module
PyMODINIT_FUNC init_cyber_record(void) {
  AINFO << "init _cyber_record";
  Py_InitModule("_cyber_record", _cyber_record_methods);
}
