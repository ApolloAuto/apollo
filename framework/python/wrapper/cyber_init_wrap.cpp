#include <Python.h>
#include <string.h>
#include <iostream>
#include <memory>

#include "py_init.h"

static PyObject *cyber_py_init(PyObject *self, PyObject *args) {
  bool is_init = apollo::cybertron::py_init();
  if (is_init) {
    Py_RETURN_TRUE;
  } else {
    Py_RETURN_FALSE;
  }
}

static PyObject *cyber_py_ok(PyObject *self, PyObject *args) {
  bool is_ok = apollo::cybertron::py_ok();
  if (is_ok) {
    Py_RETURN_TRUE;
  } else {
    Py_RETURN_FALSE;
  }
}

static PyObject *cyber_py_shutdown(PyObject *self, PyObject *args) {
  apollo::cybertron::py_shutdown();
  return Py_None;
}

static PyObject *cyber_py_is_shutdown(PyObject *self, PyObject *args) {
  bool is_shutdown = apollo::cybertron::py_is_shutdown();
  if (is_shutdown) {
    Py_RETURN_TRUE;
  } else {
    Py_RETURN_FALSE;
  }
}

static PyObject *cyber_py_waitforshutdown(PyObject *self, PyObject *args) {
  apollo::cybertron::py_waitforshutdown();
  return Py_None;
}


/////////////////////////////////////////////////////////////////////
//// global for whole page, init module
/////////////////////////////////////////////////////////////////////
static PyMethodDef _cyber_init_methods[] = {
    // global fun
    {"py_init", cyber_py_init, METH_NOARGS, ""},
    {"py_ok", cyber_py_ok, METH_NOARGS, ""},
    {"py_shutdown", cyber_py_shutdown, METH_NOARGS, ""},
    {"py_is_shutdown", cyber_py_is_shutdown, METH_NOARGS, ""},
    {"py_waitforshutdown", cyber_py_waitforshutdown, METH_NOARGS, ""},

    {NULL, NULL, 0, NULL} /* sentinel */
};

/// Init function of this module
PyMODINIT_FUNC init_cyber_init(void) {
  AINFO << "init _cyber_init";
  Py_InitModule("_cyber_init", _cyber_init_methods);
}
