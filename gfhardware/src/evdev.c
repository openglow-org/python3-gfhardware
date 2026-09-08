/*
 * Copyright 2020 514 LLC d/b/a OpenGlow
 * Written by Scott Wiederhold
 * https://community.openglow.org
 * SPDX-License-Identifier:    MIT
 *
 * Based on python-evdev
 * Copyright (c) 2012-2016 Georgi Valkov. All rights reserved.
 *
 * Python bindings to certain linux input subsystem functions.
 */

#include <Python.h>

#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <errno.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>
#include <linux/input.h>

#ifndef input_event_sec
#define input_event_sec time.tv_sec
#define input_event_usec time.tv_usec
#endif

#define MAX_NAME_SIZE 256

extern char*  EV_NAME[EV_CNT];
extern int    EV_TYPE_MAX[EV_CNT];
extern char** EV_TYPE_NAME[EV_CNT];
extern char*  BUS_NAME[];


int test_bit(const char* bitmask, int bit)
{
    return bitmask[bit/8] & (1 << (bit % 8));
}


// Read input event from a device and return a tuple that mimics input_event
static PyObject * device_read(PyObject *self, PyObject *args)
{
    int fd;
    struct input_event event;

    // get device file descriptor (O_RDONLY|O_NONBLOCK)
    if (!PyArg_ParseTuple(args, "i", &fd))
        return NULL;

    int n = read(fd, &event, sizeof(event));

    if (n < 0) {
        if (errno == EAGAIN) {
            Py_INCREF(Py_None);
            return Py_None;
        }

        PyErr_SetFromErrno(PyExc_IOError);
        return NULL;
    }

    PyObject* sec  = PyLong_FromLong(event.input_event_sec);
    PyObject* usec = PyLong_FromLong(event.input_event_usec);
    PyObject* val  = PyLong_FromLong(event.value);
    PyObject* py_input_event = NULL;

    py_input_event = Py_BuildValue("OOhhO", sec, usec, event.type, event.code, val);
    Py_DECREF(sec);
    Py_DECREF(usec);
    Py_DECREF(val);

    return py_input_event;
}


// Read multiple input events from a device and return a list of tuples
static PyObject * device_read_many(PyObject *self, PyObject *args)
{
    int fd;

    // get device file descriptor (O_RDONLY|O_NONBLOCK)
    int ret = PyArg_ParseTuple(args, "i", &fd);
    if (!ret) return NULL;

    PyObject* event_list = PyList_New(0);
    PyObject* py_input_event = NULL;
    PyObject* sec  = NULL;
    PyObject* usec = NULL;
    PyObject* val  = NULL;

    struct input_event event[64];

    size_t event_size = sizeof(struct input_event);
    ssize_t nread = read(fd, event, event_size*64);

    if (nread < 0) {
        if (errno == EAGAIN)
            return PyList_New(0);       /* no events: an empty batch, not an error */
        PyErr_SetFromErrno(PyExc_OSError);
        return NULL;
    }

    // Construct a list of event tuples, which we'll make sense of in Python
    for (unsigned i = 0 ; i < nread/event_size ; i++) {
        sec  = PyLong_FromLong(event[i].input_event_sec);
        usec = PyLong_FromLong(event[i].input_event_usec);
        val  = PyLong_FromLong(event[i].value);

        py_input_event = Py_BuildValue("OOhhO", sec, usec, event[i].type, event[i].code, val);
        PyList_Append(event_list, py_input_event);

        Py_DECREF(py_input_event);
        Py_DECREF(sec);
        Py_DECREF(usec);
        Py_DECREF(val);
    }

    return event_list;
}


static PyObject * ioctl_EVIOCGRAB(PyObject *self, PyObject *args)
{
    int fd, ret, flag;
    ret = PyArg_ParseTuple(args, "ii", &fd, &flag);
    if (!ret) return NULL;

    ret = ioctl(fd, EVIOCGRAB, (intptr_t)flag);
    if (ret != 0) {
        PyErr_SetFromErrno(PyExc_IOError);
        return NULL;
    }

    Py_INCREF(Py_None);
    return Py_None;
}


static PyObject * ioctl_EVIOCG_bits(PyObject *self, PyObject *args)
{
    int max, fd, evtype, ret;

    ret = PyArg_ParseTuple(args, "ii", &fd, &evtype);
    if (!ret) return NULL;

    switch (evtype) {
    case EV_SW:
        max = SW_MAX; break;
    default:
        PyErr_SetString(PyExc_ValueError, "unsupported event type");
        return NULL;
    }

    char bytes[(max+7)/8];
    memset(bytes, 0, sizeof bytes);

    switch (evtype) {
    case EV_SW:
        ret = ioctl(fd, EVIOCGSW(sizeof(bytes)), &bytes);
        break;
    }

    if (ret == -1)
        return PyErr_SetFromErrno(PyExc_OSError);

    PyObject* res = PyList_New(0);
    for (int i=0; i<max; i++) {
        if (test_bit(bytes, i)) {
            PyObject *bit = PyLong_FromLong(i);
            PyList_Append(res, bit);
            Py_DECREF(bit);
        }
    }

    return res;
}


static PyMethodDef MethodTable[] = {
    { "ioctl_EVIOCGRAB",      ioctl_EVIOCGRAB,      METH_VARARGS},
    { "ioctl_EVIOCG_bits",    ioctl_EVIOCG_bits,    METH_VARARGS, "get state of KEY|LED|SND|SW"},
    { "device_read",          device_read,          METH_VARARGS, "read an input event from a device" },
    { "device_read_many",     device_read_many,     METH_VARARGS, "read all available input events from a device" },

    { NULL, NULL, 0, NULL}
};


#define MODULE_NAME "input.evdev"
#define MODULE_HELP "Python bindings to attach to Glowforge input switches"


static struct PyModuleDef moduledef = {
    PyModuleDef_HEAD_INIT,
    MODULE_NAME,
    MODULE_HELP,
    -1,          /* m_size */
    MethodTable, /* m_methods */
    NULL,        /* m_reload */
    NULL,        /* m_traverse */
    NULL,        /* m_clear */
    NULL,        /* m_free */
};


static PyObject * moduleinit(void)
{
    PyObject* m = PyModule_Create(&moduledef);
    if (m == NULL) return NULL;
    return m;
}


PyMODINIT_FUNC PyInit_evdev(void)
{
    return moduleinit();
}
