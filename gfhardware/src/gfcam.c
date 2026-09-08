/*
 * python-gfhardware _cam module
 * Python extension to grab a single raw frame from an imx-media V4L2 capture
 * node, debayer it, and return it as a JPEG.
 * Copyright 2020-2026 514 LLC d/b/a OpenGlow
 * Written by Scott Wiederhold
 * Released under the MIT license.
 * SPDX-License-Identifier: MIT
 *
 * Portions Based on python-v4l2capture
 * 2009, 2010, 2011 Fredrik Portstrom
 * and V4L2 sample kernel code at "Documentation/media/uapi/v4l/v4l2grab.c",
 * both in the public domain.
 *
 * The factory firmware drove a single NXP mxc_v4l2_capture node (/dev/video0)
 * that exposed the sensor controls and a private CSI camera-select control.
 * ForgeFIRM runs mainline imx-media: a media-controller graph whose links, pad
 * formats, sensor controls and illumination are configured out-of-band (see
 * gfhardware/cam.py) before this code simply streams the capture node. So all
 * this extension does now is open the node, set the raw-Bayer capture format,
 * grab one frame, debayer it, and JPEG-encode it.
 *
 * Two sensors ship in the field: the 5 MP OV5648 delivers 8-bit samples, and
 * the 8 MP OV8856 in "HD" machines delivers 10-bit samples that the IPU CSI
 * writes as little-endian 16-bit words. `depth` selects between them; a
 * deeper frame is narrowed to 8 bits before the shared demosaic.
 */
#include <Python.h>
#include <fcntl.h>
#include <string.h>
#include <linux/videodev2.h>
#include <sys/mman.h>
#include <libv4l2.h>
#include <stdio.h>
#include <jpeglib.h>

#include "bayer.h"

#define CLEAR(x) memset(&(x), 0, sizeof(x))

struct buffer {
  void *start;
  size_t length;
};

static int _ioctl(int fd, int request, void *arg) {
  for(;;) {
    int result = v4l2_ioctl(fd, request, arg);
    if(!result)
      return 0;
    if(errno != EINTR) {
      /* -1 with errno set: the call sites check `< 0`, so a positive errno
       * return would leave failures undetected and later stages reading
       * wrong-size or stale buffers. */
      return -1;
    }
  }
}

static PyObject *method_grab(PyObject *self, PyObject *args) {
  const char *dev_path;
  int width;
  int height;
  int hflip = 0;
  int depth = 8;

  if(!PyArg_ParseTuple(args, "sii|ii", &dev_path, &width, &height, &hflip,
                       &depth)) {
    return NULL;
  }
  if (depth != 8 && depth != 10) {
    return PyErr_Format(PyExc_ValueError, "depth must be 8 or 10, not %d",
                        depth);
  }

  // All error paths must release everything acquired so far: a leaked fd
  // keeps vb2 queue ownership, so the NEXT grab's REQBUFS fails -EBUSY and
  // one transient failure (e.g. an EOF timeout) permanently wedges capture
  // in a long-running process.
  struct buffer *buffers = NULL;
  unsigned char *rgb_map = MAP_FAILED;
  uint32_t rgb_size = 0;
  unsigned char *raw8 = NULL;   /* narrowing target when depth > 8 */
  int n_buffers = 0;
  int streaming = 0;
  struct v4l2_buffer buf;
  enum v4l2_buf_type type = V4L2_BUF_TYPE_VIDEO_CAPTURE;

  // Open device
  int dev_fd = open(dev_path, O_RDWR | O_NONBLOCK, 0);
  if (dev_fd == -1)
    return PyErr_Format(PyExc_IOError, "failed to open %s", dev_path);

  // Validate capabilities
  struct v4l2_capability cap;
  if (_ioctl(dev_fd, VIDIOC_QUERYCAP, &cap) < 0) {
    PyErr_Format(PyExc_IOError, "VIDIOC_QUERYCAP failed");
    goto fail;
  }
  if (!(cap.capabilities & V4L2_CAP_VIDEO_CAPTURE)) {
    PyErr_Format(PyExc_IOError, "%s is not a capture device", dev_path);
    goto fail;
  }
  if (!(cap.capabilities & V4L2_CAP_STREAMING)) {
    PyErr_Format(PyExc_IOError, "%s is not a streaming device", dev_path);
    goto fail;
  }

  // Set capture format. The imx-media pipeline has already been configured for
  // this Bayer format and geometry by gfhardware/cam.py; this just matches the
  // node. A 10-bit media-bus code maps to the 16-bit-per-sample pixel format.
  struct v4l2_format fmt;
  CLEAR(fmt);
  fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  fmt.fmt.pix.width = width;
  fmt.fmt.pix.height = height;
  fmt.fmt.pix.pixelformat = depth > 8 ? V4L2_PIX_FMT_SBGGR16
                                      : V4L2_PIX_FMT_SBGGR8;
  fmt.fmt.pix.field = V4L2_FIELD_NONE;
  if (_ioctl(dev_fd, VIDIOC_S_FMT, &fmt) < 0) {
    PyErr_Format(PyExc_IOError, "VIDIOC_S_FMT failed");
    goto fail;
  }

  // Request and map buffers
  struct v4l2_requestbuffers req;
  CLEAR(req);
  req.count = 2;
  req.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  req.memory = V4L2_MEMORY_MMAP;
  if (_ioctl(dev_fd, VIDIOC_REQBUFS, &req) < 0) {
    PyErr_Format(PyExc_IOError, "VIDIOC_REQBUFS failed");
    goto fail;
  }
  if (req.count < 2) {
    PyErr_Format(PyExc_IOError, "Insufficient buffers");
    goto fail;
  }

  buffers = calloc(req.count, sizeof(*buffers));
  if (!buffers) {
    PyErr_Format(PyExc_MemoryError, "failed to allocate buffers");
    goto fail;
  }

  for (n_buffers = 0; n_buffers < (int)req.count; ++n_buffers) {
    CLEAR(buf);
    buf.type        = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    buf.memory      = V4L2_MEMORY_MMAP;
    buf.index       = n_buffers;
    if (_ioctl(dev_fd, VIDIOC_QUERYBUF, &buf) < 0) {
      PyErr_Format(PyExc_IOError, "VIDIOC_QUERYBUF failed");
      goto fail;
    }

    buffers[n_buffers].length = buf.length;
    buffers[n_buffers].start = mmap(NULL, buf.length,
              PROT_READ | PROT_WRITE, MAP_SHARED, dev_fd, buf.m.offset);

    if (MAP_FAILED == buffers[n_buffers].start) {
      buffers[n_buffers].start = NULL;
      PyErr_Format(PyExc_IOError, "mmap failed");
      goto fail;
    }
  }

  // Queue buffers
  for (int i = 0; i < n_buffers; ++i) {
    CLEAR(buf);
    buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    buf.memory = V4L2_MEMORY_MMAP;
    buf.index = i;

    if (_ioctl(dev_fd, VIDIOC_QBUF, &buf) < 0) {
      PyErr_Format(PyExc_IOError, "VIDIOC_QBUF failed");
      goto fail;
    }
  }

  // Stream on
  if (_ioctl(dev_fd, VIDIOC_STREAMON, &type) < 0) {
    PyErr_Format(PyExc_IOError, "VIDIOC_STREAMON failed");
    goto fail;
  }
  streaming = 1;

  // Wait for and dequeue a frame
  for (;;) {
    fd_set fds;
    struct timeval tv;
    int ret;

    FD_ZERO(&fds);
    FD_SET(dev_fd, &fds);

    tv.tv_sec = 2;
    tv.tv_usec = 0;

    ret = select(dev_fd + 1, &fds, NULL, NULL, &tv);

    if (ret == -1) {
      if (EINTR == errno)
        continue;
      PyErr_Format(PyExc_IOError, "select failed");
      goto fail;
    }

    if (ret == 0) {
      PyErr_Format(PyExc_IOError, "select timeout");
      goto fail;
    }

    CLEAR(buf);
    buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    buf.memory = V4L2_MEMORY_MMAP;

    if (_ioctl(dev_fd, VIDIOC_DQBUF, &buf) < 0) {
      if ((errno == EAGAIN) || (errno == EIO))
        continue;
      PyErr_Format(PyExc_IOError, "VIDIOC_DQBUF failed");
      goto fail;
    } else
      break;
  }

  // Convert Bayer to RGB
  rgb_size = (uint32_t)width * (uint32_t)height * 3;
  rgb_map = mmap(NULL, rgb_size, PROT_READ | PROT_WRITE,
        MAP_SHARED | MAP_ANONYMOUS, -1, 0);
  if (rgb_map == MAP_FAILED) {
    PyErr_Format(PyExc_MemoryError, "rgb_map mmap failed");
    goto fail;
  }

  // A deeper frame is narrowed to the 8-bit samples the demosaic takes.
  // Samples are little-endian in the 16-bit word and clamped on the way
  // down, so a source that turns out not to be right-aligned saturates
  // rather than wrapping.
  const uint8_t *bayer = (const uint8_t *)buffers[buf.index].start;
  if (depth > 8) {
    size_t n = (size_t)width * (size_t)height;
    raw8 = malloc(n);
    if (!raw8) {
      PyErr_Format(PyExc_MemoryError, "raw8 malloc failed");
      goto fail;
    }
    int shift = depth - 8;
    for (size_t i = 0; i < n; i++) {
      unsigned v = ((unsigned)bayer[2 * i] | ((unsigned)bayer[2 * i + 1] << 8))
                   >> shift;
      raw8[i] = (uint8_t)(v > 255 ? 255 : v);
    }
    bayer = raw8;
  }

  dc1394_bayer_decoding_8bit(
    bayer, (uint8_t*)rgb_map, width, height,
    DC1394_COLOR_FILTER_BGGR, DC1394_BAYER_METHOD_BILINEAR);

  // Stop stream and release capture resources (shared with the error path).
  Py_BEGIN_ALLOW_THREADS
  _ioctl(dev_fd, VIDIOC_STREAMOFF, &type);
  for (int i = 0; i < n_buffers; i++) {
    if (buffers[i].start)
      munmap(buffers[i].start, buffers[i].length);
  }
  free(buffers);
  free(raw8);
  close(dev_fd);
  Py_END_ALLOW_THREADS
  buffers = NULL;
  raw8 = NULL;
  dev_fd = -1;

  // JPEG encode. The ov5648 HFLIP register breaks imx-media CSI capture (frames
  // never complete -> EOF timeout), so the factory's mirrored image orientation
  // is applied here in software: when hflip is set, each debayered RGB scanline
  // is written reversed.
  unsigned char *flip_row = NULL;
  if (hflip) {
    flip_row = malloc((size_t)width * 3);
    if (!flip_row) {
      munmap(rgb_map, rgb_size);
      return PyErr_Format(PyExc_MemoryError, "flip_row malloc failed");
    }
  }

  unsigned char *jpg_buf = NULL;
  unsigned long jpg_buf_size = 0;
  struct jpeg_compress_struct jpg_cinfo;
  struct jpeg_error_mgr jpg_err;
  JSAMPROW row_pointer[1];
  jpg_cinfo.err = jpeg_std_error(&jpg_err);
  jpeg_create_compress(&jpg_cinfo);
  jpeg_mem_dest(&jpg_cinfo, &jpg_buf, &jpg_buf_size);

  jpg_cinfo.image_width = width;
  jpg_cinfo.image_height = height;
  jpg_cinfo.input_components = 3;
  jpg_cinfo.in_color_space = JCS_RGB;
  jpeg_set_defaults(&jpg_cinfo);
  jpeg_set_quality(&jpg_cinfo, 75, TRUE);
  jpeg_start_compress(&jpg_cinfo, TRUE);

  Py_BEGIN_ALLOW_THREADS
  while (jpg_cinfo.next_scanline < jpg_cinfo.image_height) {
    unsigned char *src = &rgb_map[jpg_cinfo.next_scanline * width * 3];
    if (hflip) {
      for (int x = 0; x < width; x++) {
        flip_row[x * 3 + 0] = src[(width - 1 - x) * 3 + 0];
        flip_row[x * 3 + 1] = src[(width - 1 - x) * 3 + 1];
        flip_row[x * 3 + 2] = src[(width - 1 - x) * 3 + 2];
      }
      row_pointer[0] = flip_row;
    } else {
      row_pointer[0] = src;
    }
    jpeg_write_scanlines(&jpg_cinfo, row_pointer, 1);
  }
  jpeg_finish_compress(&jpg_cinfo);
  jpeg_destroy_compress(&jpg_cinfo);
  Py_END_ALLOW_THREADS

  PyObject *result = PyBytes_FromStringAndSize((char *)jpg_buf, jpg_buf_size);

  Py_BEGIN_ALLOW_THREADS
  free(jpg_buf);
  free(flip_row);
  munmap(rgb_map, rgb_size);
  Py_END_ALLOW_THREADS

  return result;

fail:
  // Unified failure cleanup; the Python exception is already set.
  if (streaming)
    _ioctl(dev_fd, VIDIOC_STREAMOFF, &type);
  if (buffers) {
    for (int i = 0; i < n_buffers; i++) {
      if (buffers[i].start)
        munmap(buffers[i].start, buffers[i].length);
    }
    free(buffers);
  }
  free(raw8);
  if (rgb_map != MAP_FAILED)
    munmap(rgb_map, rgb_size);
  if (dev_fd >= 0)
    close(dev_fd);
  return NULL;
}

static PyMethodDef module_methods[] = {
  {"grab", method_grab, METH_VARARGS,
    "grab(device: str, width: int, height: int, hflip: int = 0,\n"
    "     depth: int = 8) -> bytes\n\n"
    "Streams one raw BGGR frame from the given V4L2 capture node, debayers\n"
    "it to RGB, optionally mirrors it horizontally (hflip), and returns it as\n"
    "a JPEG. depth is the sensor's bits per sample: 8 (OV5648, SBGGR8) or 10\n"
    "(OV8856, SBGGR16 with the sample right-aligned in the word). The\n"
    "imx-media pipeline, sensor controls and illumination must already be\n"
    "configured (see cam.py)."},
  {NULL}
};

PyMODINIT_FUNC PyInit__cam(void) {
  static struct PyModuleDef moduledef = {
    PyModuleDef_HEAD_INIT,
    "_cam", "Glowforge raw-frame grabber",
    -1, module_methods, NULL, NULL, NULL, NULL
  };
  return PyModule_Create(&moduledef);
}
