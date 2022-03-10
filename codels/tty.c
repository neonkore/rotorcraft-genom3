/*
 * Copyright (c) 2015-2022 LAAS/CNRS
 * All rights reserved.
 *
 * Redistribution and use  in source  and binary  forms,  with or without
 * modification, are permitted provided that the following conditions are
 * met:
 *
 *   1. Redistributions of  source  code must retain the  above copyright
 *      notice and this list of conditions.
 *   2. Redistributions in binary form must reproduce the above copyright
 *      notice and  this list of  conditions in the  documentation and/or
 *      other materials provided with the distribution.
 *
 *                                      Anthony Mallet on Mon Feb 16 2015
 */
#include <sys/stat.h>
#include <sys/types.h>
#include <sys/uio.h>

#include <err.h>
#include <errno.h>
#include <fcntl.h>
#include <poll.h>
#include <stdarg.h>
#include <stdio.h>
#include <termios.h>
#include <unistd.h>

#ifdef HAVE_LOW_LATENCY_IOCTL
# include <sys/ioctl.h>
# include <linux/serial.h> /* for TIOCGSERIAL and ASYNC_LOW_LATENCY */
#endif

#ifdef __linux__
# include <libudev.h>
#endif

#include "codels.h"


/* --- mk_open_tty --------------------------------------------------------- */

static const char *	usb_serial_to_tty(const char *serial);
static int		ftdi_sio_check_latency(int fd);

/* Open a serial port, configure to the given baud rate */
int
mk_open_tty(const char *device, uint32_t speed)
{
  const char *path;
  struct termios t;
  speed_t baud;
  int fd;

  /* select baud rate */
#ifndef B57600
# define B57600 57600U
#endif
#ifndef B115200
# define B115200 115200U
#endif
#ifndef B500000
# define B500000 500000U
#endif
#ifndef B2000000
# define B2000000 2000000U
#endif
  switch(speed) {
    case 0:		baud = 0; break;
    case 57600:		baud = B57600; break;
    case 115200:	baud = B115200; break;
    case 500000:	baud = B500000; break;
    case 2000000:	baud = B2000000; break;

    default: errno = EINVAL; return -1;
  }

  /* try to match a serial id first */
  path = usb_serial_to_tty(device);
  if (path) device = path;

  /* open non-blocking */
  fd = open(device, O_RDWR | O_NOCTTY | O_NONBLOCK);
  if (fd < 0) return fd;
  if (!isatty(fd)) {
    errno = ENOTTY;
    return -1;
  }

  /* configure line discipline */
  if (tcgetattr(fd, &t)) return -1;

  t.c_iflag = IGNBRK;
  t.c_oflag = 0;
  t.c_lflag = 0;
  t.c_cflag &= ~(CSIZE | CSTOPB | PARENB | PARODD);
  t.c_cflag |= CS8 | CREAD | CLOCAL;
  t.c_cc[VMIN] = 0;
  t.c_cc[VTIME] = 0;

  if (speed) {
    if (cfsetospeed(&t, baud)) return -1;
    if (cfsetispeed(&t, baud)) return -1;
  }

  if (tcsetattr(fd, TCSANOW, &t)) return -1;

  /* discard any pending data */
  tcflush(fd, TCIOFLUSH);

#ifdef HAVE_LOW_LATENCY_IOCTL
  /* Linux: enable low latency mode
   * see /sys/bus/usb-serial/devices/.../latency_timer
   */
  {
    struct serial_struct s;
    int e;

    if (ioctl(fd, TIOCGSERIAL, &s)) {
      e = errno;
      close(fd);
      errno = e;
      return -1;
    }

    if (!(s.flags & ASYNC_LOW_LATENCY)) {
      s.flags |= ASYNC_LOW_LATENCY;
      if (ioctl(fd, TIOCSSERIAL, &s)) {
        e = errno;
        close(fd);
        errno = e;
        return -1;
      }
    }
  }
#endif

  if (ftdi_sio_check_latency(fd)) {
    errno = EIO;
    return -1;
  }

  return fd;
}


/* --- usb_serial_to_tty --------------------------------------------------- */

/* Return a tty device matching the "serial" string */

static const char *
usb_serial_to_tty(const char *serial)
{
#ifdef __linux__
  struct udev *udev;
  struct udev_enumerate *scan = NULL;
  struct udev_list_entry *ttys, *tty;
  struct udev_device *dev = NULL, *usb;
  const char *path = NULL;

  udev = udev_new();
  if (!udev) return NULL;

  /* iterate over tty devices */
  scan = udev_enumerate_new(udev);
  if (udev_enumerate_add_match_subsystem(scan, "tty")) goto done;
  if (udev_enumerate_scan_devices(scan)) goto done;

  ttys = udev_enumerate_get_list_entry(scan);
  udev_list_entry_foreach(tty, ttys) {
    const char *sysfs, *userial;

    /* get sysfs entry for the device and create a corresponding udev_device */
    if (dev) udev_device_unref(dev);
    sysfs = udev_list_entry_get_name(tty);
    dev = udev_device_new_from_syspath(udev, sysfs);

    /* get the USB device, it any */
    usb = udev_device_get_parent_with_subsystem_devtype(
      dev, "usb", "usb_device");
    if (!usb) continue;

    userial = udev_device_get_sysattr_value(usb, "serial");
    if (!userial || strcmp(userial, serial)) continue;

    /* got a match, return the tty path */
    path = strdup(udev_device_get_devnode(dev)); /* this will leak ... */
    break;
  }
  if (dev) udev_device_unref(dev);

done:
  if (scan) udev_enumerate_unref(scan);
  if (udev) udev_unref(udev);
  return path;

#else
  (void)serial; /* -Wunused-parameter */

  return NULL; /* if needed, implement this for other OSes */
#endif
}


/* --- ftdi_sio_check_latency ---------------------------------------------- */

/* Check the latency value for ftdi_sio driver */

static int
ftdi_sio_check_latency(int fd)
{
#ifdef __linux__
  struct udev *udev;
  struct udev_device *dev, *parent;
  const char *attr;
  struct stat sb;
  int s = 0;

  /* get devno */
  if (fstat(fd, &sb)) return errno;

  /* get udev_device */
  udev = udev_new();
  if (!udev) {
    warnx("cannot use udev");
    return 1;
  }

  dev = udev_device_new_from_devnum(udev, 'c', sb.st_rdev);
  if (!dev) goto done;

  /* check if it is an ftdi_sio */
  for(parent = dev; parent; parent = udev_device_get_parent(parent)) {
    attr = udev_device_get_sysattr_value(parent, "driver");
    if (attr && !strcmp(attr, "ftdi_sio")) break;
  }
  if (!parent) goto done;

  /* access latency attribute */
  attr = udev_device_get_sysattr_value(parent, "latency_timer");
  if (!attr) {
    warnx("cannot access latency_timer");
    s = 1;
    goto done;
  }

  if (atoi(attr) > 1) {
    warnx("FTDI latency timer too high: %sms, should be 1ms", attr);
    s = 1;
  }

done:
  if (dev) udev_device_unref(dev);
  udev_unref(udev);
  return s;

#else
  (void)fd; /* -Wunused-parameter */

  return 0; /* if needed, implement this for other OSes */
#endif
}


/* --- mk_wait_msg --------------------------------------------------------- */

int
mk_wait_msg(const rotorcraft_conn_s *conn)
{
  struct pollfd pfd[conn->n];
  uint32_t i;
  int s;

  for(i = 0; i < conn->n; i++) {
    pfd[i].fd = conn->chan[i].fd;
    pfd[i].events = POLLIN;
  }

  s = poll(pfd, conn->n, 500/*ms*/);

  for(i = 0; i < conn->n; i++)
    if (pfd[i].revents & POLLHUP) {
      close(pfd[i].fd);

      /* cheating with const. Oh well... */
      ((struct mk_channel_s *)&conn->chan[i])->fd = -1;
      warnx("disconnected from %s", conn->chan[i].path);
    }

  return s;
}


/* --- mk_recv_msg --------------------------------------------------------- */

/* returns: 0: timeout/incomplete, -1: error, 1: complete msg */

int
mk_recv_msg(struct mk_channel_s *chan, bool block)
{
  struct iovec iov[2];
  ssize_t s;
  uint8_t c;

  if (chan->fd < 0) return -1;

  do {
    /* feed the ring  buffer */
    iov[0].iov_base = chan->buf + chan->w;
    iov[1].iov_base = chan->buf;

    if (chan->r > chan->w) {
      iov[0].iov_len = chan->r - chan->w - 1;
      iov[1].iov_len = 0;
    } else if (chan->r > 0) {
      iov[0].iov_len = sizeof(chan->buf) - chan->w;
      iov[1].iov_len = chan->r - 1;
    } else {
      iov[0].iov_len = sizeof(chan->buf) - chan->w - 1;
      iov[1].iov_len = 0;
    }

    if (iov[0].iov_len || iov[1].iov_len) {
      do {
        s = readv(chan->fd, iov, 2);
      } while(s < 0 && errno == EINTR);

      if (s < 0)
        return -1;
      else if (s == 0 && chan->start && block) {
        struct pollfd fd = { .fd = chan->fd, .events = POLLIN };

        s = poll(&fd, 1, 500/*ms*/);
        if (fd.revents & POLLHUP) return -1;
      } else if (s == 0 && chan->r == chan->w)
        return 0;
      else
        chan->w = (chan->w + s) % sizeof(chan->buf);
    }

    while(chan->r != chan->w) {
      c = chan->buf[chan->r];
      chan->r = (chan->r + 1) % sizeof(chan->buf);

      switch(c) {
        case '^':
        chan->start = true;
        chan->escape = false;
        chan->len = 0;
        break;

        case '$':
          if (!chan->start) break;
          chan->start = false;

          switch(chan->msg[0]) {
            case 'N': /* info messages */
              warnx("hardware info: %.*s", chan->len-1, &chan->msg[1]);
              break;
            case 'A': /* warning messages */
              warnx("hardware warning: %.*s", chan->len-1, &chan->msg[1]);
              break;
            case 'E': /* error messages */
              warnx("hardware error: %.*s", chan->len-1, &chan->msg[1]);
              break;

            default: return 1;
          }
          break;

        case '!':
          chan->start = false;
          break;

        case '\\':
          chan->escape = true;
          break;

        default:
          if (!chan->start) break;
          if (chan->len >= sizeof(chan->msg)) {
            chan->start = false; break;
          }

          if (chan->escape) {
            c = ~c; chan->escape = false;
          }
          chan->msg[chan->len++] = c;
          break;
      }
    }
  } while(1);

  return 0;
}


/* --- mk_send_msg --------------------------------------------------------- */

static void	mk_encode(char x, char **buf);

int
mk_send_msg(const struct mk_channel_s *chan, const char *fmt, ...)
{
  va_list ap;
  ssize_t s;
  char buf[64], *w, *r;
  char c;

  if (chan->fd < 0) return -1;

  w = buf;

  va_start(ap, fmt);
  *w++ = '^';
  while((c = *fmt++)) {
    while ((unsigned)(w - buf) > sizeof(buf)-8 /* 8 = worst case (4 bytes
                                                * escaped) */) {
      do {
        s = write(chan->fd, buf, w - buf);
      } while (s < 0 && errno == EINTR);
      if (s < 0) return -1;

      if (s > 0 && s < w - buf) memmove(buf, buf + s, w - buf - s);
      w -= s;
    }

    switch(c) {
      case '%': {
        switch(*fmt++) {
          case '1':
            mk_encode(va_arg(ap, int/*promotion*/), &w);
            break;

          case '2': {
            uint16_t x = va_arg(ap, int/*promotion*/);
            mk_encode((x >> 8) & 0xff, &w);
            mk_encode(x & 0xff, &w);
            break;
          }
          case '@': {
            uint16_t *x = va_arg(ap, uint16_t *);
            size_t l = va_arg(ap, size_t);
            while (l--) {
              mk_encode((*x >> 8) & 0xff, &w);
              mk_encode(*x & 0xff, &w);
              x++;
            }
            break;
          }

          case '4': {
            uint32_t x = va_arg(ap, uint32_t);
            mk_encode((x >> 24) & 0xff, &w);
            mk_encode((x >> 16) & 0xff, &w);
            mk_encode((x >> 8) & 0xff, &w);
            mk_encode(x & 0xff, &w);
            break;
          }

          case '%':
            mk_encode(c, &w);
            break;
        }
        break;
      }

      default:
        mk_encode(c, &w);
    }
  }
  *w++ = '$';
  va_end(ap);

  r = buf;
  while (w > r) {
    do {
      s = write(chan->fd, r, w - r);
    } while (s < 0 && errno == EINTR);
    if (s < 0) return -1;

    r += s;
  }

  return 0;
}

static void
mk_encode(char x, char **buf)
{
  switch (x) {
    case '^': case '$': case '\\': case '!':
      *(*buf)++ = '\\';
      x = ~x;
  }
  *(*buf)++ = x;
}
