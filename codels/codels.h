/*
 * Copyright (c) 2015 LAAS/CNRS
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
#ifndef H_MIKROKOPTER_CODELS
#define H_MIKROKOPTER_CODELS

#include <errno.h>
#include <stdbool.h>
#include <string.h>
#include <termios.h>

#include "mikrokopter_c_types.h"

struct mk_channel_s {
  char path[1024];
  int fd;

  uint8_t buf[64], r, w; /* read ring buffer */

  bool start;
  bool escape;
  uint8_t msg[64], len; /* last message */
};

struct mikrokopter_conn_s {
  struct mk_channel_s chan[2];
};

static inline size_t
mk_channels(void)
{
  mikrokopter_conn_s *c;
  return sizeof(c->chan)/sizeof(c->chan[0]);
}

static inline genom_event
mk_e_sys_error(const char *s, genom_context self)
{
  mikrokopter_e_sys_detail d;
  size_t l = 0;

  d.code = errno;
  if (s) {
    strncpy(d.what, s, sizeof(d.what) - 3);
    l = strlen(s);
    strcpy(d.what + l, ": ");
    l += 2;
  }
  strerror_r(d.code, d.what + l, sizeof(d.what) - l);
  return mikrokopter_e_sys(&d, self);
}

int	mk_open_tty(const char *device, speed_t baud);
int	mk_wait_msg(const struct mk_channel_s *channels, int n);
int	mk_recv_msg(struct mk_channel_s *chan, bool block);
int	mk_send_msg(const struct mk_channel_s *chan, const char *fmt, ...);

#endif /* H_MIKROKOPTER_CODELS */
