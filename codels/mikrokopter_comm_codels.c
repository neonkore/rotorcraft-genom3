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
 *					Anthony Mallet on Fri Feb 13 2015
 */
#include "acmikrokopter.h"

#include <sys/time.h>

#include <err.h>
#include <errno.h>
#include <math.h>
#include <stdbool.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <termios.h>
#include <unistd.h>

#include "mikrokopter_c_types.h"
#include "codels.h"


/* --- Task comm -------------------------------------------------------- */


/** Codel mk_comm_start of task comm.
 *
 * Triggered by mikrokopter_start.
 * Yields to mikrokopter_poll.
 * Throws mikrokopter_e_sys.
 */
genom_event
mk_comm_start(mikrokopter_conn_s **conn, genom_context self)
{
  int i;

  *conn = malloc(sizeof(**conn));
  if (!*conn) return mk_e_sys_error(NULL, self);

  for(i = 0; i < mk_channels(); i++) {
    (*conn)->chan[i].fd = -1;
    (*conn)->chan[i].r = (*conn)->chan[i].w = 0;
  }
  return mikrokopter_poll;
}


/** Codel mk_comm_poll of task comm.
 *
 * Triggered by mikrokopter_poll.
 * Yields to mikrokopter_poll, mikrokopter_recv.
 * Throws mikrokopter_e_sys.
 */
genom_event
mk_comm_poll(const mikrokopter_conn_s *conn, genom_context self)
{
  int s;

  s = mk_wait_msg(conn->chan, mk_channels());
  if (s < 0) {
    if (errno != EINTR) return mk_e_sys_error(NULL, self);
    return mikrokopter_poll;
  }
  else if (s == 0) return mikrokopter_poll;

  return mikrokopter_recv;
}


/** Codel mk_comm_recv of task comm.
 *
 * Triggered by mikrokopter_recv.
 * Yields to mikrokopter_poll.
 * Throws mikrokopter_e_sys.
 */
genom_event
mk_comm_recv(mikrokopter_conn_s **conn,
             const mikrokopter_ids_imu_calibration_s *imu_calibration,
             const mikrokopter_imu *imu,
             const mikrokopter_rotors *rotors,
             mikrokopter_ids_battery_s *battery, genom_context self)
{
  int i;
  uint8_t *msg, len;
  int16_t v16;
  uint16_t u16;
  struct timeval tv;

  for(i = 0; i < mk_channels(); i++) {
    while(mk_recv_msg(&(*conn)->chan[i], false) == 1) {
      gettimeofday(&tv, NULL);
      msg = (*conn)->chan[i].msg;
      len = (*conn)->chan[i].len;
      switch(*msg++) {
        case 'I': /* IMU data */
          if (len == 13) {
            or_pose_estimator_state *idata = imu->data(self);

            idata->ts.sec = tv.tv_sec;
            idata->ts.nsec = tv.tv_usec * 1000;

            v16 = ((int16_t)(*msg++) << 8);
            v16 |= ((uint16_t)(*msg++) << 0);
            idata->acc._value.ax = v16/1000.;

            v16 = ((int16_t)(*msg++) << 8);
            v16 |= ((uint16_t)(*msg++) << 0);
            idata->acc._value.ay = v16/1000.;

            v16 = ((int16_t)(*msg++) << 8);
            v16 |= ((uint16_t)(*msg++) << 0);
            idata->acc._value.az = v16/1000.;

            idata->vel._value.vx = nan("");
            idata->vel._value.vy = nan("");
            idata->vel._value.vz = nan("");

            v16 = ((int16_t)(*msg++) << 8);
            v16 |= ((uint16_t)(*msg++) << 0);
            idata->vel._value.wx = v16/1000. + imu_calibration->wx_off;

            v16 = ((int16_t)(*msg++) << 8);
            v16 |= ((uint16_t)(*msg++) << 0);
            idata->vel._value.wy = v16/1000. + imu_calibration->wy_off;

            v16 = ((int16_t)(*msg++) << 8);
            v16 |= ((uint16_t)(*msg++) << 0);
            idata->vel._value.wz = v16/1000. + imu_calibration->wz_off;

            idata->vel._present = true;
            idata->acc._present = true;
          }
          break;

        case 'M': /* motor data */
          if (len == 8) {
            mikrokopter_rotors_s *rdata = rotors->data(self);
            uint8_t state = *msg++;
            uint8_t id = state & 0xf;

            if (id < 1 || id > rdata->_maximum) break;
            if (id > rdata->_length) rdata->_length = id;
            id--;

            rdata->_buffer[id].emerg = !!(state & 0x80);
            rdata->_buffer[id].spinning = !!(state & 0x20);
            rdata->_buffer[id].starting = !!(state & 0x10);

            u16 = ((uint16_t)(*msg++) << 8);
            u16 |= ((uint16_t)(*msg++) << 0);
            if (rdata->_buffer[id].spinning)
              rdata->_buffer[id].velocity =  (u16 > 0) ? 1e6/u16 : 0;
            else
              rdata->_buffer[id].velocity = 0;

            u16 = ((uint16_t)(*msg++) << 8);
            u16 |= ((uint16_t)(*msg++) << 0);
            rdata->_buffer[id].pwm = u16 * 100./1024.;

            u16 = ((uint16_t)(*msg++) << 8);
            u16 |= ((uint16_t)(*msg++) << 0);
            rdata->_buffer[id].current = u16 / 1e3;
          }
          break;

        case 'B': /* battery data */
          if (len == 3) {
            u16 = ((uint16_t)(*msg++) << 8);
            u16 |= ((uint16_t)(*msg++) << 0);
            battery->level = u16/1000.;
          }
          break;

        case 'Z': /* calibration done */
          if (len == 1) {
            /* calibration_done = true; */
          }
          break;
      }
    }
  }

  return mikrokopter_poll;
}


/** Codel mk_comm_stop of task comm.
 *
 * Triggered by mikrokopter_stop.
 * Yields to mikrokopter_ether.
 * Throws mikrokopter_e_sys.
 */
genom_event
mk_comm_stop(mikrokopter_conn_s **conn, genom_context self)
{
  if (!*conn) return mikrokopter_ether;

  mk_send_msg(&(*conn)->chan[0], "x");
  mk_set_sensor_rate(&(struct mikrokopter_ids_sensor_rate_s){
      .imu = 0, .motor = 0, .battery = 0
        }, *conn, self);

  return mikrokopter_ether;
}


/* --- Activity connect ------------------------------------------------- */

/** Codel mk_connect_start of activity connect.
 *
 * Triggered by mikrokopter_start.
 * Yields to mikrokopter_ether.
 * Throws mikrokopter_e_sys, mikrokopter_e_baddev.
 */
genom_event
mk_connect_start(const char serial[2][64], uint32_t baud,
                 mikrokopter_conn_s **conn,
                 const mikrokopter_ids_sensor_rate_s *sensor_rate,
                 genom_context self)
{
  static const char magic[] = "?mkfl";

  speed_t s;
  int i, c;

  for(i = 0; i < mk_channels(); i++) {
    if ((*conn)->chan[i].fd >= 0) {
      close((*conn)->chan[i].fd);
      warnx("disconnected from %s", (*conn)->chan[i].path);
    }

    if (!*serial[i]) {
      (*conn)->chan[i].fd = -1;
      continue;
    }

    /* open tty */
    (*conn)->chan[i].fd = mk_open_tty(serial[i], baud);
    if ((*conn)->chan[i].fd < 0) return mk_e_sys_error(serial[i], self);

    /* check endpoint */
    while (mk_recv_msg(&(*conn)->chan[i], true) == 1); /* flush buffer */
    c = 0;
    do {
      do {
        if (mk_send_msg(&(*conn)->chan[i], "?")) /* ask for id */
          return mk_e_sys_error(serial[i], self);

        s = mk_wait_msg(&(*conn)->chan[i], 1);
        if (s < 0 && errno != EINTR) return mk_e_sys_error(NULL, self);
        if (s > 0) break;
      } while(c++ < 3);

      s = mk_recv_msg(&(*conn)->chan[i], true);
    } while(s == 1 && (*conn)->chan[i].msg[0] != magic[0]);

    if (s != 1 ||
        strncmp((char *)(*conn)->chan[i].msg, magic, sizeof(magic)-1)) {
      mikrokopter_e_baddev_detail d;
      strncpy(d.dev, serial[i], sizeof(d.dev));

      close((*conn)->chan[i].fd);
      (*conn)->chan[i].fd = -1;
      return mikrokopter_e_baddev(&d, self);
    }

    snprintf((*conn)->chan[i].path, sizeof((*conn)->chan[i].path),
             "%s", serial[i]);
    warnx("connected to %s", (*conn)->chan[i].path);
  }

  /* configure data streaming */
  mk_set_sensor_rate(sensor_rate, *conn, self);

  return mikrokopter_ether;
}


/* --- Activity disconnect ---------------------------------------------- */

/** Codel mk_disconnect_start of activity disconnect.
 *
 * Triggered by mikrokopter_start.
 * Yields to mikrokopter_ether.
 * Throws mikrokopter_e_sys.
 */
genom_event
mk_disconnect_start(mikrokopter_conn_s **conn, genom_context self)
{
  int i;

  mk_send_msg(&(*conn)->chan[0], "x");
  mk_set_sensor_rate(&(struct mikrokopter_ids_sensor_rate_s){
      .imu = 0, .motor = 0, .battery = 0
        }, *conn, self);

  for(i = 0; i < mk_channels(); i++) {
    if ((*conn)->chan[i].fd >= 0) {
      close((*conn)->chan[i].fd);
      warnx("disconnected from %s", (*conn)->chan[i].path);
    }
    (*conn)->chan[i].fd = -1;
  }

  return mikrokopter_ether;
}


/* --- Activity monitor ------------------------------------------------- */

/** Codel mk_monitor_check of activity monitor.
 *
 * Triggered by mikrokopter_start, mikrokopter_sleep.
 * Yields to mikrokopter_sleep, mikrokopter_ether.
 * Throws mikrokopter_e_sys.
 */
genom_event
mk_monitor_check(const mikrokopter_conn_s *conn, genom_context self)
{
  int i;

  for(i = 0; i < mk_channels(); i++)
    if (conn->chan[i].fd >= 0) return mikrokopter_sleep;

  return mikrokopter_ether;
}
