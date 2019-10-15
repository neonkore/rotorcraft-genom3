/*
 * Copyright (c) 2015-2019 LAAS/CNRS
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
#include "acrotorcraft.h"

#include <sys/time.h>

#include <assert.h>
#include <err.h>
#include <errno.h>
#include <float.h>
#include <fnmatch.h>
#include <math.h>
#include <stdbool.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <termios.h>
#include <unistd.h>

#include "rotorcraft_c_types.h"
#include "codels.h"

static void	mk_get_ts(uint8_t seq, struct timeval rtv, double rate,
                        rotorcraft_ids_sensor_time_s_ts_s *timings,
                        or_time_ts *ts, double *lprate);


/* --- Task comm -------------------------------------------------------- */

/** Codel mk_comm_start of task comm.
 *
 * Triggered by rotorcraft_start.
 * Yields to rotorcraft_poll.
 * Throws rotorcraft_e_sys.
 */
genom_event
mk_comm_start(rotorcraft_conn_s **conn, const genom_context self)
{
  *conn = malloc(sizeof(**conn));
  if (!*conn) return mk_e_sys_error(NULL, self);

  (*conn)->chan.fd = -1;
  (*conn)->chan.r = (*conn)->chan.w = 0;
  (*conn)->device = RC_NONE;

  return rotorcraft_poll;
}


/** Codel mk_comm_poll of task comm.
 *
 * Triggered by rotorcraft_poll.
 * Yields to rotorcraft_nodata, rotorcraft_recv.
 * Throws rotorcraft_e_sys.
 */
genom_event
mk_comm_poll(const rotorcraft_conn_s *conn, const genom_context self)
{
  int s;

  s = mk_wait_msg(&conn->chan);
  if (s < 0) {
    if (errno != EINTR) return mk_e_sys_error(NULL, self);
    return rotorcraft_nodata;
  }
  else if (s == 0) return rotorcraft_nodata;

  return rotorcraft_recv;
}


/** Codel mk_comm_nodata of task comm.
 *
 * Triggered by rotorcraft_nodata.
 * Yields to rotorcraft_poll.
 * Throws rotorcraft_e_sys.
 */
genom_event
mk_comm_nodata(rotorcraft_conn_s **conn,
               rotorcraft_ids_sensor_time_s *sensor_time,
               const rotorcraft_imu *imu, const rotorcraft_mag *mag,
               rotorcraft_ids_battery_s *battery,
               const genom_context self)
{
  or_pose_estimator_state *idata = imu->data(self);
  or_pose_estimator_state *mdata = mag->data(self);

  /* reset exported data in case of timeout */
  idata->avel._present = false;
  idata->avel._value.wx = idata->avel._value.wy = idata->avel._value.wz =
    nan("");
  idata->acc._present = false;
  idata->acc._value.ax = idata->acc._value.ay = idata->acc._value.az =
    nan("");

  mdata->att._present = false;
  mdata->att._value.qw =
    idata->att._value.qy =
    idata->att._value.qz =
    idata->att._value.qx = nan("");

  battery->level = 0.;

  if (mk_set_sensor_rate(&sensor_time->rate, *conn, sensor_time, self))
    mk_disconnect_start(conn, self);

  return rotorcraft_poll;
}


/** Codel mk_comm_recv of task comm.
 *
 * Triggered by rotorcraft_recv.
 * Yields to rotorcraft_poll, rotorcraft_recv.
 * Throws rotorcraft_e_sys.
 */
genom_event
mk_comm_recv(rotorcraft_conn_s **conn,
             const rotorcraft_ids_imu_calibration_s *imu_calibration,
             rotorcraft_ids_sensor_time_s *sensor_time,
             const rotorcraft_imu *imu, const rotorcraft_mag *mag,
             rotorcraft_ids_rotor_data_s *rotor_data,
             rotorcraft_ids_battery_s *battery,
             const genom_context self)
{
  struct timeval tv;
  int more;
  size_t i;
  uint8_t *msg, len;
  int16_t v16;
  uint16_t u16;

  more = 0;
  if (mk_recv_msg(&(*conn)->chan, false) == 1) {
    more = 1;
    gettimeofday(&tv, NULL);

    msg = (*conn)->chan.msg;
    len = (*conn)->chan.len;
    switch(*msg++) {
      case 'I': /* IMU data */
        if (len == 14) {
          or_pose_estimator_state *idata = imu->data(self);
          double v[3];
          uint8_t seq = *msg++;

          if (seq == sensor_time->imu.seq) break;

          mk_get_ts(
            seq, tv, sensor_time->rate.imu, &sensor_time->imu,
            &idata->ts, &sensor_time->measured_rate.imu);

          v16 = ((int16_t)(*msg++) << 8);
          v16 |= ((uint16_t)(*msg++) << 0);
          v[0] = v16/1000. + imu_calibration->abias[0];

          v16 = ((int16_t)(*msg++) << 8);
          v16 |= ((uint16_t)(*msg++) << 0);
          v[1] = v16/1000. + imu_calibration->abias[1];

          v16 = ((int16_t)(*msg++) << 8);
          v16 |= ((uint16_t)(*msg++) << 0);
          v[2] = v16/1000. + imu_calibration->abias[2];

          idata->acc._value.ax =
            imu_calibration->ascale[0] * v[0] +
            imu_calibration->ascale[1] * v[1] +
            imu_calibration->ascale[2] * v[2];
          idata->acc._value.ay =
            imu_calibration->ascale[3] * v[0] +
            imu_calibration->ascale[4] * v[1] +
            imu_calibration->ascale[5] * v[2];
          idata->acc._value.az =
            imu_calibration->ascale[6] * v[0] +
            imu_calibration->ascale[7] * v[1] +
            imu_calibration->ascale[8] * v[2];

          v16 = ((int16_t)(*msg++) << 8);
          v16 |= ((uint16_t)(*msg++) << 0);
          v[0] = v16/1000. + imu_calibration->gbias[0];

          v16 = ((int16_t)(*msg++) << 8);
          v16 |= ((uint16_t)(*msg++) << 0);
          v[1] = v16/1000. + imu_calibration->gbias[1];

          v16 = ((int16_t)(*msg++) << 8);
          v16 |= ((uint16_t)(*msg++) << 0);
          v[2] = v16/1000. + imu_calibration->gbias[2];

          idata->avel._value.wx =
            imu_calibration->gscale[0] * v[0] +
            imu_calibration->gscale[1] * v[1] +
            imu_calibration->gscale[2] * v[2];
          idata->avel._value.wy =
            imu_calibration->gscale[3] * v[0] +
            imu_calibration->gscale[4] * v[1] +
            imu_calibration->gscale[5] * v[2];
          idata->avel._value.wz =
            imu_calibration->gscale[6] * v[0] +
            imu_calibration->gscale[7] * v[1] +
            imu_calibration->gscale[8] * v[2];

          idata->avel._present = true;
          idata->acc._present = true;
        } else
          warnx("bad IMU message");
        break;

      case 'C': /* magnetometer data */
        if (len == 8) {
          or_pose_estimator_state *mdata = mag->data(self);
          double v[3];
          uint8_t seq = *msg++;

          if (seq == sensor_time->mag.seq) break;

          mk_get_ts(&tv, &sensor_time->mag,
                    &mdata->ts, &sensor_time->measured_rate.mag);

          v16 = ((int16_t)(*msg++) << 8);
          v16 |= ((uint16_t)(*msg++) << 0);
          v[0] = v16/1e8;

          v16 = ((int16_t)(*msg++) << 8);
          v16 |= ((uint16_t)(*msg++) << 0);
          v[1] = v16/1e8;

          v16 = ((int16_t)(*msg++) << 8);
          v16 |= ((uint16_t)(*msg++) << 0);
          v[2] = v16/1e8;

          mdata->att._value.qx = v[0];
          mdata->att._value.qy = v[1];
          mdata->att._value.qz = v[2];

          mdata->att._present = true;
        } else
          warnx("bad magnetometer message");
        break;

      case 'M': /* motor data */
        if (len == 9) {
          uint8_t seq = *msg++;
          uint8_t state = *msg++;
          uint8_t id = state & 0xf;

          if (id < 1 || id > or_rotorcraft_max_rotors) break;
          id--;
          if (seq == sensor_time->motor[id].seq) break;

          if (!rotor_data->state[id].ts.sec && rotor_data->state[id].disabled)
            rotor_data->state[id].disabled = 0;

          mk_get_ts(
            seq, tv, sensor_time->rate.motor, &sensor_time->motor[id],
            &rotor_data->state[id].ts, &sensor_time->measured_rate.motor);

          rotor_data->state[id].emerg = !!(state & 0x80);
          rotor_data->state[id].spinning = !!(state & 0x20);
          rotor_data->state[id].starting = !!(state & 0x10);

          v16 = ((int16_t)(*msg++) << 8);
          v16 |= ((uint16_t)(*msg++) << 0);
          if (rotor_data->state[id].spinning)
            rotor_data->state[id].velocity = v16 ? 1e6/2/v16 : 0.;
          else
            rotor_data->state[id].velocity = 0.;

          v16 = ((int16_t)(*msg++) << 8);
          v16 |= ((uint16_t)(*msg++) << 0);
          rotor_data->state[id].throttle = v16 * 100./1023.;

          u16 = ((uint16_t)(*msg++) << 8);
          u16 |= ((uint16_t)(*msg++) << 0);
          rotor_data->state[id].consumption = u16 / 1e3;
        } else
          warnx("bad motor data message");
        break;

      case 'B': /* battery data */
        if (len == 4) {
          uint8_t seq  __attribute__((unused)) = *msg++;
          double p;

          u16 = ((uint16_t)(*msg++) << 8);
          u16 |= ((uint16_t)(*msg++) << 0);
          battery->level = u16/1000.;

          p = 100. *
              (battery->level - battery->min)/(battery->max - battery->min);
          for(i = 0; i < or_rotorcraft_max_rotors; i++)
            rotor_data->state[i].energy_level = p;
        } else
          warnx("bad battery message");
        break;

      case 'T': /* clock rate */
        if (len == 3) {
          uint8_t id = *msg++;

          if (id < 1 || id > or_rotorcraft_max_rotors) break;
          id--;
          rotor_data->clkrate[id] = *msg;
        } else
          warnx("bad clock rate message");
        break;

      case '?': /* ignored messages */
        break;

      default:
        warnx("received unknown message");
    }
  }

  return more ? rotorcraft_recv : rotorcraft_poll;
}


/** Codel mk_comm_stop of task comm.
 *
 * Triggered by rotorcraft_stop.
 * Yields to rotorcraft_ether.
 * Throws rotorcraft_e_sys.
 */
genom_event
mk_comm_stop(rotorcraft_conn_s **conn, const genom_context self)
{
  if (!*conn) return rotorcraft_ether;

  mk_send_msg(&(*conn)->chan, "x");
  mk_set_sensor_rate(
    &(struct rotorcraft_ids_sensor_time_s_rate_s){ 0 }, *conn, NULL, self);

  return rotorcraft_ether;
}


/* --- Activity connect ------------------------------------------------- */

/** Codel mk_connect_start of activity connect.
 *
 * Triggered by rotorcraft_start.
 * Yields to rotorcraft_ether.
 * Throws rotorcraft_e_sys, rotorcraft_e_baddev.
 */
genom_event
mk_connect_start(const char serial[64], uint32_t baud,
                 rotorcraft_conn_s **conn,
                 rotorcraft_ids_sensor_time_s *sensor_time,
                 const genom_context self)
{
  /* supported devices */
  static const struct {
    enum rc_device dev; const char *match; double rev;
  } rc_devices[] = {
    { .dev = RC_MKBL,		.match = "%*cmkbl%lf",	1.8 },
    { .dev = RC_MKFL,		.match = "mkfl%lf",	1.8 },
    { .dev = RC_FLYMU,		.match = "flymu%lf",	1.8 },
    { .dev = RC_CHIMERA,	.match = "chimera%lf",	1.0 }
  };

  double rev;
  size_t c;
  int s;

  if ((*conn)->chan.fd >= 0) {
    close((*conn)->chan.fd);
    warnx("disconnected from %s", (*conn)->chan.path);
  }

  /* open tty */
  (*conn)->chan.fd = mk_open_tty(serial, baud);
  if ((*conn)->chan.fd < 0) return mk_e_sys_error(serial, self);

  (*conn)->chan.r = (*conn)->chan.w = 0;
  (*conn)->chan.start = (*conn)->chan.escape = false;

  /* check endpoint */
  while (mk_recv_msg(&(*conn)->chan, true) == 1); /* flush buffer */
  do {
    c = 0;
    do {
      if (mk_send_msg(&(*conn)->chan, "?")) /* ask for id */
        return mk_e_sys_error(serial, self);

      s = mk_wait_msg(&(*conn)->chan);
      if (s < 0 && errno != EINTR) return mk_e_sys_error(NULL, self);
      if (s > 0) break;
    } while(c++ < 3);
    if (c > 3) {
      errno = ETIMEDOUT;
      return mk_e_sys_error(NULL, self);
    }

    s = mk_recv_msg(&(*conn)->chan, true);
  } while(s == 1 && (*conn)->chan.msg[0] != '?');
  if (s != 1) {
    errno = ENOMSG;
    return mk_e_sys_error(NULL, self);
  }

  /* match device */
  (*conn)->chan.msg[(*conn)->chan.len] = 0;
  (*conn)->device = RC_NONE;
  for (c = 0; c < sizeof(rc_devices)/sizeof(rc_devices[0]); c++) {
    if (sscanf((char *)&(*conn)->chan.msg[1], rc_devices[c].match, &rev) != 1)
      continue;
    if (rev < rc_devices[c].rev) {
      rotorcraft_e_baddev_detail d;
      snprintf(d.dev, sizeof(d.dev), "hardware device version `%g' too old, "
               "version `%g' or newer is required", rev, rc_devices[c].rev);
      close((*conn)->chan.fd);
      (*conn)->chan.fd = -1;
      return rotorcraft_e_baddev(&d, self);
    }

    (*conn)->device = rc_devices[c].dev;
    break;
  }
  if ((*conn)->device == RC_NONE) {
    rotorcraft_e_baddev_detail d;
    snprintf(d.dev, sizeof(d.dev), "unsupported hardware device `%s'",
             &(*conn)->chan.msg[1]);
    close((*conn)->chan.fd);
    (*conn)->chan.fd = -1;
    return rotorcraft_e_baddev(&d, self);
  }

  snprintf((*conn)->chan.path, sizeof((*conn)->chan.path), "%s", serial);
  warnx("connected to %s, %s", &(*conn)->chan.msg[1], (*conn)->chan.path);

  /* configure data streaming */
  mk_set_sensor_rate(&sensor_time->rate, *conn, sensor_time, self);

  return rotorcraft_ether;
}


/* --- Activity disconnect ---------------------------------------------- */

/** Codel mk_disconnect_start of activity disconnect.
 *
 * Triggered by rotorcraft_start.
 * Yields to rotorcraft_ether.
 * Throws rotorcraft_e_sys.
 */
genom_event
mk_disconnect_start(rotorcraft_conn_s **conn,
                    const genom_context self)
{
  mk_send_msg(&(*conn)->chan, "x");
  mk_set_sensor_rate(
    &(struct rotorcraft_ids_sensor_time_s_rate_s){
      .imu = 0, .motor = 0, .battery = 0
        }, *conn, NULL, self);

  if ((*conn)->chan.fd >= 0) {
    close((*conn)->chan.fd);
    warnx("disconnected from %s", (*conn)->chan.path);
  }
  (*conn)->chan.fd = -1;

  return rotorcraft_ether;
}


/* --- Activity monitor ------------------------------------------------- */

/** Codel mk_monitor_check of activity monitor.
 *
 * Triggered by rotorcraft_start, rotorcraft_sleep.
 * Yields to rotorcraft_pause_sleep, rotorcraft_ether.
 * Throws rotorcraft_e_sys.
 */
genom_event
mk_monitor_check(const rotorcraft_conn_s *conn,
                 const genom_context self)
{
  (void)self;

  if (conn->chan.fd >= 0) return rotorcraft_pause_sleep;

  return rotorcraft_ether;
}


/* --- mk_get_ts ----------------------------------------------------------- */

/** Implements Olson, Edwin. "A passive solution to the sensor synchronization
 * problem." International conference on Intelligent Robots and Systems (IROS),
 * 2010 IEEE/RSJ */
static void
mk_get_ts(uint8_t seq, struct timeval rtv, double rate,
          rotorcraft_ids_sensor_time_s_ts_s *timings, or_time_ts *ts,
          double *lprate)
{
  double ats, df;
  uint8_t ds;
  assert(rate > 0.);

  /* delta samples */
  ds = seq - timings->seq;
  if (ds > 16)
    /* if too many samples were lost, we might have missed more than 255
     * samples: reset the offset */
    timings->offset = -DBL_MAX;
  else
    /* consider a 0.5% clock drift on the sender side */
    timings->offset -= 0.005 * ds / rate;

  /* update remote timestamp */
  timings->ts += ds / rate;
  timings->seq = seq;

  /* update offset */
  ats = rtv.tv_sec + 1e-6 * rtv.tv_usec;
  if (timings->ts - ats > timings->offset)
    timings->offset = timings->ts - ats;

  /* local timestamp - reset offset if it diverged too much from realtime,
   * maybe the sensor is not sending at the specified rate */
  if (ats - (timings->ts - timings->offset) > 2. / rate)
    timings->offset = timings->ts - ats;
  else
    ats = timings->ts - timings->offset;

  /* update estimated rate */
  df = 1. / (ats - ts->sec - ts->nsec * 1e-9);

  if (df > timings->rmed)
    timings->rerr = (timings->rerr + 1.) / 2.;
  else
    timings->rerr = (timings->rerr - 1.) / 2.;

  if (fabs(timings->rerr) > 0.75)
    timings->rgain *= 2;
  else
    timings->rgain /= 2;
  if (timings->rgain < 0.1) timings->rgain = 0.1;

  if (df > timings->rmed)
    timings->rmed += timings->rgain;
  else
    timings->rmed -= timings->rgain;

  *lprate += 0.25 * (timings->rmed - *lprate);

  /* update timestamp */
  ts->sec = floor(ats);
  ts->nsec = (ats - ts->sec) * 1e9;
}
