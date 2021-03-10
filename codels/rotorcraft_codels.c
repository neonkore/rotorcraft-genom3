/*
 * Copyright (c) 2015-2021 LAAS/CNRS
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
#include <fcntl.h>
#include <float.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include <unistd.h>

#include "rotorcraft_c_types.h"
#include "codels.h"


/* --- Attribute set_sensor_rate ---------------------------------------- */

/** Validation codel mk_set_sensor_rate of attribute set_sensor_rate.
 *
 * Returns genom_ok.
 * Throws .
 */
genom_event
mk_set_sensor_rate(const rotorcraft_ids_sensor_time_s_rate_s *rate,
                   const rotorcraft_conn_s *conn,
                   rotorcraft_ids_imu_filter_s *imu_filter,
                   rotorcraft_ids_sensor_time_s *sensor_time,
                   const genom_context self)
{
  int i;

  if (rate->imu < 0. || rate->imu > 2000. ||
      rate->mag < 0. || rate->mag > 2000. ||
      rate->motor < 0. || rate->motor > 2000. ||
      rate->battery < 0. || rate->battery > 2000.)
    return rotorcraft_e_range(self);

  if (sensor_time) {
    sensor_time->imu.offset = -DBL_MAX;
    sensor_time->mag.offset = -DBL_MAX;
    sensor_time->battery.offset = -DBL_MAX;
    for(i = 0; i < or_rotorcraft_max_rotors; i++) {
      sensor_time->motor[i].offset = -DBL_MAX;
    }
  }

  /* reconfigure existing connection */
  if (conn && conn->chan.fd >= 0) {
    uint32_t p;

    p = rate->battery > 0. ? 1000000/rate->battery : 0;
    mk_send_msg(&conn->chan, "b%4", p);
    p = rate->motor > 0. ? 1000000/rate->motor : 0;
    mk_send_msg(&conn->chan, "m%4", p);
    p = rate->imu > 0. ? 1000000/rate->imu : 0;
    mk_send_msg(&conn->chan, "i%4", p);
    p = rate->mag > 0. ? 1000000/rate->mag : 0;
    mk_send_msg(&conn->chan, "c%4", p);
  }

  /* reconfigure filters */
  if (imu_filter) {
    double gfc[3], afc[3], mfc[3];

    rc_get_imu_filter(imu_filter, &sensor_time->rate /* old rate */,
                      gfc, afc, mfc, self);
    rc_set_imu_filter(gfc, afc, mfc, rate /* new rate */, imu_filter, self);
  }

  return genom_ok;
}


/* --- Attribute set_battery_limits ------------------------------------- */

/** Validation codel mk_set_battery_limits of attribute set_battery_limits.
 *
 * Returns genom_ok.
 * Throws rotorcraft_e_range.
 */
genom_event
mk_set_battery_limits(double min, double max,
                      const genom_context self)
{
  if (min < 0.) return rotorcraft_e_range(self);
  if (min >= max - 1e-2) return rotorcraft_e_range(self);
  return genom_ok;
}


/* --- Attribute set_imu_calibration ------------------------------------ */

/** Validation codel mk_set_imu_calibration of attribute set_imu_calibration.
 *
 * Returns genom_ok.
 * Throws .
 */
genom_event
mk_set_imu_calibration(bool *imu_calibration_updated,
                       const genom_context self)
{
  (void)self;

  *imu_calibration_updated = true;
  return genom_ok;
}


/* --- Function set_velocity -------------------------------------------- */

/** Validation codel mk_validate_input of function set_velocity.
 *
 * Returns genom_ok.
 * Throws rotorcraft_e_connection, rotorcraft_e_rotor_failure.
 */
genom_event
mk_validate_input(const or_rotorcraft_rotor_state state[8],
                  or_rotorcraft_rotor_control *desired,
                  const genom_context self)
{
  rotorcraft_e_rotor_failure_detail e;
  size_t i, l;

  /* check rotors status */
  for(i = 0; i < or_rotorcraft_max_rotors; i++) {
    if (state[i].disabled) continue;
    if (state[i].emerg) {
      e.id = 1 + i;
      return rotorcraft_e_rotor_failure(&e, self);
    }
  }

  /* discard trailing nans */
  l = desired->_length;
  while(l && isnan(desired->_buffer[l-1])) l--;
  desired->_length = l;
  return genom_ok;
}


/* --- Function set_throttle -------------------------------------------- */

/** Validation codel mk_validate_input of function set_throttle.
 *
 * Returns genom_ok.
 * Throws rotorcraft_e_connection, rotorcraft_e_rotor_failure.
 */
/* already defined in service set_velocity validation */



/* --- Function get_imu_filter ------------------------------------------ */

/** Codel rc_get_imu_filter of function get_imu_filter.
 *
 * Returns genom_ok.
 */
genom_event
rc_get_imu_filter(const rotorcraft_ids_imu_filter_s *imu_filter,
                  const rotorcraft_ids_sensor_time_s_rate_s *rate,
                  double gfc[3], double afc[3], double mfc[3],
                  const genom_context self)
{
  (void)self;
  double wc;
  unsigned int i;

  wc = rate->imu * 0.5 / M_PI;
  for(i = 0; i < 3; i++) {
    if (imu_filter->galpha[i] < 1.)
      gfc[i] = wc * imu_filter->galpha[i] / (1. - imu_filter->galpha[i]);
    else
      gfc[i] = 0.;

    if (imu_filter->aalpha[i] < 1.)
      afc[i] = wc * imu_filter->aalpha[i] / (1. - imu_filter->aalpha[i]);
    else
      afc[i] = 0.;
  }

  wc = rate->mag * 0.5 / M_PI;
  for(i = 0; i < 3; i++)
    if (imu_filter->malpha[i] < 1.)
      mfc[i] = wc * imu_filter->malpha[i] / (1. - imu_filter->malpha[i]);
    else
      mfc[i] = 0.;

  return genom_ok;
}


/* --- Function set_imu_filter ------------------------------------------ */

/** Codel rc_set_imu_filter of function set_imu_filter.
 *
 * Returns genom_ok.
 */
genom_event
rc_set_imu_filter(const double gfc[3], const double afc[3],
                  const double mfc[3],
                  const rotorcraft_ids_sensor_time_s_rate_s *rate,
                  rotorcraft_ids_imu_filter_s *imu_filter,
                  const genom_context self)
{
  (void)self;
  double wc;
  unsigned int i;

  if (rate->imu > 0.)
    wc = 2 * M_PI / rate->imu;
  else
    wc = 0.;

  for(i = 0; i < 3; i++) {
    if (gfc[i] > 0.)
      imu_filter->galpha[i] = wc * gfc[i] / (wc * gfc[i] + 1);
    else
      imu_filter->galpha[i] = 1.;

    if (afc[i] > 0.)
      imu_filter->aalpha[i] = wc * afc[i] / (wc * afc[i] + 1);
    else
      imu_filter->aalpha[i] = 1.;
  }

  if (rate->mag > 0.)
    wc = 2 * M_PI / rate->mag;
  else
    wc = 0.;

  for(i = 0; i < 3; i++)
    if (mfc[i] > 0.)
      imu_filter->malpha[i] = wc * mfc[i] / (wc * mfc[i] + 1);
    else
      imu_filter->malpha[i] = 1.;

  return genom_ok;
}


/* --- Function disable_motor ------------------------------------------- */

/** Codel mk_disable_motor of function disable_motor.
 *
 * Returns genom_ok.
 */
genom_event
mk_disable_motor(uint16_t motor, const rotorcraft_conn_s *conn,
                 or_rotorcraft_rotor_state state[8],
                 const genom_context self)
{
  struct timeval tv;

  if (motor < 1 || motor > or_rotorcraft_max_rotors)
    return rotorcraft_e_range(self);

  gettimeofday(&tv, NULL);
  state[motor - 1] = (or_rotorcraft_rotor_state){
    .ts = { .sec = tv.tv_sec, .nsec = tv.tv_usec * 1000 },
    .emerg = false, .spinning = false, .starting = false, .disabled = true,
    .velocity = nan(""), .throttle = nan(""), .consumption = nan(""),
    .energy_level = nan("")
  };

  if (conn) mk_send_msg(&conn->chan, "x%1", (uint8_t){motor});

  return genom_ok;
}


/* --- Function enable_motor -------------------------------------------- */

/** Codel mk_enable_motor of function enable_motor.
 *
 * Returns genom_ok.
 */
genom_event
mk_enable_motor(uint16_t motor, const rotorcraft_conn_s *conn,
                or_rotorcraft_rotor_state state[8],
                const genom_context self)
{
  struct timeval tv;

  if (motor < 1 || motor > or_rotorcraft_max_rotors)
    return rotorcraft_e_range(self);

  gettimeofday(&tv, NULL);
  state[motor - 1] = (or_rotorcraft_rotor_state){
    .ts = { .sec = tv.tv_sec, .nsec = tv.tv_usec * 1000 },
    .emerg = false, .spinning = false, .starting = false, .disabled = false
  };

  if (conn) {
    size_t i;
    for(i = 0; i < or_rotorcraft_max_rotors; i++) {
      if (state[i].disabled) continue;
      if (!state[i].spinning) continue;

      mk_send_msg(&conn->chan, "g%1", (uint8_t){motor});
      break;
    }
  }

  return genom_ok;
}


/* --- Function set_velocity -------------------------------------------- */

/** Codel mk_set_velocity of function set_velocity.
 *
 * Returns genom_ok.
 * Throws rotorcraft_e_connection, rotorcraft_e_rotor_failure.
 */
genom_event
mk_set_velocity(const rotorcraft_conn_s *conn,
                rotorcraft_ids_rotor_data_s *rotor_data,
                const or_rotorcraft_rotor_control *desired,
                const genom_context self)
{
  int16_t p[or_rotorcraft_max_rotors];
  size_t i, l;
  (void)self;

  l = desired->_length;
  if (l == 0) return genom_ok;

  /* rotational period */
  for(i = 0; i < l; i++) {
    rotor_data->wd[i] =
      rotor_data->state[i].disabled ? 0. : desired->_buffer[i];
    if (isnan(rotor_data->wd[i])) rotor_data->wd[i] = 0.;

    p[i] = (fabs(rotor_data->wd[i]) < 1000000./65535.) ?
      copysign(32767, rotor_data->wd[i]) : 1000000/2/rotor_data->wd[i];
  }

  /* send */
  mk_send_msg(&conn->chan, "w%@", p, l);
  return genom_ok;
}


/* --- Function set_throttle -------------------------------------------- */

/** Codel mk_set_throttle of function set_throttle.
 *
 * Returns genom_ok.
 * Throws rotorcraft_e_connection, rotorcraft_e_rotor_failure.
 */
genom_event
mk_set_throttle(const rotorcraft_conn_s *conn,
                rotorcraft_ids_rotor_data_s *rotor_data,
                const or_rotorcraft_rotor_control *desired,
                const genom_context self)
{
  int16_t p[or_rotorcraft_max_rotors];
  size_t i, l;
  (void)self;

  l = desired->_length;
  if (l == 0) return genom_ok;

  /* convert to -1023..1023 */
  for(i = 0; i < l; i++) {
    rotor_data->wd[i] = 0.;
    if (isnan(desired->_buffer[i]))
      p[i] = 0.;
    else
      p[i] =
        rotor_data->state[i].disabled ? 0. : desired->_buffer[i] * 1023./100.;
  }

  /* send */
  mk_send_msg(&conn->chan, "q%@", p, l);
  return genom_ok;
}


/* --- Function log ----------------------------------------------------- */

/** Codel mk_log_start of function log.
 *
 * Returns genom_ok.
 * Throws rotorcraft_e_sys.
 */
genom_event
mk_log_start(const char path[64], uint32_t decimation,
             const rotorcraft_ids_imu_calibration_s *imu_calibration,
             rotorcraft_log_s **log, const genom_context self)
{
  time_t t;
  int fd;
  int s;

  t = time(NULL);

  fd = open(path, O_WRONLY|O_APPEND|O_CREAT|O_TRUNC, 0666);
  if (fd < 0) return mk_e_sys_error(path, self);

  s = dprintf(
    fd,
    "# %s" /* ctime(3) has a \n */
    "# IMU calibration\n"
#define mk_log_cal(x)                           \
    "# " #x "scale {\n"                         \
    "#  0 %20g  1 %20g  2 %20g\n"               \
    "#  3 %20g  4 %20g  5 %20g\n"               \
    "#  6 %20g  7 %20g  8 %20g\n"               \
    "# }\n"                                     \
    "# " #x "bias {\n"                          \
    "#  0 %20g  1 %20g  2 %20g\n"               \
    "# }\n"                                     \
    "# " #x "stddev {\n"                        \
    "#  0 %20g  1 %20g  2 %20g\n"               \
    "# }\n"

    mk_log_cal(g) mk_log_cal(a) mk_log_cal(m)
#undef mk_log_cal
    "\n"
    rotorcraft_log_header "\n",

    ctime(&t),
#define mk_calm(x, y) imu_calibration->x ## y
#define mk_log_cal(x)                                                   \
    mk_calm(x, scale)[0], mk_calm(x, scale)[1], mk_calm(x, scale)[2],   \
    mk_calm(x, scale)[3], mk_calm(x, scale)[4], mk_calm(x, scale)[5],   \
    mk_calm(x, scale)[6], mk_calm(x, scale)[7], mk_calm(x, scale)[8],   \
    mk_calm(x, bias)[0], mk_calm(x, bias)[1], mk_calm(x, bias)[2],      \
    mk_calm(x, stddev)[0], mk_calm(x, stddev)[1], mk_calm(x, stddev)[2]

    mk_log_cal(g), mk_log_cal(a), mk_log_cal(m)
#undef mk_log_cal
#undef mk_calm
    );
  if (s < 0) return mk_e_sys_error(path, self);

  if ((*log)->req.aio_fildes >= 0) {
    close((*log)->req.aio_fildes);

    if ((*log)->pending)
      while (aio_error(&(*log)->req) == EINPROGRESS)
        /* empty body */;
  }
  (*log)->req.aio_fildes = fd;
  (*log)->pending = false;
  (*log)->skipped = false;
  (*log)->decimation = decimation < 1 ? 1 : decimation;
  (*log)->missed = 0;
  (*log)->total = 0;

  return genom_ok;
}


/* --- Function log_stop ------------------------------------------------ */

/** Codel mk_log_stop of function log_stop.
 *
 * Returns genom_ok.
 */
genom_event
mk_log_stop(rotorcraft_log_s **log, const genom_context self)
{
  (void)self; /* -Wunused-parameter */

  if (*log && (*log)->req.aio_fildes >= 0)
    close((*log)->req.aio_fildes);
  (*log)->req.aio_fildes = -1;

  return genom_ok;
}


/* --- Function log_info ------------------------------------------------ */

/** Codel mk_log_info of function log_info.
 *
 * Returns genom_ok.
 */
genom_event
mk_log_info(const rotorcraft_log_s *log, uint32_t *miss,
            uint32_t *total, const genom_context self)
{
  (void)self; /* -Wunused-parameter */

  *miss = *total = 0;
  if (log) {
    *miss = log->missed;
    *total = log->total;
  }
  return genom_ok;
}
