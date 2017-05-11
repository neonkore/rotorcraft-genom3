/*
 * Copyright (c) 2015-2017 LAAS/CNRS
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
#include <float.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>

#include "mikrokopter_c_types.h"
#include "codels.h"


/* --- Attribute set_sensor_rate ---------------------------------------- */

/** Validation codel mk_set_sensor_rate of attribute set_sensor_rate.
 *
 * Returns genom_ok.
 * Throws .
 */
genom_event
mk_set_sensor_rate(const mikrokopter_ids_sensor_time_s_rate_s *rate,
                   const mikrokopter_conn_s *conn,
                   mikrokopter_ids_sensor_time_s *sensor_time,
                   const genom_context self)
{
  int mainc, auxc;
  uint32_t p;
  int i;

  if (rate->imu < 0. || rate->imu > 2000. ||
      rate->motor < 0. || rate->motor > 2000. ||
      rate->battery < 0. || rate->battery > 2000.)
    return mikrokopter_e_range(self);

  if (sensor_time) {
    sensor_time->imu.seq = 0;
    sensor_time->imu.ts = 0.;
    sensor_time->imu.offset = -DBL_MAX;
    for(i = 0; i < or_rotorcraft_max_rotors; i++) {
      sensor_time->motor[i].seq = 0;
      sensor_time->motor[i].ts = 0.;
      sensor_time->motor[i].offset = -DBL_MAX;
    }
    sensor_time->battery.seq = 0;
    sensor_time->battery.ts = 0.;
    sensor_time->battery.offset = -DBL_MAX;
  }

  /* reconfigure existing connection */
  if (!conn) return genom_ok;
  mainc = auxc = -1;
  if (conn->chan[0].fd >= 0) { mainc = 0; auxc = 0; }
  if (conn->chan[1].fd >= 0) auxc = 1;
  if (mainc < 0 && auxc < 0) return genom_ok;

  p = rate->battery > 0. ? 1000000/rate->battery : 0;
  mk_send_msg(&conn->chan[auxc], "b%4", p);
  p = rate->motor > 0. ? 1000000/rate->motor : 0;
  mk_send_msg(&conn->chan[auxc], "m%4", p);
  p = rate->motor > 0. ? 1000000/rate->imu : 0;
  mk_send_msg(&conn->chan[mainc], "i%4", p);

  return genom_ok;
}


/* --- Attribute set_battery_limits ------------------------------------- */

/** Validation codel mk_set_battery_limits of attribute set_battery_limits.
 *
 * Returns genom_ok.
 * Throws mikrokopter_e_range.
 */
genom_event
mk_set_battery_limits(double min, double max,
                      const genom_context self)
{
  if (min < 0.) return mikrokopter_e_range(self);
  if (min >= max - 1e-2) return mikrokopter_e_range(self);
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


/* --- Attribute set_imu_filter ----------------------------------------- */

/** Validation codel mk_set_imu_filter of attribute set_imu_filter.
 *
 * Returns genom_ok.
 * Throws .
 */
genom_event
mk_set_imu_filter(const mikrokopter_ids_imu_filter_s *imu_filter,
                  const genom_context self)
{
  (void)self;

  mk_imu_iirf_init(1000. / mikrokopter_control_period_ms,
                   imu_filter->gain, imu_filter->Q, 15, 143);
  return genom_ok;
}


/* --- Function set_velocity -------------------------------------------- */

/** Validation codel mk_validate_input of function set_velocity.
 *
 * Returns genom_ok.
 * Throws mikrokopter_e_connection, mikrokopter_e_rotor_failure.
 */
genom_event
mk_validate_input(const or_rotorcraft_rotor_state rotor_state[8],
                  or_rotorcraft_rotor_control *desired,
                  const genom_context self)
{
  mikrokopter_e_rotor_failure_detail e;
  size_t i, l;

  /* check rotors status */
  for(i = 0; i < or_rotorcraft_max_rotors; i++) {
    if (rotor_state[i].disabled) continue;
    if (rotor_state[i].emerg) {
      e.id = 1 + i;
      return mikrokopter_e_rotor_failure(&e, self);
    }
  }

  /* discard trailing nans */
  l = desired->_length;
  while(l && isnan(desired->_buffer[l-1])) l--;
  return genom_ok;
}


/* --- Function set_throttle -------------------------------------------- */

/** Validation codel mk_validate_input of function set_throttle.
 *
 * Returns genom_ok.
 * Throws mikrokopter_e_connection, mikrokopter_e_rotor_failure.
 */
/* already defined in service set_velocity validation */



/* --- Function disable_motor ------------------------------------------- */

/** Codel mk_disable_motor of function disable_motor.
 *
 * Returns genom_ok.
 */
genom_event
mk_disable_motor(uint16_t motor, const mikrokopter_conn_s *conn,
                 or_rotorcraft_rotor_state rotor_state[8],
                 const genom_context self)
{
  struct timeval tv;

  if (motor < 1 || motor > or_rotorcraft_max_rotors)
    return mikrokopter_e_range(self);

  gettimeofday(&tv, NULL);
  rotor_state[motor - 1] = (or_rotorcraft_rotor_state){
    .ts = { .sec = tv.tv_sec, .nsec = tv.tv_usec * 1000 },
    .emerg = false, .spinning = false, .starting = false, .disabled = true,
    .velocity = nan(""), .throttle = nan(""), .consumption = nan(""),
    .energy_level = nan("")
  };

  if (conn) mk_send_msg(&conn->chan[0], "x%1", (uint8_t){motor});

  return genom_ok;
}


/* --- Function enable_motor -------------------------------------------- */

/** Codel mk_enable_motor of function enable_motor.
 *
 * Returns genom_ok.
 */
genom_event
mk_enable_motor(uint16_t motor, const mikrokopter_conn_s *conn,
                or_rotorcraft_rotor_state rotor_state[8],
                const genom_context self)
{
  struct timeval tv;

  if (motor < 1 || motor > or_rotorcraft_max_rotors)
    return mikrokopter_e_range(self);

  gettimeofday(&tv, NULL);
  rotor_state[motor - 1] = (or_rotorcraft_rotor_state){
    .ts = { .sec = tv.tv_sec, .nsec = tv.tv_usec * 1000 },
    .emerg = false, .spinning = false, .starting = false, .disabled = false
  };

  if (conn) {
    size_t i;
    for(i = 0; i < or_rotorcraft_max_rotors; i++) {
      if (rotor_state[i].disabled) continue;
      if (!rotor_state[i].spinning) continue;

      mk_send_msg(&conn->chan[0], "g%1", (uint8_t){motor});
      break;
    }
  }

  return genom_ok;
}


/* --- Function set_velocity -------------------------------------------- */

/** Codel mk_set_velocity of function set_velocity.
 *
 * Returns genom_ok.
 * Throws mikrokopter_e_connection, mikrokopter_e_rotor_failure.
 */
genom_event
mk_set_velocity(const mikrokopter_conn_s *conn,
                const or_rotorcraft_rotor_state rotor_state[8],
                double rotor_wd[8],
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
    rotor_wd[i] = rotor_state[i].disabled ? 0. : desired->_buffer[i];

    p[i] = (fabs(rotor_wd[i]) < 1000000./65535.) ?
      copysign(32767, rotor_wd[i]) : 1000000/2/rotor_wd[i];
  }

  /* send */
  mk_send_msg(&conn->chan[0], "w%@", p, l);
  return genom_ok;
}


/* --- Function set_throttle -------------------------------------------- */

/** Codel mk_set_throttle of function set_throttle.
 *
 * Returns genom_ok.
 * Throws mikrokopter_e_connection, mikrokopter_e_rotor_failure.
 */
genom_event
mk_set_throttle(const mikrokopter_conn_s *conn,
                const or_rotorcraft_rotor_state rotor_state[8],
                double rotor_wd[8],
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
    rotor_wd[i] = 0.;
    p[i] = rotor_state[i].disabled ? 0. : desired->_buffer[i] * 1023./100.;
  }

  /* send */
  mk_send_msg(&conn->chan[0], "q%@", p, l);
  return genom_ok;
}


/* --- Function log ----------------------------------------------------- */

/** Codel mk_log_start of function log.
 *
 * Returns genom_ok.
 * Throws mikrokopter_e_sys.
 */
genom_event
mk_log_start(const char path[64], mikrokopter_log_s **log,
             const genom_context self)
{
  FILE *f;

  mk_log_stop(log, self);

  f = fopen(path, "w");
  if (!f) return mk_e_sys_error("log", self);
  fprintf(f, mikrokopter_log_header "\n");

  *log = malloc(sizeof(**log));
  if (!*log) {
    fclose(f);
    unlink(path);
    errno = ENOMEM;
    return mk_e_sys_error("log", self);
  }

  (*log)->logf = f;
  return genom_ok;
}


/* --- Function log_stop ------------------------------------------------ */

/** Codel mk_log_stop of function log_stop.
 *
 * Returns genom_ok.
 */
genom_event
mk_log_stop(mikrokopter_log_s **log, const genom_context self)
{
  (void)self;

  if (!*log) return genom_ok;

  fclose((*log)->logf);
  free(*log);
  *log = NULL;
  return genom_ok;
}
