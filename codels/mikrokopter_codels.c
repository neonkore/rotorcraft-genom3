/*
 * Copyright (c) 2015-2016 LAAS/CNRS
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
                   genom_context self)
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


/* --- Attribute set_imu_calibration ------------------------------------ */

/** Validation codel mk_set_imu_calibration of attribute set_imu_calibration.
 *
 * Returns genom_ok.
 * Throws .
 */
genom_event
mk_set_imu_calibration(bool *imu_calibration_updated,
                       genom_context self)
{
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
                  genom_context self)
{
  mk_imu_iirf_init(1000. / mikrokopter_control_period_ms,
                   imu_filter->gain, imu_filter->Q, 15, 143);
  return genom_ok;
}


/* --- Function disable_motor ------------------------------------------- */

/** Codel mk_disable_motor of function disable_motor.
 *
 * Returns genom_ok.
 */
genom_event
mk_disable_motor(uint16_t motor, const mikrokopter_conn_s *conn,
                 sequence8_mikrokopter_rotor_state_s *rotors_state,
                 genom_context self)
{
  uint8_t id;
  if (motor < 1 || motor > rotors_state->_maximum)
    return mikrokopter_e_range(self);

  if (motor > rotors_state->_length) rotors_state->_length = motor;
  id = motor - 1;
  rotors_state->_buffer[id].emerg = false;
  rotors_state->_buffer[id].spinning = false;
  rotors_state->_buffer[id].starting = false;
  rotors_state->_buffer[id].disabled = true;
  if (conn) mk_send_msg(&conn->chan[0], "x%1", id+1);

  return genom_ok;
}


/* --- Function enable_motor -------------------------------------------- */

/** Codel mk_enable_motor of function enable_motor.
 *
 * Returns genom_ok.
 */
genom_event
mk_enable_motor(uint16_t motor, const mikrokopter_conn_s *conn,
                sequence8_mikrokopter_rotor_state_s *rotors_state,
                genom_context self)
{
  uint8_t id;
  if (motor < 1 || motor > rotors_state->_maximum)
    return mikrokopter_e_range(self);

  if (motor > rotors_state->_length) rotors_state->_length = motor;
  id = motor - 1;
  rotors_state->_buffer[id].emerg = false;
  rotors_state->_buffer[id].spinning = false;
  rotors_state->_buffer[id].starting = false;
  rotors_state->_buffer[id].disabled = false;

  if (conn) {
    int i;
    for(i = 0; i < rotors_state->_length; i++) {
      if (i < rotors_state->_length && rotors_state->_buffer[i].disabled)
        continue;
      if (!rotors_state->_buffer[i].spinning) continue;

      mk_send_msg(&conn->chan[0], "g%1", id+1);
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
                const sequence8_mikrokopter_rotor_state_s *rotors_state,
                double rotors_wd[8],
                const or_rotorcraft_propeller_velocity *w,
                genom_context self)
{
  mikrokopter_e_rotor_failure_detail e;
  int16_t p[or_rotorcraft_max_rotors];
  size_t l;
  int i;

  /* check rotors status */
  for(i = 0; i < rotors_state->_length; i++) {
    if (i < rotors_state->_length && rotors_state->_buffer[i].disabled)
      continue;

    if (!rotors_state->_buffer[i].spinning) {
      e.id = 1 + i;
      return mikrokopter_e_rotor_failure(&e, self);
    }
  }

  /* discard trailing nans */
  l = w->_length;
  while(isnan(w->_buffer[l-1])) l--;

  /* rotational period */
  for(i = 0; i < l; i++) {
    if (i < rotors_state->_length && rotors_state->_buffer[i].disabled)
      rotors_wd[i] = 0.;
    else
      rotors_wd[i] = w->_buffer[i];

    p[i] = (fabs(rotors_wd[i]) < 1000000./65535.) ?
      copysign(32767, rotors_wd[i]) : 1000000/2/rotors_wd[i];
  }

  /* send */
  mk_send_msg(&conn->chan[0], "w%@", p, l);
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
             genom_context self)
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
mk_log_stop(mikrokopter_log_s **log, genom_context self)
{
  if (!*log) return genom_ok;

  fclose((*log)->logf);
  free(*log);
  *log = NULL;
  return genom_ok;
}
