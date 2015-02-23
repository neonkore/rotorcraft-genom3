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

#include "mikrokopter_c_types.h"
#include "codels.h"


/* --- Attribute set_sensor_rate ---------------------------------------- */

/** Validation codel mk_set_sensor_rate of attribute set_sensor_rate.
 *
 * Returns genom_ok.
 * Throws .
 */
genom_event
mk_set_sensor_rate(const mikrokopter_ids_sensor_rate_s *sensor_rate,
                   const mikrokopter_conn_s *conn, genom_context self)
{
  int mainc, auxc;
  uint32_t p;

  if (!conn) return mikrokopter_e_connection(self);
  if (sensor_rate->imu < 0. || sensor_rate->imu > 1000. ||
      sensor_rate->motor < 0. || sensor_rate->motor > 1000. ||
      sensor_rate->battery < 0. || sensor_rate->battery > 1000.)
    return mikrokopter_e_range(self);

  /* reconfigure existing connection */
  mainc = auxc = -1;
  if (conn->chan[0].fd >= 0) { mainc = 0; auxc = 0; }
  if (conn->chan[1].fd >= 0) auxc = 1;
  if (mainc < 0 && auxc < 0) return genom_ok;

  p = sensor_rate->battery > 0. ? 1000000/sensor_rate->battery : 0;
  mk_send_msg(&conn->chan[auxc], "b%4", p);
  p = sensor_rate->motor > 0. ? 1000000/sensor_rate->motor : 0;
  mk_send_msg(&conn->chan[auxc], "m%4", p);
  p = sensor_rate->motor > 0. ? 1000000/sensor_rate->imu : 0;
  mk_send_msg(&conn->chan[mainc], "i%4", p);

  return genom_ok;
}


/* --- Function disable_motor ------------------------------------------- */

/** Codel mk_disable_motor of function disable_motor.
 *
 * Returns genom_ok.
 */
genom_event
mk_disable_motor(uint16_t motor, sequence8_boolean *disabled_motors,
                 genom_context self)
{
  if (motor < 1 || motor > disabled_motors->_maximum)
    return mikrokopter_e_range(self);

  if (motor > disabled_motors->_length) disabled_motors->_length = motor;
  disabled_motors->_buffer[motor-1] = true;
  return genom_ok;
}


/* --- Function enable_motor -------------------------------------------- */

/** Codel mk_enable_motor of function enable_motor.
 *
 * Returns genom_ok.
 */
genom_event
mk_enable_motor(uint16_t motor, sequence8_boolean *disabled_motors,
                genom_context self)
{
  if (motor < 1 || motor > disabled_motors->_maximum)
    return mikrokopter_e_range(self);

  if (motor > disabled_motors->_length) disabled_motors->_length = motor;
  disabled_motors->_buffer[motor-1] = false;
  return genom_ok;
}


/* --- Function stop ---------------------------------------------------- */

/** Codel mk_stop of function stop.
 *
 * Returns genom_ok.
 */
genom_event
mk_stop(const mikrokopter_conn_s *conn, genom_context self)
{
  if (!conn) return mikrokopter_e_connection(self);
  mk_send_msg(&conn->chan[0], "x");
  return genom_ok;
}
