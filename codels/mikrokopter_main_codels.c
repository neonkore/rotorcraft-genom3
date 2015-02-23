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
#include <math.h>

#include "mikrokopter_c_types.h"
#include "codels.h"


/* --- Task main -------------------------------------------------------- */


/** Codel mk_main_init of task main.
 *
 * Triggered by mikrokopter_start.
 * Yields to mikrokopter_main.
 */
genom_event
mk_main_init(mikrokopter_ids_sensor_rate_s *sensor_rate,
             mikrokopter_ids_battery_s *battery,
             mikrokopter_ids_attitude_s *attitude,
             mikrokopter_ids_imu_calibration_s *imu_calibration,
             sequence8_boolean *disabled_motors,
             const mikrokopter_rotors *rotors,
             const mikrokopter_imu *imu, genom_context self)
{
  sensor_rate->imu = 1000;
  sensor_rate->motor = 100;
  sensor_rate->battery = 1;

  battery->alarm = 13.7;
  battery->level = 0.;

  attitude->gyro_tau = 2;
  attitude->roll = attitude->pitch = attitude->yaw = 0.;

  imu_calibration->wx_off = 0.;
  imu_calibration->wy_off = 0.;
  imu_calibration->wz_off = 0.;

  disabled_motors->_length = 0;

  rotors->data(self)->_length = 0;

  or_pose_estimator_state *imu_data = imu->data(self);
  imu_data->ts.sec = 0;
  imu_data->ts.nsec = 0;
  imu_data->intrinsic = true;
  imu_data->pos._present = false;
  imu_data->pos_cov._present = false;
  imu_data->vel._present = false;
  imu_data->vel_cov._present = false;
  imu_data->acc._present = false;
  imu_data->acc_cov._present = false;

  return mikrokopter_main;
}


/** Codel mk_main_perm of task main.
 *
 * Triggered by mikrokopter_main.
 * Yields to mikrokopter_main.
 */
genom_event
mk_main_perm(const mikrokopter_conn_s *conn,
             const mikrokopter_ids_battery_s *battery,
             mikrokopter_ids_attitude_s *attitude,
             const mikrokopter_rotors *rotors,
             const mikrokopter_imu *imu, genom_context self)
{
  /* battery level */
  if (conn && conn->chan[0].fd >= 0) {
    if (battery->level > 0. && battery->level < battery->alarm) {
      static int cnt;

      if (!cnt) mk_send_msg(&conn->chan[0], "~%2", 440);
      cnt = (cnt + 1) % 500;
    }
  }

  /* attitude estimation */
  {
    or_pose_estimator_state *imu_data = imu->data(self);
    double dt = mikrokopter_control_period_ms / 1000.;
    double a = attitude->gyro_tau/(attitude->gyro_tau + dt);
    double aroll, apitch;

    double cr, cp, cy, sr, sp, sy;

    aroll = atan2(-imu_data->acc._value.ay,
                  hypot(imu_data->acc._value.ax, imu_data->acc._value.az));
    apitch = atan2(imu_data->acc._value.ax,
                   hypot(imu_data->acc._value.ay, imu_data->acc._value.az));

    attitude->roll =
      a * (attitude->roll + imu_data->vel._value.wx * dt) + (1.-a) * aroll;
    attitude->pitch =
      a * (attitude->pitch + imu_data->vel._value.wy * dt) + (1.-a) * apitch;
    attitude->yaw =
      attitude->yaw + imu_data->vel._value.wz * dt;

    cr = cos(attitude->roll/2);  sr = sin(attitude->roll/2);
    cp = cos(attitude->pitch/2); sp = sin(attitude->pitch/2);
    cy = cos(attitude->yaw/2);   sy = sin(attitude->yaw/2);

    imu_data->pos._value.x = nan("");
    imu_data->pos._value.y = nan("");
    imu_data->pos._value.z = nan("");
    imu_data->pos._value.qw = cr * cp * cy + sr * sp * sy;
    imu_data->pos._value.qx = sr * cp * cy - cr * sp * sy;
    imu_data->pos._value.qy = cr * sp * cy + sr * cp * sy;
    imu_data->pos._value.qz = cr * cp * sy - sr * sp * cy;
    imu_data->pos._present = true;
  }

  /* publish */
  rotors->write(self);
  imu->write(self);
  return mikrokopter_main;
}


/* --- Activity calibrate_imu ------------------------------------------- */

/** Codel mk_calibrate_imu_start of activity calibrate_imu.
 *
 * Triggered by mikrokopter_start.
 * Yields to mikrokopter_main.
 * Throws mikrokopter_e_connection.
 */
genom_event
mk_calibrate_imu_start(mikrokopter_ids_imu_calibration_s *accum,
                       uint32_t *t, genom_context self)
{
  accum->wx_off = accum->wy_off = accum->wz_off = 0.;
  *t = 0;
  return mikrokopter_main;
}

/** Codel mk_calibrate_imu of activity calibrate_imu.
 *
 * Triggered by mikrokopter_main.
 * Yields to mikrokopter_pause_main, mikrokopter_stop.
 * Throws mikrokopter_e_connection.
 */
genom_event
mk_calibrate_imu(double seconds, uint32_t *t,
                 const mikrokopter_imu *imu,
                 mikrokopter_ids_imu_calibration_s *accum,
                 genom_context self)
{
  or_pose_estimator_state *imu_data = imu->data(self);

  (*t)++;
  if (*t * mikrokopter_control_period_ms / 1000. > seconds)
    return mikrokopter_stop;

  accum->wx_off += imu_data->vel._value.wx;
  accum->wy_off += imu_data->vel._value.wy;
  accum->wz_off += imu_data->vel._value.wz;

  return mikrokopter_pause_main;
}

/** Codel mk_calibrate_imu_stop of activity calibrate_imu.
 *
 * Triggered by mikrokopter_stop.
 * Yields to mikrokopter_ether.
 * Throws mikrokopter_e_connection.
 */
genom_event
mk_calibrate_imu_stop(uint32_t t,
                      const mikrokopter_ids_imu_calibration_s *accum,
                      mikrokopter_ids_imu_calibration_s *imu_calibration,
                      genom_context self)
{
  imu_calibration->wx_off -= accum->wx_off / t;
  imu_calibration->wy_off -= accum->wy_off / t;
  imu_calibration->wz_off -= accum->wz_off / t;

  return mikrokopter_ether;
}


/* --- Activity start --------------------------------------------------- */

/** Codel mk_start_start of activity start.
 *
 * Triggered by mikrokopter_start.
 * Yields to mikrokopter_monitor.
 */
genom_event
mk_start_start(const mikrokopter_conn_s *conn, genom_context self)
{
  mk_send_msg(&conn->chan[0], "g");
  return mikrokopter_sleep;
}

/** Codel mk_start_monitor of activity start.
 *
 * Triggered by mikrokopter_monitor.
 * Yields to mikrokopter_pause_monitor, mikrokopter_ether.
 */
genom_event
mk_start_monitor(const mikrokopter_rotors *rotors,
                 const sequence8_boolean *disabled_motors,
                 genom_context self)
{
  mikrokopter_rotors_s *rdata = rotors->data(self);
  int i;

  if (rdata->_length == 0) return mikrokopter_sleep;
  for(i = 0; i < rdata->_length; i++) {
    if (i < disabled_motors->_length && disabled_motors->_buffer[i])
      continue;
    if (!rdata->_buffer[i].spinning) return mikrokopter_pause_monitor;
  }

  return mikrokopter_ether;
}

/** Codel mk_start_stop of activity start.
 *
 * Triggered by mikrokopter_stop.
 * Yields to mikrokopter_ether.
 */
genom_event
mk_start_stop(const mikrokopter_conn_s *conn, genom_context self)
{
  mk_send_msg(&conn->chan[0], "x");
  return mikrokopter_ether;
}


/* --- Activity servo --------------------------------------------------- */

/** Codel mk_servo_start of activity servo.
 *
 * Triggered by mikrokopter_start.
 * Yields to mikrokopter_step.
 * Throws mikrokopter_e_connection, mikrokopter_e_hardware,
 *        mikrokopter_e_input.
 */
genom_event
mk_servo_start(const mikrokopter_conn_s *conn,
               const mikrokopter_cmd_wrench *cmd_wrench,
               const mikrokopter_rotors *rotors,
               const sequence8_boolean *disabled_motors,
               genom_context self)
{
  mikrokopter_rotors_s *rotor_data = rotors->data(self);
  or_rotorcraft_ts_wrench *cmd_data = cmd_wrench->data(self);
  int i;

  if (!conn) return mikrokopter_e_connection(self);
  if (!rotor_data) return mikrokopter_e_hardware(self);
  for(i = 0; i < rotor_data->_length; i++) {
    if (i < disabled_motors->_length && disabled_motors->_buffer[i])
      continue;
    if (!rotor_data->_buffer[i].spinning) return mikrokopter_e_hardware(self);
  }
  if (!cmd_data) return mikrokopter_e_input(self);

  return mikrokopter_step;
}

/** Codel mk_servo_step of activity servo.
 *
 * Triggered by mikrokopter_step.
 * Yields to mikrokopter_pause_step.
 * Throws mikrokopter_e_connection, mikrokopter_e_hardware,
 *        mikrokopter_e_input.
 */
genom_event
mk_servo_step(const mikrokopter_conn_s *conn,
              const mikrokopter_ids_servo_s *servo,
              const mikrokopter_cmd_wrench *cmd_wrench,
              const mikrokopter_rotors *rotors,
              const sequence8_boolean *disabled_motors,
              genom_context self)
{
  mikrokopter_rotors_s *rotor_data = rotors->data(self);
  or_rotorcraft_ts_wrench *cmd_data = cmd_wrench->data(self);

  or_rb3d_force force;
  or_rb3d_torque torque;

  double tthrust, tlimit;
  double f[4], v;
  uint16_t p[4];
  int i;

  double d = 0.23;
  double c = 0.0154;
  double kf = 6.5e-4;

  /* check rotors status */
  for(i = 0; i < rotor_data->_length; i++) {
    if (i < disabled_motors->_length && disabled_motors->_buffer[i])
      continue;

    if (!rotor_data->_buffer[i].spinning) {
      mk_send_msg(&conn->chan[0], "x");
      return mikrokopter_e_hardware(self);
    }
  }

  /* update input */
  cmd_wrench->read(self);
  force.x = force.y = 0.;
  force.z = cmd_data->w.f.z;
  torque = cmd_data->w.t;

  /* total thrust */
  tthrust = fabs(force.z);

  /* torque limitation */
  tlimit = servo->vmax * servo->vmax * kf - tthrust/4;
  if (tthrust/4 - servo->vmin * servo->vmin * kf < tlimit)
    tlimit = tthrust/4 - servo->vmin * servo->vmin * kf;
  if (tlimit < 0.) tlimit = 0.;
  tlimit = d * tlimit;

  if (fabs(torque.x) > tlimit) torque.x = copysign(tlimit, torque.x);
  if (fabs(torque.y) > tlimit) torque.y = copysign(tlimit, torque.y);
  if (fabs(torque.z) > 0.2) torque.z = copysign(0.2, torque.z);

  /* forces */
  f[0] = tthrust/4 - torque.y/d/2. + torque.z/c/4;
  f[1] = tthrust/4 + torque.y/d/2. + torque.z/c/4;
  f[2] = tthrust/4 - torque.x/d/2. - torque.z/c/4;
  f[3] = tthrust/4 + torque.x/d/2. - torque.z/c/4;

  /* velocities */
  for(i = 0; i < 4; i++) {
    v = sqrt(f[i]/kf);
    p[i] = (1000000/v > 65535) ? 65535 : 1000000/v;

    rotor_data->_buffer[i].target = 1000000./p[i];
  }

  /* send */
  mk_send_msg(&conn->chan[0], "w%2%2%2%2", p[0], p[1], p[2], p[3]);

  return mikrokopter_pause_step;
}

/** Codel mk_servo_stop of activity servo.
 *
 * Triggered by mikrokopter_stop.
 * Yields to mikrokopter_ether.
 * Throws mikrokopter_e_connection, mikrokopter_e_hardware,
 *        mikrokopter_e_input.
 */
genom_event
mk_servo_stop(const mikrokopter_conn_s *conn, genom_context self)
{
  mk_send_msg(&conn->chan[0], "x");
  return mikrokopter_ether;
}
