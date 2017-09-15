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
#include <err.h>
#include <float.h>
#include <math.h>

#include "mikrokopter_c_types.h"
#include "codels.h"


/* --- Task main -------------------------------------------------------- */

static struct mk_iir_filter Hax, Hay, Haz, Hwx, Hwy, Hwz;


/** Codel mk_main_init of task main.
 *
 * Triggered by mikrokopter_start.
 * Yields to mikrokopter_main.
 */
genom_event
mk_main_init(mikrokopter_ids *ids, const mikrokopter_imu *imu,
             const genom_context self)
{
  genom_event e;
  size_t i;

  ids->sensor_time.rate.imu = 1000;
  ids->sensor_time.rate.motor = 100;
  ids->sensor_time.rate.battery = 1;
  e = mk_set_sensor_rate(
    &ids->sensor_time.rate, NULL, &ids->sensor_time, self);
  if (e) return e;

  ids->battery.min = 14.0;
  ids->battery.max = 16.8;
  ids->battery.level = 0.;

  ids->imu_calibration = (mikrokopter_ids_imu_calibration_s){
    .gscale = {
      1., 0., 0.,
      0., 1., 0.,
      0., 0., 1.
    },
    .gbias = { 0., 0., 0. },
    .gstddev = { 1e-2, 1e-2, 1e-2 },

    .ascale = {
      1., 0., 0.,
      0., 1., 0.,
      0., 0., 1.
    },
    .abias = { 0., 0., 0. },
    .astddev = { 5e-2, 5e-2, 5e-2 },
  };
  ids->imu_calibration_updated = true;

  ids->imu_filter.enable = false;
  ids->imu_filter.gain = 0.05;
  ids->imu_filter.Q = 5;
  Hax = Hay = Haz = Hwx = Hwy = Hwz = MK_IIRF_INIT(nan(""));

  for(i = 0; i < or_rotorcraft_max_rotors; i++) {
    ids->rotor_data.state[i] = (or_rotorcraft_rotor_state){
      .ts = { 0, 0 },
      .emerg = false, .spinning = false, .starting = false, .disabled = true,
      .velocity = nan(""), .throttle = nan(""), .consumption = nan(""),
      .energy_level = nan("")
    };
    ids->rotor_data.wd[i] = 0.;
    ids->rotor_data.clkrate[i] = 0;
  }

  ids->servo.ramp = 3.;

  *imu->data(self) = (or_pose_estimator_state){
    .ts = { 0, 0 },
    .intrinsic = true,
    .pos._present = false,
    .pos_cov._present = false,
    .vel._present = false,
    .vel_cov._present = false,
    .acc._present = false,
    .acc_cov._present = false
  };

  return mikrokopter_main;
}


/** Codel mk_main_perm of task main.
 *
 * Triggered by mikrokopter_main.
 * Yields to mikrokopter_pause_main.
 */
genom_event
mk_main_perm(const mikrokopter_conn_s *conn,
             const mikrokopter_ids_battery_s *battery,
             const mikrokopter_ids_imu_calibration_s *imu_calibration,
             const mikrokopter_ids_imu_filter_s *imu_filter,
             const mikrokopter_ids_rotor_data_s *rotor_data,
             bool *imu_calibration_updated,
             const mikrokopter_log_s *log,
             const mikrokopter_rotor_measure *rotor_measure,
             const mikrokopter_imu *imu, const genom_context self)
{
  or_pose_estimator_state *idata = imu->data(self);
  ssize_t i;

  /* battery level */
  if (conn && conn->chan[0].fd >= 0) {
    if (battery->level > 0. && battery->level < battery->min) {
      static int cnt;

      if (!cnt) mk_send_msg(&conn->chan[0], "~%2", 440);
      cnt = (cnt + 1) % 500;
    }
  }

  /* imu covariance data */
  if (*imu_calibration_updated) {
    idata->vel_cov._value.cov[0] = 0.;
    idata->vel_cov._value.cov[1] = 0.;
    idata->vel_cov._value.cov[2] = 0.;
    idata->vel_cov._value.cov[3] = 0.;
    idata->vel_cov._value.cov[4] = 0.;
    idata->vel_cov._value.cov[5] = 0.;
    idata->vel_cov._value.cov[6] = 0.;
    idata->vel_cov._value.cov[7] = 0.;
    idata->vel_cov._value.cov[8] = 0.;
    idata->vel_cov._value.cov[9] =
      imu_calibration->gstddev[0] * imu_calibration->gstddev[0];
    idata->vel_cov._value.cov[10] = 0.;
    idata->vel_cov._value.cov[11] = 0.;
    idata->vel_cov._value.cov[12] = 0.;
    idata->vel_cov._value.cov[13] = 0.;
    idata->vel_cov._value.cov[14] =
      imu_calibration->gstddev[1] * imu_calibration->gstddev[1];
    idata->vel_cov._value.cov[15] = 0.;
    idata->vel_cov._value.cov[16] = 0.;
    idata->vel_cov._value.cov[17] = 0.;
    idata->vel_cov._value.cov[18] = 0.;
    idata->vel_cov._value.cov[19] = 0.;
    idata->vel_cov._value.cov[20] =
      imu_calibration->gstddev[2] * imu_calibration->gstddev[2];
    idata->vel_cov._present = true;

    idata->acc_cov._value.cov[0] =
      imu_calibration->astddev[0] * imu_calibration->astddev[0];
    idata->acc_cov._value.cov[1] = 0.;
    idata->acc_cov._value.cov[2] =
      imu_calibration->astddev[1] * imu_calibration->astddev[1];
    idata->acc_cov._value.cov[3] = 0.;
    idata->acc_cov._value.cov[4] = 0.;
    idata->acc_cov._value.cov[5] =
      imu_calibration->astddev[2] * imu_calibration->astddev[2];
    idata->acc_cov._present = true;

    *imu_calibration_updated = false;
  }

  /* filter IMU */
  if (imu_filter->enable) {
    int c;
    double favg = 0.;

    for(i = c = 0; i < or_rotorcraft_max_rotors; i++) {
      if (rotor_data->wd[i] > 15.) { c++; favg += rotor_data->wd[i]; }
    }

    if (c) {
      favg /= c;

      idata->acc._value.ax = mk_imu_iirf(idata->acc._value.ax, &Hax, favg);
      idata->acc._value.ay = mk_imu_iirf(idata->acc._value.ay, &Hay, favg);
      idata->acc._value.az = mk_imu_iirf(idata->acc._value.az, &Haz, favg);

      idata->vel._value.wx = mk_imu_iirf(idata->vel._value.wx, &Hwx, favg);
      idata->vel._value.wy = mk_imu_iirf(idata->vel._value.wy, &Hwy, favg);
      idata->vel._value.wz = mk_imu_iirf(idata->vel._value.wz, &Hwz, favg);
    }
  }


  /* publish */
  for(i = 0; i < or_rotorcraft_max_rotors; i++) {
    rotor_measure->data(self)->rotor._buffer[i] = rotor_data->state[i];
    if (!rotor_data->state[i].disabled)
      rotor_measure->data(self)->rotor._length = i + 1;
  }
  rotor_measure->write(self);
  imu->write(self);


  /* log */
  if (log) {
    or_pose_estimator_state *idata = imu->data(self);
    or_rotorcraft_rotor_state *rdata =
      rotor_measure->data(self)->rotor._buffer;
    struct timeval tv;

    gettimeofday(&tv, NULL);
    fprintf(
      log->logf, mikrokopter_log_line "\n",
      (uint64_t)tv.tv_sec, (uint32_t)tv.tv_usec * 1000,
      idata->ts.sec, idata->ts.nsec,
      idata->vel._value.wx, idata->vel._value.wy, idata->vel._value.wz,
      idata->acc._value.ax, idata->acc._value.ay, idata->acc._value.az,

      rotor_data->wd[0], rotor_data->wd[1], rotor_data->wd[2],
      rotor_data->wd[3], rotor_data->wd[4], rotor_data->wd[5],
      rotor_data->wd[6], rotor_data->wd[7],

      rdata[0].velocity, rdata[1].velocity, rdata[2].velocity,
      rdata[3].velocity, rdata[4].velocity, rdata[5].velocity,
      rdata[6].velocity, rdata[7].velocity,

      rotor_data->clkrate[0], rotor_data->clkrate[1], rotor_data->clkrate[2],
      rotor_data->clkrate[3], rotor_data->clkrate[4], rotor_data->clkrate[5],
      rotor_data->clkrate[6], rotor_data->clkrate[7]);
  }

  return mikrokopter_pause_main;
}


/** Codel mk_main_stop of task main.
 *
 * Triggered by mikrokopter_stop.
 * Yields to mikrokopter_ether.
 */
genom_event
mk_main_stop(mikrokopter_log_s **log, const genom_context self)
{
  mk_log_stop(log, self);
  return mikrokopter_ether;
}


/* --- Activity calibrate_imu ------------------------------------------- */

/** Codel mk_calibrate_imu_start of activity calibrate_imu.
 *
 * Triggered by mikrokopter_start.
 * Yields to mikrokopter_collect.
 * Throws mikrokopter_e_sys, mikrokopter_e_connection.
 */
genom_event
mk_calibrate_imu_start(double tstill, uint16_t nposes,
                       const genom_context self)
{
  uint32_t sps;
  int s;

  sps = 1000/mikrokopter_control_period_ms;
  s = mk_calibration_init(tstill * sps, nposes, sps);
  if (s) {
    errno = s;
    return mk_e_sys_error("calibration", self);
  }

  warnx("calibration started");
  return mikrokopter_collect;
}

/** Codel mk_calibrate_imu_collect of activity calibrate_imu.
 *
 * Triggered by mikrokopter_collect.
 * Yields to mikrokopter_pause_collect, mikrokopter_main.
 * Throws mikrokopter_e_sys, mikrokopter_e_connection.
 */
genom_event
mk_calibrate_imu_collect(const mikrokopter_imu *imu,
                         const genom_context self)
{
  int32_t still;
  int s;

  s = mk_calibration_collect(imu->data(self), &still);
  switch(s) {
    case 0: break;

    case EAGAIN:
      if (still == 0)
        warnx("stay still");
      else if (still > 0)
        warnx("calibration acquired pose %d", still);
      return mikrokopter_pause_collect;

    default:
      warnx("calibration aborted");
      mk_calibration_fini(NULL, NULL, NULL, NULL);
      errno = s;
      return mk_e_sys_error("calibration", self);
  }

  warnx("calibration acquired all poses");
  return mikrokopter_main;
}

/** Codel mk_calibrate_imu_main of activity calibrate_imu.
 *
 * Triggered by mikrokopter_main.
 * Yields to mikrokopter_ether.
 * Throws mikrokopter_e_sys, mikrokopter_e_connection.
 */
genom_event
mk_calibrate_imu_main(mikrokopter_ids_imu_calibration_s *imu_calibration,
                      bool *imu_calibration_updated,
                      const genom_context self)
{
  double maxa[3], maxw[3];
  int s;

  s = mk_calibration_acc(imu_calibration->ascale, imu_calibration->abias);
  if (s) {
    mk_calibration_fini(NULL, NULL, NULL, NULL);
    errno = s;
    return mk_e_sys_error("calibration", self);
  }

  s = mk_calibration_gyr(imu_calibration->gscale, imu_calibration->gbias);
  if (s) {
    mk_calibration_fini(NULL, NULL, NULL, NULL);
    errno = s;
    return mk_e_sys_error("calibration", self);
  }

  mk_calibration_fini(imu_calibration->astddev, imu_calibration->gstddev,
                      maxa, maxw);
  warnx("calibration max acceleration: "
        "x %.2fm/s², y %.2fm/s², z %.2fm/s²", maxa[0], maxa[1], maxa[2]);
  warnx("calibration max angular velocity: "
        "x %.2f⁰/s, y %.2f⁰/s, z %.2f⁰/s",
        maxw[0] * 180./M_PI, maxw[1] * 180./M_PI, maxw[2] * 180./M_PI);

  *imu_calibration_updated = true;
  return mikrokopter_ether;
}


/* --- Activity set_zero ------------------------------------------------ */

/** Codel mk_set_zero_start of activity set_zero.
 *
 * Triggered by mikrokopter_start.
 * Yields to mikrokopter_collect.
 * Throws mikrokopter_e_sys.
 */
genom_event
mk_set_zero_start(double accum[3], double gycum[3], uint32_t *n,
                  const genom_context self)
{
  (void)self;

  gycum[0] = gycum[1] = gycum[2] = 0.;
  accum[0] = accum[1] = accum[2] = 0.;
  *n = 0;
  return mikrokopter_collect;
}

/** Codel mk_set_zero_collect of activity set_zero.
 *
 * Triggered by mikrokopter_collect.
 * Yields to mikrokopter_pause_collect, mikrokopter_main.
 * Throws mikrokopter_e_sys.
 */
genom_event
mk_set_zero_collect(const mikrokopter_imu *imu, double accum[3],
                    double gycum[3], uint32_t *n,
                    const genom_context self)
{
  or_pose_estimator_state *imu_data = imu->data(self);

  if (!imu_data->vel._present || !imu_data->acc._present) {
    errno = EIO;
    return mk_e_sys_error("set_zero", self);
  }

  gycum[0] = (*n * gycum[0] - imu_data->vel._value.wx) / (1 + *n);
  gycum[1] = (*n * gycum[1] - imu_data->vel._value.wy) / (1 + *n);
  gycum[2] = (*n * gycum[2] - imu_data->vel._value.wz) / (1 + *n);

  accum[0] = (*n * accum[0] + imu_data->acc._value.ax) / (1 + *n);
  accum[1] = (*n * accum[1] + imu_data->acc._value.ay) / (1 + *n);
  accum[2] = (*n * accum[2] + imu_data->acc._value.az) / (1 + *n);

  return ((*n)++ < 2000.) ? mikrokopter_pause_collect : mikrokopter_main;
}

/** Codel mk_set_zero of activity set_zero.
 *
 * Triggered by mikrokopter_main.
 * Yields to mikrokopter_ether.
 * Throws mikrokopter_e_sys.
 */
genom_event
mk_set_zero(double accum[3], double gycum[3],
            mikrokopter_ids_imu_calibration_s *imu_calibration,
            bool *imu_calibration_updated, const genom_context self)
{
  double roll, pitch;
  double cr, cp, sr, sp;
  double r[9];
  (void)self;

  roll = atan2(accum[1], accum[2]);
  cr = cos(roll);  sr = sin(roll);
  pitch = atan2(-accum[0] * cr, accum[2]);
  cp = cos(pitch); sp = sin(pitch);

  r[0] = cp;   r[1] = sr * sp;  r[2] = cr * sp;
  r[3] = 0.;   r[4] = cr;       r[5] = -sr;
  r[6] = -sp;  r[7] = cp * sr;  r[8] = cr * cp;

  mk_calibration_bias(gycum, imu_calibration->gscale, imu_calibration->gbias);
  mk_calibration_rotate(r, imu_calibration->gscale);
  mk_calibration_rotate(r, imu_calibration->ascale);

  *imu_calibration_updated = true;
  return mikrokopter_ether;
}


/* --- Activity start --------------------------------------------------- */

/** Codel mk_start_start of activity start.
 *
 * Triggered by mikrokopter_start.
 * Yields to mikrokopter_monitor.
 * Throws mikrokopter_e_connection, mikrokopter_e_started,
 *        mikrokopter_e_rotor_failure,
 *        mikrokopter_e_rotor_not_disabled.
 */
genom_event
mk_start_start(const mikrokopter_conn_s *conn, uint16_t *state,
               const or_rotorcraft_rotor_state rotor_state[8],
               const genom_context self)
{
  size_t i;

  if (!conn) return mikrokopter_e_connection(self);
  for(i = 0; i < or_rotorcraft_max_rotors; i++) {
    if (rotor_state[i].disabled) continue;
    if (rotor_state[i].spinning) return mikrokopter_e_started(self);
  }

  *state = 0;
  for(i = 0; i < or_rotorcraft_max_rotors; i++) {
    if (rotor_state[i].disabled) continue;
    mk_send_msg(&conn->chan[0], "g%1", (uint8_t){i+1});
  }
  return mikrokopter_monitor;
}

/** Codel mk_start_monitor of activity start.
 *
 * Triggered by mikrokopter_monitor.
 * Yields to mikrokopter_pause_monitor, mikrokopter_ether.
 * Throws mikrokopter_e_connection, mikrokopter_e_started,
 *        mikrokopter_e_rotor_failure,
 *        mikrokopter_e_rotor_not_disabled.
 */
genom_event
mk_start_monitor(const mikrokopter_conn_s *conn, uint16_t *state,
                 const or_rotorcraft_rotor_state rotor_state[8],
                 const genom_context self)
{
  mikrokopter_e_rotor_failure_detail e;
  mikrokopter_e_rotor_not_disabled_detail d;
  size_t i;

  for(i = 0; i < or_rotorcraft_max_rotors; i++) {
    if (rotor_state[i].disabled) {
      if (rotor_state[i].spinning) {
        mk_send_msg(&conn->chan[0], "x");
        d.id = 1 + i;
        return mikrokopter_e_rotor_not_disabled(&d, self);
      }

      if (rotor_state[i].starting)
        return mikrokopter_pause_monitor;
      continue;
    }

    if (rotor_state[i].starting) *state |= 1 << i;
    if (rotor_state[i].spinning) continue;

    if ((!rotor_state[i].starting && (*state & (1 << i))) ||
        rotor_state[i].emerg) {
      mk_send_msg(&conn->chan[0], "x");
      e.id = 1 + i;
      return mikrokopter_e_rotor_failure(&e, self);
    }

    return mikrokopter_pause_monitor;
  }

  return mikrokopter_ether;
}

/** Codel mk_start_stop of activity start.
 *
 * Triggered by mikrokopter_stop.
 * Yields to mikrokopter_pause_stop, mikrokopter_ether.
 * Throws mikrokopter_e_connection, mikrokopter_e_started,
 *        mikrokopter_e_rotor_failure,
 *        mikrokopter_e_rotor_not_disabled.
 */
genom_event
mk_start_stop(const mikrokopter_conn_s *conn,
              const or_rotorcraft_rotor_state rotor_state[8],
              const genom_context self)
{
  genom_event e;

  e = mk_stop(conn, rotor_state, self);
  if (e == mikrokopter_ether) return mikrokopter_ether;

  return mikrokopter_pause_stop;
}


/* --- Activity servo --------------------------------------------------- */

/** Codel mk_servo_start of activity servo.
 *
 * Triggered by mikrokopter_start.
 * Yields to mikrokopter_main.
 * Throws mikrokopter_e_connection, mikrokopter_e_rotor_failure,
 *        mikrokopter_e_input.
 */
genom_event
mk_servo_start(double *scale, const genom_context self)
{
  (void)self;

  *scale = 0.;
  return mikrokopter_main;
}

/** Codel mk_servo_main of activity servo.
 *
 * Triggered by mikrokopter_main.
 * Yields to mikrokopter_pause_main, mikrokopter_stop.
 * Throws mikrokopter_e_connection, mikrokopter_e_rotor_failure,
 *        mikrokopter_e_input.
 */
genom_event
mk_servo_main(const mikrokopter_conn_s *conn,
              mikrokopter_ids_rotor_data_s *rotor_data,
              const mikrokopter_rotor_input *rotor_input,
              const mikrokopter_ids_servo_s *servo, double *scale,
              const genom_context self)
{
  or_rotorcraft_input *input_data;
  struct timeval tv;
  genom_event e;

  if (!conn) return mikrokopter_e_connection(self);

  /* update input */
  if (rotor_input->read(self)) return mikrokopter_e_input(self);

  input_data = rotor_input->data(self);
  if (!input_data) return mikrokopter_e_input(self);

  /* watchdog */
  gettimeofday(&tv, NULL);
  if (tv.tv_sec + 1e-6*tv.tv_usec >
      0.5 + input_data->ts.sec + 1e-9*input_data->ts.nsec) {
    /* do something smart here, instead of the following */
    return mikrokopter_stop;
  }

  /* linear input scaling for the first servo->ramp seconds */
  if (*scale < 1.) {
    static or_time_ts tvp;

    /* don't scale twice */
    if (tvp.sec != input_data->ts.sec || tvp.nsec != input_data->ts.nsec) {
      size_t i;
      for(i = 0; i < input_data->desired._length; i++)
        input_data->desired._buffer[i] *= *scale;
    }
    tvp = input_data->ts;

    *scale += 1e-3 * mikrokopter_control_period_ms / servo->ramp;
  }

  /* send */
  switch(input_data->control) {
    case or_rotorcraft_velocity:
      e = mk_set_velocity(
        conn, rotor_data, &input_data->desired, self);
      break;

    case or_rotorcraft_throttle:
      e = mk_set_throttle(
        conn, rotor_data, &input_data->desired, self);
      break;
  }
  if (e) return e;

  return mikrokopter_pause_main;
}

/** Codel mk_servo_stop of activity servo.
 *
 * Triggered by mikrokopter_stop.
 * Yields to mikrokopter_ether.
 * Throws mikrokopter_e_connection, mikrokopter_e_rotor_failure,
 *        mikrokopter_e_input.
 */
genom_event
mk_servo_stop(const mikrokopter_conn_s *conn,
              const genom_context self)
{
  uint16_t p[or_rotorcraft_max_rotors];
  int i;
  (void)self;

  for(i = 0; i < or_rotorcraft_max_rotors; i++) p[i] = 32767;

  mk_send_msg(&conn->chan[0], "w%@", p, or_rotorcraft_max_rotors);

  return mikrokopter_ether;
}


/* --- Activity stop ---------------------------------------------------- */

/** Codel mk_stop of activity stop.
 *
 * Triggered by mikrokopter_start.
 * Yields to mikrokopter_pause_start, mikrokopter_ether.
 */
genom_event
mk_stop(const mikrokopter_conn_s *conn,
        const or_rotorcraft_rotor_state state[8],
        const genom_context self)
{
  size_t i;

  if (!conn) return mikrokopter_e_connection(self);
  mk_send_msg(&conn->chan[0], "x");

  for(i = 0; i < or_rotorcraft_max_rotors; i++) {
    if (state[i].disabled) continue;
    if (state[i].spinning) return mikrokopter_pause_start;
  }

  return mikrokopter_ether;
}
