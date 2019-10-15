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
#include <err.h>
#include <float.h>
#include <math.h>
#include <unistd.h>

#include "rotorcraft_c_types.h"
#include "codels.h"


/* --- Task main -------------------------------------------------------- */

static struct mk_iir_filter Hax, Hay, Haz, Hwx, Hwy, Hwz;


/** Codel mk_main_init of task main.
 *
 * Triggered by rotorcraft_start.
 * Yields to rotorcraft_main.
 */
genom_event
mk_main_init(rotorcraft_ids *ids, const rotorcraft_imu *imu,
             const rotorcraft_mag *mag, const genom_context self)
{
  genom_event e;
  size_t i;

  ids->sensor_time = (rotorcraft_ids_sensor_time_s){
    .rate = { .imu = 1000., .mag = 100., .motor = 100., .battery = 1. }
  };
  ids->publish_time = (rotorcraft_ids_publish_time_s){
    .imu = { 0 }, .mag =  { 0 }, .motor = {{ 0 }}
  };
  e = mk_set_sensor_rate(
    &ids->sensor_time.rate, NULL, &ids->sensor_time, self);
  if (e) return e;

  ids->battery.min = 14.0;
  ids->battery.max = 16.8;
  ids->battery.level = 0.;

  ids->imu_calibration = (rotorcraft_ids_imu_calibration_s){
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

  ids->servo.timeout = 30.;
  ids->servo.ramp = 3.;

  /* init logging */
  ids->log = malloc(sizeof(*ids->log));
  if (!ids->log) abort();
  *ids->log = (rotorcraft_log_s){
    .req = {
      .aio_fildes = -1,
      .aio_offset = 0,
      .aio_buf = ids->log->buffer,
      .aio_nbytes = 0,
      .aio_reqprio = 0,
      .aio_sigevent = { .sigev_notify = SIGEV_NONE },
      .aio_lio_opcode = LIO_NOP
    },
    .pending = false, .skipped = false,
    .decimation = 1, .missed = 0, .total = 0
  };

  *imu->data(self) = *mag->data(self) = (or_pose_estimator_state){
    .ts = { 0, 0 },
    .intrinsic = true,

    .pos._present = false,
    .att._present = false,
    .pos_cov._present = false,
    .att_cov._present = false,
    .att_pos_cov._present = false,

    .vel._present = false,
    .vel_cov._present = false,
    .avel._present = false,
    .avel_cov._present = false,

    .acc._present = false,
    .acc_cov._present = false,
    .aacc._present = false,
    .aacc_cov._present = false
  };

  return rotorcraft_main;
}


/** Codel mk_main_perm of task main.
 *
 * Triggered by rotorcraft_main.
 * Yields to rotorcraft_pause_main.
 */
genom_event
mk_main_perm(const rotorcraft_conn_s *conn,
             const rotorcraft_ids_battery_s *battery,
             const rotorcraft_ids_imu_calibration_s *imu_calibration,
             const rotorcraft_ids_imu_filter_s *imu_filter,
             const rotorcraft_ids_rotor_data_s *rotor_data,
             rotorcraft_ids_sensor_time_s *sensor_time,
             rotorcraft_ids_publish_time_s *publish_time,
             bool *imu_calibration_updated, rotorcraft_log_s **log,
             const rotorcraft_rotor_measure *rotor_measure,
             const rotorcraft_imu *imu, const rotorcraft_mag *mag,
             const genom_context self)
{
  or_pose_estimator_state *idata = imu->data(self);
  or_pose_estimator_state *mdata = mag->data(self);
  or_rotorcraft_output *rdata = rotor_measure->data(self);
  struct timeval tv;
  ssize_t i;

  gettimeofday(&tv, NULL);

  /* battery level */
  if (conn && conn->chan.fd >= 0) {
    if (battery->level > 0. && battery->level < battery->min) {
      static int cnt;

      if (!cnt) mk_send_msg(&conn->chan, "~%2", 440);
      cnt = (cnt + 1) % 500;
    }
  }

  /* imu covariance data */
  if (*imu_calibration_updated) {
    idata->avel_cov._value.cov[0] =
      imu_calibration->gstddev[0] * imu_calibration->gstddev[0];
    idata->avel_cov._value.cov[1] = 0.;
    idata->avel_cov._value.cov[2] =
      imu_calibration->gstddev[1] * imu_calibration->gstddev[1];
    idata->avel_cov._value.cov[3] = 0.;
    idata->avel_cov._value.cov[4] = 0.;
    idata->avel_cov._value.cov[5] =
      imu_calibration->gstddev[2] * imu_calibration->gstddev[2];
    idata->avel_cov._present = true;

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

      idata->avel._value.wx = mk_imu_iirf(idata->avel._value.wx, &Hwx, favg);
      idata->avel._value.wy = mk_imu_iirf(idata->avel._value.wy, &Hwy, favg);
      idata->avel._value.wz = mk_imu_iirf(idata->avel._value.wz, &Hwz, favg);
    }
  }


  /* publish, only if timestamps changed */
  for(i = 0; i < or_rotorcraft_max_rotors; i++) {
    rdata->rotor._buffer[i] = rotor_data->state[i];
    if (!rotor_data->state[i].disabled)
      rdata->rotor._length = i + 1;
  }

  for(i = 0; i < or_rotorcraft_max_rotors; i++) {
    if (publish_time->motor[i].sec != rotor_data->state[i].ts.sec ||
        publish_time->motor[i].nsec != rotor_data->state[i].ts.nsec) {
      rotor_measure->write(self);
      for(; i < or_rotorcraft_max_rotors; i++)
        publish_time->motor[i] = rotor_data->state[i].ts;
      break;
    }
  }

  if (publish_time->imu.sec != idata->ts.sec ||
      publish_time->imu.nsec != idata->ts.nsec) {
    imu->write(self);
    publish_time->imu = idata->ts;
  }

  if (publish_time->mag.sec != mdata->ts.sec ||
      publish_time->mag.nsec != mdata->ts.nsec) {
    mag->write(self);
    publish_time->mag = mdata->ts;
  }


  /* update sensor time */
  {
    double rate;

    rate = 1./ (
      tv.tv_sec - idata->ts.sec +
      (1 + tv.tv_usec * 1000 - idata->ts.nsec) * 1e-9)	;
    if (rate < 0.1 * sensor_time->rate.imu)
      sensor_time->measured_rate.imu = 0.;

    rate = 1./ (
      tv.tv_sec - mdata->ts.sec +
      (1 + tv.tv_usec * 1000 - mdata->ts.nsec) * 1e-9)	;
    if (rate < 0.1 * sensor_time->rate.mag)
      sensor_time->measured_rate.mag = 0.;

    for(i = 0; i < or_rotorcraft_max_rotors; i++) {
      if (rotor_data->state[i].disabled) continue;

      rate = 1. / (
        tv.tv_sec - rotor_data->state[i].ts.sec +
        (1 + tv.tv_usec * 1000 - rotor_data->state[i].ts.nsec) * 1e-9);
      if (rate < 0.1 * sensor_time->rate.motor) {
        sensor_time->measured_rate.motor = 0.;
      }
    }
  }


  /* log */
  if ((*log)->req.aio_fildes >= 0) {
    (*log)->total++;
    if ((*log)->total % (*log)->decimation == 0) {
      if ((*log)->pending) {
        if (aio_error(&(*log)->req) != EINPROGRESS) {
          (*log)->pending = false;
          if (aio_return(&(*log)->req) <= 0) {
            warn("log");
            close((*log)->req.aio_fildes);
            (*log)->req.aio_fildes = -1;
          }
        } else {
          (*log)->skipped = true;
          (*log)->missed++;
        }
      }
    }

    if ((*log)->req.aio_fildes >= 0 && !(*log)->pending) {
      (*log)->req.aio_nbytes = snprintf(
        (*log)->buffer, sizeof((*log)->buffer),
        "%s" rotorcraft_log_line "\n",
        (*log)->skipped ? "\n" : "",
        (uint64_t)tv.tv_sec, (uint32_t)tv.tv_usec * 1000,
        sensor_time->measured_rate.imu,
        sensor_time->measured_rate.mag,
        sensor_time->measured_rate.motor,
        idata->avel._value.wx, idata->avel._value.wy, idata->avel._value.wz,
        idata->acc._value.ax, idata->acc._value.ay, idata->acc._value.az,
        mdata->att._value.qx, mdata->att._value.qy, mdata->att._value.qz,

        rotor_data->wd[0], rotor_data->wd[1], rotor_data->wd[2],
        rotor_data->wd[3], rotor_data->wd[4], rotor_data->wd[5],
        rotor_data->wd[6], rotor_data->wd[7],

        rdata->rotor._buffer[0].velocity, rdata->rotor._buffer[1].velocity,
        rdata->rotor._buffer[2].velocity, rdata->rotor._buffer[3].velocity,
        rdata->rotor._buffer[4].velocity, rdata->rotor._buffer[5].velocity,
        rdata->rotor._buffer[6].velocity, rdata->rotor._buffer[7].velocity,

        rotor_data->clkrate[0], rotor_data->clkrate[1], rotor_data->clkrate[2],
        rotor_data->clkrate[3], rotor_data->clkrate[4], rotor_data->clkrate[5],
        rotor_data->clkrate[6], rotor_data->clkrate[7]);

      if (aio_write(&(*log)->req)) {
        warn("log");
        close((*log)->req.aio_fildes);
        (*log)->req.aio_fildes = -1;
      } else
        (*log)->pending = true;

      (*log)->skipped = false;
    }
  }

  return rotorcraft_pause_main;
}


/** Codel mk_main_stop of task main.
 *
 * Triggered by rotorcraft_stop.
 * Yields to rotorcraft_ether.
 */
genom_event
mk_main_stop(rotorcraft_log_s **log, const genom_context self)
{
  mk_log_stop(log, self);
  if (*log) free(*log);

  return rotorcraft_ether;
}


/* --- Activity calibrate_imu ------------------------------------------- */

/** Codel mk_calibrate_imu_start of activity calibrate_imu.
 *
 * Triggered by rotorcraft_start.
 * Yields to rotorcraft_collect.
 * Throws rotorcraft_e_sys, rotorcraft_e_connection.
 */
genom_event
mk_calibrate_imu_start(double tstill, uint16_t nposes,
                       const genom_context self)
{
  uint32_t sps;
  int s;

  sps = 1000/rotorcraft_control_period_ms;
  s = mk_calibration_init(tstill * sps, nposes, sps);
  if (s) {
    errno = s;
    return mk_e_sys_error("calibration", self);
  }

  warnx("calibration started");
  return rotorcraft_collect;
}

/** Codel mk_calibrate_imu_collect of activity calibrate_imu.
 *
 * Triggered by rotorcraft_collect.
 * Yields to rotorcraft_pause_collect, rotorcraft_main.
 * Throws rotorcraft_e_sys, rotorcraft_e_connection.
 */
genom_event
mk_calibrate_imu_collect(const rotorcraft_imu *imu,
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
      return rotorcraft_pause_collect;

    default:
      warnx("calibration aborted");
      mk_calibration_fini(NULL, NULL, NULL, NULL);
      errno = s;
      return mk_e_sys_error("calibration", self);
  }

  warnx("calibration acquired all poses");
  return rotorcraft_main;
}

/** Codel mk_calibrate_imu_main of activity calibrate_imu.
 *
 * Triggered by rotorcraft_main.
 * Yields to rotorcraft_ether.
 * Throws rotorcraft_e_sys, rotorcraft_e_connection.
 */
genom_event
mk_calibrate_imu_main(rotorcraft_ids_imu_calibration_s *imu_calibration,
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
  return rotorcraft_ether;
}


/* --- Activity set_zero ------------------------------------------------ */

/** Codel mk_set_zero_start of activity set_zero.
 *
 * Triggered by rotorcraft_start.
 * Yields to rotorcraft_collect.
 * Throws rotorcraft_e_sys.
 */
genom_event
mk_set_zero_start(double accum[3], double gycum[3], uint32_t *n,
                  const genom_context self)
{
  (void)self;

  gycum[0] = gycum[1] = gycum[2] = 0.;
  accum[0] = accum[1] = accum[2] = 0.;
  *n = 0;
  return rotorcraft_collect;
}

/** Codel mk_set_zero_collect of activity set_zero.
 *
 * Triggered by rotorcraft_collect.
 * Yields to rotorcraft_pause_collect, rotorcraft_main.
 * Throws rotorcraft_e_sys.
 */
genom_event
mk_set_zero_collect(const rotorcraft_imu *imu, double accum[3],
                    double gycum[3], uint32_t *n,
                    const genom_context self)
{
  or_pose_estimator_state *imu_data = imu->data(self);

  if (!imu_data->avel._present || !imu_data->acc._present) {
    errno = EIO;
    return mk_e_sys_error("set_zero", self);
  }

  gycum[0] = (*n * gycum[0] - imu_data->avel._value.wx) / (1 + *n);
  gycum[1] = (*n * gycum[1] - imu_data->avel._value.wy) / (1 + *n);
  gycum[2] = (*n * gycum[2] - imu_data->avel._value.wz) / (1 + *n);

  accum[0] = (*n * accum[0] + imu_data->acc._value.ax) / (1 + *n);
  accum[1] = (*n * accum[1] + imu_data->acc._value.ay) / (1 + *n);
  accum[2] = (*n * accum[2] + imu_data->acc._value.az) / (1 + *n);

  return ((*n)++ < 2000.) ? rotorcraft_pause_collect : rotorcraft_main;
}

/** Codel mk_set_zero of activity set_zero.
 *
 * Triggered by rotorcraft_main.
 * Yields to rotorcraft_ether.
 * Throws rotorcraft_e_sys.
 */
genom_event
mk_set_zero(double accum[3], double gycum[3],
            rotorcraft_ids_imu_calibration_s *imu_calibration,
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
  return rotorcraft_ether;
}


/* --- Activity start --------------------------------------------------- */

/** Codel mk_start_start of activity start.
 *
 * Triggered by rotorcraft_start.
 * Yields to rotorcraft_pause_start, rotorcraft_monitor.
 * Throws rotorcraft_e_connection, rotorcraft_e_started,
 *        rotorcraft_e_sys, rotorcraft_e_rotor_failure,
 *        rotorcraft_e_rotor_not_disabled.
 */
genom_event
mk_start_start(const rotorcraft_conn_s *conn,
               const rotorcraft_ids_servo_s *servo, uint32_t *timeout,
               uint16_t *state,
               const or_rotorcraft_rotor_state rotor_state[8],
               const genom_context self)
{
  size_t i;

  if (!conn) return rotorcraft_e_connection(self);
  for(i = 0; i < or_rotorcraft_max_rotors; i++) {
    if (rotor_state[i].disabled) continue;
    if (rotor_state[i].spinning) return rotorcraft_e_started(self);
  }

  *timeout = servo->timeout * 1e3 / rotorcraft_control_period_ms;
  *state = 0;
  for(i = 0; i < or_rotorcraft_max_rotors; i++) {
    if (rotor_state[i].disabled) continue;
    mk_send_msg(&conn->chan, "g%1", (uint8_t){i+1});

    /* wait until motor have cleared any emergency flag */
    if (rotor_state[i].emerg) return rotorcraft_pause_start;
  }

  return rotorcraft_monitor;
}

/** Codel mk_start_monitor of activity start.
 *
 * Triggered by rotorcraft_monitor.
 * Yields to rotorcraft_pause_monitor, rotorcraft_ether.
 * Throws rotorcraft_e_connection, rotorcraft_e_started,
 *        rotorcraft_e_sys, rotorcraft_e_rotor_failure,
 *        rotorcraft_e_rotor_not_disabled.
 */
genom_event
mk_start_monitor(const rotorcraft_conn_s *conn,
                 const rotorcraft_ids_sensor_time_s *sensor_time,
                 uint32_t *timeout, uint16_t *state,
                 const or_rotorcraft_rotor_state rotor_state[8],
                 const genom_context self)
{
  rotorcraft_e_rotor_failure_detail e;
  rotorcraft_e_rotor_not_disabled_detail d;
  size_t i;
  bool complete;

  if (!(*timeout)--) {
    mk_send_msg(&conn->chan, "x");
    errno = EAGAIN;
    return mk_e_sys_error("start", self);
  }

  complete = true;
  for(i = 0; i < or_rotorcraft_max_rotors; i++) {
    if (rotor_state[i].disabled) {
      if (rotor_state[i].starting || rotor_state[i].spinning) {
        mk_send_msg(&conn->chan, "x");
        d.id = 1 + i;
        return rotorcraft_e_rotor_not_disabled(&d, self);
      }

      continue;
    }

    if (rotor_state[i].starting) *state |= 1 << i;
    if (rotor_state[i].spinning) continue;

    if ((!rotor_state[i].starting && (*state & (1 << i))) ||
        rotor_state[i].emerg) {
      mk_send_msg(&conn->chan, "x");
      e.id = 1 + i;
      return rotorcraft_e_rotor_failure(&e, self);
    }

    /* resend startup message every 100 periods */
    if (!rotor_state[i].starting && *timeout % 100 == 0)
      mk_send_msg(&conn->chan, "g%1", (uint8_t){i+1});

    complete = false;
  }

  /* check sensor rate */
  if (sensor_time->measured_rate.imu < 0.8 * sensor_time->rate.imu ||
      sensor_time->measured_rate.motor < 0.8 * sensor_time->rate.motor)
    complete = false;

  return complete ? rotorcraft_ether : rotorcraft_pause_monitor;
}

/** Codel mk_start_stop of activity start.
 *
 * Triggered by rotorcraft_stop.
 * Yields to rotorcraft_pause_stop, rotorcraft_ether.
 * Throws rotorcraft_e_connection, rotorcraft_e_started,
 *        rotorcraft_e_sys, rotorcraft_e_rotor_failure,
 *        rotorcraft_e_rotor_not_disabled.
 */
genom_event
mk_start_stop(const rotorcraft_conn_s *conn,
              const or_rotorcraft_rotor_state rotor_state[8],
              const genom_context self)
{
  genom_event e;

  e = mk_stop(conn, rotor_state, self);
  if (e == rotorcraft_ether) return rotorcraft_ether;

  return rotorcraft_pause_stop;
}


/* --- Activity servo --------------------------------------------------- */

/** Codel mk_servo_start of activity servo.
 *
 * Triggered by rotorcraft_start.
 * Yields to rotorcraft_main.
 * Throws rotorcraft_e_connection, rotorcraft_e_rotor_failure,
 *        rotorcraft_e_rate, rotorcraft_e_input.
 */
genom_event
mk_servo_start(double *scale, const genom_context self)
{
  (void)self;

  *scale = 0.;
  return rotorcraft_main;
}

/** Codel mk_servo_main of activity servo.
 *
 * Triggered by rotorcraft_main.
 * Yields to rotorcraft_pause_main, rotorcraft_stop.
 * Throws rotorcraft_e_connection, rotorcraft_e_rotor_failure,
 *        rotorcraft_e_rate, rotorcraft_e_input.
 */
genom_event
mk_servo_main(const rotorcraft_conn_s *conn,
              const rotorcraft_ids_sensor_time_s *sensor_time,
              rotorcraft_ids_rotor_data_s *rotor_data,
              const rotorcraft_rotor_input *rotor_input,
              const rotorcraft_ids_servo_s *servo, double *scale,
              const genom_context self)
{
  or_rotorcraft_input *input_data;
  struct timeval tv;
  genom_event e;
  int i;

  if (!conn) return rotorcraft_e_connection(self);

  /* update input */
  if (rotor_input->read(self)) return rotorcraft_e_input(self);

  input_data = rotor_input->data(self);
  if (!input_data) return rotorcraft_e_input(self);

  or_rotorcraft_rotor_control desired = input_data->desired;

  /* watchdog on input */
  gettimeofday(&tv, NULL);
  if (tv.tv_sec + 1e-6*tv.tv_usec >
      0.5 + input_data->ts.sec + 1e-9*input_data->ts.nsec) {

    *scale -= 2e-3 * rotorcraft_control_period_ms / servo->ramp;
    if (*scale < 0.) {
      mk_stop(conn, rotor_data->state, self);
      return rotorcraft_e_input(self);
    }
  }

  /* check sensor rate */
  if (sensor_time->measured_rate.imu < 0.9 * sensor_time->rate.imu ||
      sensor_time->measured_rate.motor < 0.9 * sensor_time->rate.motor) {

    *scale -= 2e-3 * rotorcraft_control_period_ms / servo->ramp;
    if (*scale < 0.) {
      mk_stop(conn, rotor_data->state, self);
      return rotorcraft_e_rate(self);
    }
  }

  /* check rotors status */
  for(i = 0; i < or_rotorcraft_max_rotors; i++) {
    if (rotor_data->state[i].disabled) continue;
    if (rotor_data->state[i].emerg || !rotor_data->state[i].spinning) {
      rotorcraft_e_rotor_failure_detail e;

      mk_stop(conn, rotor_data->state, self);
      e.id = 1 + i;
      return rotorcraft_e_rotor_failure(&e, self);
    }
  }

  /* linear input scaling for the first servo->ramp seconds or in case of
   * emergency */
  if (*scale < 1.) {
    size_t i;
    for(i = 0; i < input_data->desired._length; i++)
      desired._buffer[i] *= *scale;

    *scale += 1e-3 * rotorcraft_control_period_ms / servo->ramp;
  }

  /* send */
  switch(input_data->control) {
    case or_rotorcraft_velocity:
      e = mk_set_velocity(
        conn, rotor_data, &desired, self);
      if (e) return e;
      break;

    case or_rotorcraft_throttle:
      e = mk_set_throttle(
        conn, rotor_data, &desired, self);
      if (e) return e;
      break;
  }

  return rotorcraft_pause_main;
}

/** Codel mk_servo_stop of activity servo.
 *
 * Triggered by rotorcraft_stop.
 * Yields to rotorcraft_ether.
 * Throws rotorcraft_e_connection, rotorcraft_e_rotor_failure,
 *        rotorcraft_e_rate, rotorcraft_e_input.
 */
genom_event
mk_servo_stop(const rotorcraft_conn_s *conn, const genom_context self)
{
  uint16_t p[or_rotorcraft_max_rotors];
  int i;
  (void)self;

  for(i = 0; i < or_rotorcraft_max_rotors; i++) p[i] = 32767;

  mk_send_msg(&conn->chan, "w%@", p, or_rotorcraft_max_rotors);

  return rotorcraft_ether;
}


/* --- Activity stop ---------------------------------------------------- */

/** Codel mk_stop of activity stop.
 *
 * Triggered by rotorcraft_start.
 * Yields to rotorcraft_pause_start, rotorcraft_ether.
 */
genom_event
mk_stop(const rotorcraft_conn_s *conn,
        const or_rotorcraft_rotor_state state[8],
        const genom_context self)
{
  size_t i;

  if (!conn) return rotorcraft_e_connection(self);
  mk_send_msg(&conn->chan, "x");

  for(i = 0; i < or_rotorcraft_max_rotors; i++) {
    if (state[i].disabled) continue;
    if (state[i].spinning) return rotorcraft_pause_start;
  }

  return rotorcraft_ether;
}
