/*
 * Copyright (c) 2015-2019,2021 LAAS/CNRS
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
#ifndef H_ROTORCRAFT_CODELS
#define H_ROTORCRAFT_CODELS

#include <aio.h>
#include <errno.h>
#include <inttypes.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>
#include <termios.h>

#include "rotorcraft_c_types.h"

struct rotorcraft_log_s {
  struct aiocb req;
  char buffer[4096];
  bool pending, skipped;
  uint32_t decimation;
  size_t missed, total;

# define rotorcraft_logfmt	"%g "
# define rotorcraft_log_header                                          \
  "ts imu_rate mag_rate motor_rate "                                    \
  "imu_wx imu_wy imu_wz  imu_ax imu_ay imu_az  mag_x mag_y mag_z "      \
  "cmd_v0 cmd_v1 cmd_v2 cmd_v3 cmd_v4 cmd_v5 cmd_v6 cmd_v7 "            \
  "meas_v0 meas_v1 meas_v2 meas_v3 meas_v4 meas_v5 meas_v6 meas_v7 "    \
  "clk0 clk1 clk2 clk3 clk4 clk5 clk6 clk7"
# define rotorcraft_log_line                                            \
  "%" PRIu64 ".%09d " \
  rotorcraft_logfmt rotorcraft_logfmt rotorcraft_logfmt                 \
  rotorcraft_logfmt rotorcraft_logfmt rotorcraft_logfmt                 \
  rotorcraft_logfmt rotorcraft_logfmt rotorcraft_logfmt                 \
  rotorcraft_logfmt rotorcraft_logfmt rotorcraft_logfmt                 \
                                                                        \
  rotorcraft_logfmt rotorcraft_logfmt rotorcraft_logfmt                 \
  rotorcraft_logfmt rotorcraft_logfmt rotorcraft_logfmt                 \
  rotorcraft_logfmt rotorcraft_logfmt                                   \
                                                                        \
  rotorcraft_logfmt rotorcraft_logfmt rotorcraft_logfmt                 \
  rotorcraft_logfmt rotorcraft_logfmt rotorcraft_logfmt                 \
  rotorcraft_logfmt rotorcraft_logfmt                                   \
                                                                        \
  "%d " "%d " "%d " "%d " "%d " "%d " "%d " "%d"
};

struct mk_channel_s {
  char path[1024];
  int fd;

  uint8_t buf[64], r, w; /* read ring buffer */

  bool start;
  bool escape;
  uint8_t msg[64], len; /* last message */
};

enum rc_device {
  RC_NONE,
  RC_MKBL,
  RC_MKFL,
  RC_FLYMU,
  RC_CHIMERA
};

struct rotorcraft_conn_s {
  struct mk_channel_s chan;
  enum rc_device device;
};

static inline genom_event
mk_e_sys_error(const char *s, genom_context self)
{
  rotorcraft_e_sys_detail d;
  size_t l = 0;

  d.code = errno;
  if (s) {
    strncpy(d.what, s, sizeof(d.what) - 3);
    l = strlen(s);
    strcpy(d.what + l, ": ");
    l += 2;
  }
  strerror_r(d.code, d.what + l, sizeof(d.what) - l);
  return rotorcraft_e_sys(&d, self);
}

int	mk_open_tty(const char *device, speed_t baud);
int	mk_wait_msg(const struct mk_channel_s *channel);
int	mk_recv_msg(struct mk_channel_s *chan, bool block);
int	mk_send_msg(const struct mk_channel_s *chan, const char *fmt, ...);

#ifdef __cplusplus
extern "C" {
#endif

  int	mk_calibration_init(uint32_t sstill, uint32_t nposes, uint32_t sps);
  int	mk_calibration_collect(or_pose_estimator_state *imu_data,
                               or_pose_estimator_state *mag_data,
                               int32_t *still);
  int	mk_calibration_acc(double ascale[9], double abias[3]);
  int	mk_calibration_gyr(double gscale[9], double gbias[3]);
  int	mk_calibration_mag(double mscale[9], double mbias[3]);
  void	mk_calibration_fini(double stddeva[3], double stddevw[3],
                double stddevm[3], double *maxa, double *maxw, double *avga,
                double *avgw);
  void	mk_calibration_log(const char *path);

  void	mk_calibration_rotate(double r[9], double s[9]);
  void	mk_calibration_bias(double b1[3], double s[9], double b[3]);

#ifdef __cplusplus
}
#endif

#endif /* H_ROTORCRAFT_CODELS */
