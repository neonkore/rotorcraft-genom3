/*
 * Copyright (c) 2016 LAAS/CNRS
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
 *                                      Anthony Mallet on Wed Oct 19 2016
 */

#include <math.h>
#include <stdio.h>
#include <stdlib.h>

#include "codels.h"


/* --- imu_iirf_init ------------------------------------------------------- */

/* Notch filter coefficients object */
#define IIRF_COEFFS 256
static struct br_coeffs {
  double d[3];
} imu_iirf_coeff[IIRF_COEFFS + 1];
static double iirf_fmin, iirf_fmax;

/* This initialization function will create the
 * notch filter coefficients array, you have to specify:
 * fsfilt = Sampling Frequency
 * gb     = Gain at cut frequencies
 * Q      = Q factor, Higher Q gives narrower band
 * fmin   = Minimum frequency for the range of center frequencies
 * fmax   = Maximum frequency for the range of center frequencies
 */
void
mk_imu_iirf_init(double fsfilt, double gb, double Q, double fmin, double fmax)
{
  int i;
  double damp;
  double wo, e, p;

  iirf_fmin = fmin;
  iirf_fmax = fmax;

  damp = sqrt(1 - gb*gb)/gb;
  for(i = 0; i <= IIRF_COEFFS; i++) {
    wo = 2*M_PI*(fmin + (fmax - fmin)*i/IIRF_COEFFS)/fsfilt;
    e = 1/(1 + damp * tan(wo/(Q*2)));
    p = cos(wo);

    imu_iirf_coeff[i].d[0] = e;
    imu_iirf_coeff[i].d[1] = 2 * e * p;
    imu_iirf_coeff[i].d[2] = 2 * e - 1.;
  }
}


/* --- mk_imu_iirf --------------------------------------------------------- */

/* This function does the actual notch filter processing
 * v = input sample
 * H = filter object
 * f0 = center frequency
 */
double
mk_imu_iirf(double v, struct mk_iir_filter *H, double f0)
{
  int i = (f0 - iirf_fmin) / (iirf_fmax - iirf_fmin) * IIRF_COEFFS;
  if (i < 0) i = 0; else if (i > IIRF_COEFFS) i = IIRF_COEFFS;

  if (isnan(H->x[0])) {
    H->x[0] = H->x[1] = v;
    H->y[0] = H->y[1] = v;
  } else {
    H->x[0] = H->x[1];
    H->x[1] = H->x[2];

    H->y[0] = H->y[1];
    H->y[1] = H->y[2];
  }

  H->x[2] = v;
  H->y[2] =
    imu_iirf_coeff[i].d[0] * H->x[2]
    - imu_iirf_coeff[i].d[1] * H->x[1]
    + imu_iirf_coeff[i].d[0] * H->x[0]
    + imu_iirf_coeff[i].d[1] * H->y[1]
    - imu_iirf_coeff[i].d[2] * H->y[0];

  return H->y[2];
}
