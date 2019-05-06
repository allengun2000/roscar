/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * later_car.h
 *
 * Code generation for function 'later_car'
 *
 */

#ifndef LATER_CAR_H
#define LATER_CAR_H

/* Include files */
#include <stddef.h>
#include <stdlib.h>
#include "rtwtypes.h"
#include "later_car_types.h"

/* Function Declarations */
extern double later_car(double refPose[3], double currPose[3], double
  currVelocity, boolean_T direction, double gain, double wheelbase, double
  maxSteer);

#endif

/* End of code generation (later_car.h) */
