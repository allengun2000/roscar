/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * _coder_later_car_api.h
 *
 * Code generation for function '_coder_later_car_api'
 *
 */

#ifndef _CODER_LATER_CAR_API_H
#define _CODER_LATER_CAR_API_H

/* Include files */
#include "tmwtypes.h"
#include "mex.h"
#include "emlrt.h"
#include <stddef.h>
#include <stdlib.h>
#include "_coder_later_car_api.h"

/* Variable Declarations */
extern emlrtCTX emlrtRootTLSGlobal;
extern emlrtContext emlrtContextGlobal;

/* Function Declarations */
extern real_T later_car(real_T refPose[3], real_T currPose[3], real_T
  currVelocity, boolean_T direction, real_T gain, real_T wheelbase, real_T
  maxSteer);
extern void later_car_api(const mxArray *prhs[7], int32_T nlhs, const mxArray
  *plhs[1]);
extern void later_car_atexit(void);
extern void later_car_initialize(void);
extern void later_car_terminate(void);
extern void later_car_xil_shutdown(void);
extern void later_car_xil_terminate(void);

#endif

/* End of code generation (_coder_later_car_api.h) */
