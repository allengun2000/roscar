/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * later_car.cpp
 *
 * Code generation for function 'later_car'
 *
 */

/* Include files */
#include <cmath>
#include "rt_nonfinite.h"
#include "later_car.h"
#include "angleUtilities.h"

/* Function Definitions */
double later_car(double refPose[3], double currPose[3], double currVelocity,
                 boolean_T direction, double gain, double wheelbase, double
                 maxSteer)
{
  double steerCmd;
  double d_idx_0;
  double d_idx_1;
  double b_d_idx_0;

  /*  Parse and check inputs */
  /*  Clip heading angle to be within [0, 360) and convert to radian */
  refPose[2] *= 0.017453292519943295;
  angleUtilities_wrapTo2Pi(&refPose[2]);
  currPose[2] *= 0.017453292519943295;
  angleUtilities_wrapTo2Pi(&currPose[2]);

  /*  Compute position error */
  /* -------------------------------------------------------------------------- */
  /* computePositionError Compute the distance error */
  /*  Tangent direction of the reference path */
  /*  Tracking error vector */
  if (direction) {
    d_idx_0 = (currPose[0] + wheelbase * std::cos(currPose[2])) - refPose[0];
    d_idx_1 = (currPose[1] + wheelbase * std::sin(currPose[2])) - refPose[1];
  } else {
    d_idx_0 = currPose[0] - refPose[0];
    d_idx_1 = currPose[1] - refPose[1];
  }

  /*  Implement equation (8) in reference [2] */
  d_idx_0 = -(d_idx_0 * std::sin(refPose[2]) - d_idx_1 * std::cos(refPose[2]));

  /*  Compute heading error */
  /* -------------------------------------------------------------------------- */
  /* computeSteeringAngleError Compute the steering angle error */
  d_idx_1 = (currPose[2] - refPose[2]) + 3.1415926535897931;
  angleUtilities_wrapTo2Pi(&d_idx_1);

  /*  Avoid oversensitiveness at low speeds */
  /*  Compute steering angle by implementing equation (5) in reference [1] */
  if (direction) {
    d_idx_0 = -((d_idx_1 - 3.1415926535897931) + std::atan(gain * d_idx_0 / (1.0
      + currVelocity)));
  } else {
    /*  In reverse motion, rear wheel velocity is required to compute the */
    /*  steering angle. However, this will require the current steering angle  */
    /*  as an input. To simplify the design without affecting the performance */
    /*  of the controller, the front wheel velocity is used. */
    d_idx_0 = (d_idx_1 - 3.1415926535897931) + std::atan(gain * d_idx_0 / (-1.0
      + currVelocity));
  }

  /*  Saturate and convert to degree */
  steerCmd = 57.295779513082323 * d_idx_0;

  /* -------------------------------------------------------------------------- */
  /* -------------------------------------------------------------------------- */
  /*  Helper function */
  /* -------------------------------------------------------------------------- */
  /* -------------------------------------------------------------------------- */
  /* saturate Saturate steering angle when it exceeds MaxSteeringAngle */
  d_idx_1 = steerCmd;
  if (steerCmd < 0.0) {
    d_idx_1 = -1.0;
  } else if (steerCmd > 0.0) {
    d_idx_1 = 1.0;
  } else {
    if (steerCmd == 0.0) {
      d_idx_1 = 0.0;
    }
  }

  d_idx_0 = std::abs(steerCmd);
  if ((d_idx_0 < maxSteer) || rtIsNaN(maxSteer)) {
    b_d_idx_0 = d_idx_0;
  } else {
    b_d_idx_0 = maxSteer;
  }

  return d_idx_1 * b_d_idx_0;
}

/* End of code generation (later_car.cpp) */
