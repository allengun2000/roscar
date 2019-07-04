/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * angleUtilities.cpp
 *
 * Code generation for function 'angleUtilities'
 *
 */

/* Include files */
#include <cmath>
#include "rt_nonfinite.h"
#include "later_car.h"
#include "angleUtilities.h"

/* Function Definitions */
void angleUtilities_wrapTo2Pi(double *theta)
{
  boolean_T positiveInput;
  double x;
  boolean_T rEQ0;
  double q;
  positiveInput = (*theta > 0.0);
  x = *theta;
  if (rtIsNaN(x) || rtIsInf(x)) {
    *theta = rtNaN;
  } else if (x == 0.0) {
    *theta = 0.0;
  } else {
    *theta = std::fmod(x, 6.2831853071795862);
    rEQ0 = (*theta == 0.0);
    if (!rEQ0) {
      q = std::abs(x / 6.2831853071795862);
      rEQ0 = (std::abs(q - std::floor(q + 0.5)) <= 2.2204460492503131E-16 * q);
    }

    if (rEQ0) {
      *theta = 0.0;
    } else {
      if (x < 0.0) {
        *theta += 6.2831853071795862;
      }
    }
  }

  *theta += 6.2831853071795862 * static_cast<double>(((*theta == 0.0) &&
    positiveInput));
}

/* End of code generation (angleUtilities.cpp) */
