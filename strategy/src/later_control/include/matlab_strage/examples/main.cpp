/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * main.cpp
 *
 * Code generation for function 'main'
 *
 */

/*************************************************************************/
/* This automatically generated example C++ main file shows how to call  */
/* entry-point functions that MATLAB Coder generated. You must customize */
/* this file for your application. Do not modify this file directly.     */
/* Instead, make a copy of this file, modify it, and integrate it into   */
/* your development environment.                                         */
/*                                                                       */
/* This file initializes entry-point function arguments to a default     */
/* size and value before calling the entry-point functions. It does      */
/* not store or use any values returned from the entry-point functions.  */
/* If necessary, it does pre-allocate memory for returned values.        */
/* You can use this file as a starting point for a main function that    */
/* you can deploy in your application.                                   */
/*                                                                       */
/* After you copy the file, and before you deploy it, you must make the  */
/* following changes:                                                    */
/* * For variable-size function arguments, change the example sizes to   */
/* the sizes that your application requires.                             */
/* * Change the example values of function arguments to the values that  */
/* your application requires.                                            */
/* * If the entry-point functions return values, store these values or   */
/* otherwise use them as required by your application.                   */
/*                                                                       */
/*************************************************************************/
/* Include files */
#include "rt_nonfinite.h"
#include "later_car.h"
#include "main.h"
#include "later_car_terminate.h"
#include "later_car_initialize.h"

/* Function Declarations */
static void argInit_1x3_real_T(double result[3]);
static boolean_T argInit_boolean_T();
static double argInit_real_T();
static void main_later_car();

/* Function Definitions */
static void argInit_1x3_real_T(double result[3])
{
  double result_tmp;

  /* Loop over the array to initialize each element. */
  /* Set the value of the array element.
     Change this value to the value that the application requires. */
  result_tmp = argInit_real_T();
  result[0] = result_tmp;

  /* Set the value of the array element.
     Change this value to the value that the application requires. */
  result[1] = result_tmp;

  /* Set the value of the array element.
     Change this value to the value that the application requires. */
  result[2] = argInit_real_T();
}

static boolean_T argInit_boolean_T()
{
  return false;
}

static double argInit_real_T()
{
  return 0.0;
}

static void main_later_car()
{
  double refPose_tmp[3];
  double b_refPose_tmp[3];
  double c_refPose_tmp[3];
  double steerCmd;

  /* Initialize function 'later_car' input arguments. */
  /* Initialize function input argument 'refPose'. */
  argInit_1x3_real_T(refPose_tmp);

  /* Initialize function input argument 'currPose'. */
  /* Call the entry-point 'later_car'. */
  b_refPose_tmp[0] = refPose_tmp[0];
  c_refPose_tmp[0] = refPose_tmp[0];
  b_refPose_tmp[1] = refPose_tmp[1];
  c_refPose_tmp[1] = refPose_tmp[1];
  b_refPose_tmp[2] = refPose_tmp[2];
  c_refPose_tmp[2] = refPose_tmp[2];
  steerCmd = later_car(b_refPose_tmp, c_refPose_tmp, argInit_real_T(),
                       argInit_boolean_T(), argInit_real_T(), argInit_real_T(),
                       argInit_real_T());
}

int main(int, const char * const [])
{
  /* Initialize the application.
     You do not need to do this more than one time. */
  later_car_initialize();

  /* Invoke the entry-point functions.
     You can call entry-point functions multiple times. */
  main_later_car();

  /* Terminate the application.
     You do not need to do this more than one time. */
  later_car_terminate();
  return 0;
}

/* End of code generation (main.cpp) */
