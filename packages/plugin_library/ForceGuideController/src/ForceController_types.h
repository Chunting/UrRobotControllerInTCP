//
// File: ForceController_types.h
//
// Code generated for Simulink model 'ForceController'.
//
// Model version                  : 1.84
// Simulink Coder version         : 8.11 (R2016b) 25-Aug-2016
// C/C++ source code generated on : Sat Apr 15 16:34:54 2017
//
// Target selection: ert.tlc
// Embedded hardware selection: 32-bit Generic
// Code generation objectives: Unspecified
// Validation result: Not run
//
#ifndef RTW_HEADER_ForceController_types_h_
#define RTW_HEADER_ForceController_types_h_
#include "rtwtypes.h"
#ifndef DEFINED_TYPEDEF_FOR_pid_param_
#define DEFINED_TYPEDEF_FOR_pid_param_

typedef struct {
  real_T P;
  real_T I;
  real_T D;
  real_T N;
} pid_param;

#endif

// Parameters (auto storage)
typedef struct P_ForceController_T_ P_ForceController_T;

#endif                                 // RTW_HEADER_ForceController_types_h_

//
// File trailer for generated code.
//
// [EOF]
//
