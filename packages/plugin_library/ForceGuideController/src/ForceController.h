//
// File: ForceController.h
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
#ifndef RTW_HEADER_ForceController_h_
#define RTW_HEADER_ForceController_h_
#include <string.h>
#ifndef ForceController_COMMON_INCLUDES_
# define ForceController_COMMON_INCLUDES_
#include "rtwtypes.h"
#endif                                 // ForceController_COMMON_INCLUDES_

#include "ForceController_types.h"

// Macros for accessing real-time model data structure

// Block states (auto storage) for system '<Root>'
typedef struct {
  real_T filter_states[6];             // '<Root>/filter'
  real_T Integrator_DSTATE[6];         // '<S1>/Integrator'
  real_T Filter_DSTATE[6];             // '<S1>/Filter'
} DW_ForceController_T;

// External inputs (root inport signals with auto storage)
typedef struct {
  real_T force[6];                     // '<Root>/forceIn'
} ExtU_ForceController_T;

// Parameters (auto storage)
struct P_ForceController_T_ {
  pid_param pid;                       // Variable: pid
                                       //  Referenced by:
                                       //    '<S1>/Derivative Gain'
                                       //    '<S1>/Filter Coefficient'
                                       //    '<S1>/Integral Gain'
                                       //    '<S1>/Proportional Gain'

};

#ifdef __cplusplus

extern "C" {

#endif

#ifdef __cplusplus

}
#endif

// Class declaration for model ForceController
class ForceControllerModelClass {
  // public data and function members
 public:
  // External inputs
  ExtU_ForceController_T ForceController_U;

  // model initialize function
  void initialize();

  // model step function
  void step(const real_T (&force)[6], const real_T (&currentPose)[6], real_T
            (&targetPose)[6]);

  // model terminate function
  void terminate();

  // Constructor
  ForceControllerModelClass();

  // Destructor
  ~ForceControllerModelClass();

  // private data and function members
 private:
  // Tunable parameters
  P_ForceController_T ForceController_P;

  // Block states
  DW_ForceController_T ForceController_DW;
};

//-
//  These blocks were eliminated from the model due to optimizations:
//
//  Block '<S2>/Bias' : Eliminated nontunable bias of 0


//-
//  The generated code includes comments that allow you to trace directly
//  back to the appropriate location in the model.  The basic format
//  is <system>/block_name, where system is the system number (uniquely
//  assigned by Simulink) and block_name is the name of the block.
//
//  Use the MATLAB hilite_system command to trace the generated code back
//  to the model.  For example,
//
//  hilite_system('<S3>')    - opens system 3
//  hilite_system('<S3>/Kp') - opens and selects block Kp which resides in S3
//
//  Here is the system hierarchy for this model
//
//  '<Root>' : 'ForceController'
//  '<S1>'   : 'ForceController/Discrete PID Controller'
//  '<S2>'   : 'ForceController/GravityCompensation'
//  '<S3>'   : 'ForceController/GravityCompensation/FrameTranslation'

#endif                                 // RTW_HEADER_ForceController_h_

//
// File trailer for generated code.
//
// [EOF]
//
