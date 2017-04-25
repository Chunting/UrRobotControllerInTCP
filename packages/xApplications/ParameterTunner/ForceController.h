//
// File: ForceController.h
//
// Code generated for Simulink model 'ForceController'.
//
// Model version                  : 1.157
// Simulink Coder version         : 8.11 (R2016b) 25-Aug-2016
// C/C++ source code generated on : Tue Apr 25 00:35:17 2017
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

// Macros for accessing real-time model data structure

// Forward declaration for rtModel
typedef struct tag_RTM_ForceController_T RT_MODEL_ForceController_T;

// Block states (auto storage) for system '<Root>'
typedef struct {
  real_T Integrator_DSTATE[6];         // '<S1>/Integrator'
  real_T Filter_DSTATE[6];             // '<S1>/Filter'
  real_T PrevY[6];                     // '<Root>/Rate Limiter'
} DW_ForceController_T;

// Constant parameters (auto storage)
typedef struct {
  // Expression: [0.0002,0.0002,0.0002,0.5,0.5,0.5]
  //  Referenced by: '<Root>/Gain'

  real_T Gain_Gain[6];
} ConstP_ForceController_T;

// External outputs (root outports fed by signals with auto storage)
typedef struct {
  real_T poseOffset_ee[6];             // '<Root>/poseOffset_ee'
} ExtY_ForceController_T;

// Real-time Model Data Structure
struct tag_RTM_ForceController_T {
  //
  //  Timing:
  //  The following substructure contains information regarding
  //  the timing information for the model.

  struct {
    struct {
      uint8_T TID[2];
    } TaskCounters;
  } Timing;
};

#ifdef __cplusplus

extern "C" {

#endif

#ifdef __cplusplus

}
#endif

// Constant parameters (auto storage)
extern const ConstP_ForceController_T ForceController_ConstP;

//
//  Exported Global Signals
//
//  Note: Exported global signals are block signals with an exported global
//  storage class designation.  Code generation will declare the memory for
//  these signals and export their symbols.
//

extern real_T force_error[6];          // '<Root>/Dead Zone'

//
//  Exported Global Parameters
//
//  Note: Exported global parameters are tunable parameters with an exported
//  global storage class designation.  Code generation will declare the memory for
//  these parameters and exports their symbols.
//

extern real_T PID_D[6];                // Variable: D
                                       //  Referenced by: '<S1>/Derivative Gain'

extern real_T PID_I[6];                // Variable: I
                                       //  Referenced by: '<S1>/Integral Gain'

extern real_T PID_N[6];                // Variable: N
                                       //  Referenced by: '<S1>/Filter Coefficient'

extern real_T PID_P[6];                // Variable: P
                                       //  Referenced by: '<S1>/Proportional Gain'

extern real_T dead_zone_end[6];        // Variable: dead_zone_end
                                       //  Referenced by: '<Root>/Dead Zone'

extern real_T dead_zone_start[6];      // Variable: dead_zone_start
                                       //  Referenced by: '<Root>/Dead Zone'


// Class declaration for model ForceController
class ForceControllerClass {
  // public data and function members
 public:
  // model initialize function
  void initialize();

  // model step function
  void step(const real_T (&force_ee)[6], const real_T (&gravity_ee)[6], real_T (
             &poseOffset_ee)[6]);

  // model terminate function
  void terminate();

  // Constructor
  ForceControllerClass();

  // Destructor
  ~ForceControllerClass();

  // Real-Time Model get method
  RT_MODEL_ForceController_T * getRTM();

  // protected data and function members
 protected:
  // External outputs
  ExtY_ForceController_T ForceController_Y;

  // private data and function members
 private:
  // Block states
  DW_ForceController_T ForceController_DW;

  // Real-Time Model
  RT_MODEL_ForceController_T ForceController_M;
};

//-
//  These blocks were eliminated from the model due to optimizations:
//
//  Block '<Root>/Rate Transition' : Eliminated since input and output rates are identical


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

#endif                                 // RTW_HEADER_ForceController_h_

//
// File trailer for generated code.
//
// [EOF]
//
