//
// File: ForceController.h
//
// Code generated for Simulink model 'ForceController'.
//
// Model version                  : 1.195
// Simulink Coder version         : 8.11 (R2016b) 25-Aug-2016
// C/C++ source code generated on : Mon May 08 08:07:10 2017
//
// Target selection: ert.tlc
// Embedded hardware selection: 32-bit Generic
// Code generation objectives: Unspecified
// Validation result: Not run
//
#ifndef RTW_HEADER_ForceController_h_
#define RTW_HEADER_ForceController_h_
#include "rtwtypes.h"
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
  real_T uorderTF_states[6];           // '<Root>/2-order TF'
  real_T UD_DSTATE[6];                 // '<S3>/UD'
  real_T PrevY[6];                     // '<Root>/Rate Limiter'
} DW_ForceController_T;

// External outputs (root outports fed by signals with auto storage)
typedef struct {
  real_T poseOffset_ee[6];             // '<Root>/poseOffset_ee'
} ExtY_ForceController_T;

// Parameters (auto storage)
struct P_ForceController_T_ {
  real_T uorderTF_InitialStates;       // Expression: 0
                                       //  Referenced by: '<Root>/2-order TF'

  real_T TSamp_WtEt;                   // Computed Parameter: TSamp_WtEt
                                       //  Referenced by: '<S3>/TSamp'

  real_T UD_InitialCondition;          // Expression: DifferentiatorICPrevScaledInput
                                       //  Referenced by: '<S3>/UD'

  real_T RateLimiter_RisingLim;        // Computed Parameter: RateLimiter_RisingLim
                                       //  Referenced by: '<Root>/Rate Limiter'

  real_T RateLimiter_FallingLim;       // Computed Parameter: RateLimiter_FallingLim
                                       //  Referenced by: '<Root>/Rate Limiter'

  real_T RateLimiter_IC;               // Expression: 0
                                       //  Referenced by: '<Root>/Rate Limiter'

  real_T Gain_Gain[6];                 // Expression: [0.0005,0.0005,0.0005,0.01,0.01,0.01]
                                       //  Referenced by: '<Root>/Gain'

  uint32_T UD_DelayLength;             // Computed Parameter: UD_DelayLength
                                       //  Referenced by: '<S3>/UD'

};

// Parameters (auto storage)
typedef struct P_ForceController_T_ P_ForceController_T;

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

//
//  Exported Global Signals
//
//  Note: Exported global signals are block signals with an exported global
//  storage class designation.  Code generation will declare the memory for
//  these signals and export their symbols.
//

extern real_T force_filter[6];         // '<Root>/2-order TF'
extern real_T force_error[6];          // '<Root>/Sum2'

//
//  Exported Global Parameters
//
//  Note: Exported global parameters are tunable parameters with an exported
//  global storage class designation.  Code generation will declare the memory for
//  these parameters and exports their symbols.
//

extern real_T PID_D[6];                // Variable: D
                                       //  Referenced by: '<S1>/Derivative Gain'

extern real_T PID_P[6];                // Variable: P
                                       //  Referenced by: '<S1>/Proportional Gain'

extern real_T dead_zone_end[6];        // Variable: dead_zone_end
                                       //  Referenced by: '<Root>/Dead Zone'

extern real_T dead_zone_start[6];      // Variable: dead_zone_start
                                       //  Referenced by: '<Root>/Dead Zone'

extern real_T filter_den[2];           // Variable: filter_den
                                       //  Referenced by: '<Root>/2-order TF'

extern real_T filter_num[2];           // Variable: filter_num
                                       //  Referenced by: '<Root>/2-order TF'


// Class declaration for model ForceController
class ForceControllerClass {
  // public data and function members
 public:
  // Tunable parameters
  P_ForceController_T ForceController_P;

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

  // Block parameters get method
  const P_ForceController_T & getBlockParameters() const
  {
    return ForceController_P;
  }

  // Block parameters set method
  void setBlockParameters(const P_ForceController_T *pForceController_P)
  {
    ForceController_P = *pForceController_P;
  }

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
//  Block '<S3>/DTDup' : Unused code path elimination
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
//  '<S2>'   : 'ForceController/LPF1'
//  '<S3>'   : 'ForceController/Discrete PID Controller/Differentiator'

#endif                                 // RTW_HEADER_ForceController_h_

//
// File trailer for generated code.
//
// [EOF]
//
