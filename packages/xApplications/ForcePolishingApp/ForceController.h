//
// File: ForceController.h
//
// Code generated for Simulink model 'ForceController'.
//
// Model version                  : 1.205
// Simulink Coder version         : 8.12 (R2017a) 16-Feb-2017
// C/C++ source code generated on : Thu May 11 12:06:56 2017
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

// Block signals (auto storage)
typedef struct {
  real_T Gain1;                        // '<Root>/Gain1'
} B_ForceController_T;

// Block states (auto storage) for system '<Root>'
typedef struct {
  real_T uorderTF_states;              // '<Root>/2-order TF'
  real_T Integrator_DSTATE;            // '<S1>/Integrator'
  real_T Filter_DSTATE;                // '<S1>/Filter'
  real_T PrevY;                        // '<Root>/Rate Limiter'
} DW_ForceController_T;

// External inputs (root inport signals with auto storage)
typedef struct {
  real_T Fz_touch;                     // '<Root>/Fz_touch'
} ExtU_ForceController_T;

// External outputs (root outports fed by signals with auto storage)
typedef struct {
  real_T disOffset[6];                 // '<Root>/disOffset'
} ExtY_ForceController_T;

// Parameters (auto storage)
struct P_ForceController_T_ {
  real_T uorderTF_InitialStates;       // Expression: 0
                                       //  Referenced by: '<Root>/2-order TF'

  real_T Memory_X0;                    // Expression: 0
                                       //  Referenced by: '<Root>/Memory'

  real_T Integrator_gainval;           // Computed Parameter: Integrator_gainval
                                       //  Referenced by: '<S1>/Integrator'

  real_T Integrator_IC;                // Expression: InitialConditionForIntegrator
                                       //  Referenced by: '<S1>/Integrator'

  real_T Filter_gainval;               // Computed Parameter: Filter_gainval
                                       //  Referenced by: '<S1>/Filter'

  real_T Filter_IC;                    // Expression: InitialConditionForFilter
                                       //  Referenced by: '<S1>/Filter'

  real_T RateLimiter_RisingLim;        // Computed Parameter: RateLimiter_RisingLim
                                       //  Referenced by: '<Root>/Rate Limiter'

  real_T RateLimiter_FallingLim;       // Computed Parameter: RateLimiter_FallingLim
                                       //  Referenced by: '<Root>/Rate Limiter'

  real_T RateLimiter_IC;               // Expression: 0
                                       //  Referenced by: '<Root>/Rate Limiter'

  real_T Gain_Gain[6];                 // Expression: [0.0005,0.0005,0.0005,0.01,0.01,0.01]
                                       //  Referenced by: '<Root>/Gain'

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

extern real_T force_filter;            // '<Root>/2-order TF'
extern real_T CV;                      // '<Root>/Saturation'

//
//  Exported Global Parameters
//
//  Note: Exported global parameters are tunable parameters with an exported
//  global storage class designation.  Code generation will declare the memory for
//  these parameters and exports their symbols.
//

extern real_T PID_D;                   // Variable: D
                                       //  Referenced by: '<S1>/Derivative Gain'

extern real_T PID_I;                   // Variable: I
                                       //  Referenced by: '<S1>/Integral Gain'

extern real_T K_lamuda;                // Variable: K_lamuda
                                       //  Referenced by: '<Root>/Gain1'

extern real_T PID_N;                   // Variable: N
                                       //  Referenced by: '<S1>/Filter Coefficient'

extern real_T PID_P;                   // Variable: P
                                       //  Referenced by: '<S1>/Proportional Gain'

extern real_T dead_zone_end;           // Variable: dead_zone_end
                                       //  Referenced by: '<Root>/Dead Zone'

extern real_T dead_zone_start;         // Variable: dead_zone_start
                                       //  Referenced by: '<Root>/Dead Zone'

extern real_T filter_den[2];           // Variable: filter_den
                                       //  Referenced by: '<Root>/2-order TF'

extern real_T filter_num[2];           // Variable: filter_num
                                       //  Referenced by: '<Root>/2-order TF'

extern real_T saturation_lower_limit;  // Variable: saturation_lower_limit
                                       //  Referenced by: '<Root>/Saturation'

extern real_T saturation_upper_limit;  // Variable: saturation_upper_limit
                                       //  Referenced by: '<Root>/Saturation'


// Class declaration for model ForceController
class ForceControllerClass {
  // public data and function members
 public:
  // Tunable parameters
  P_ForceController_T ForceController_P;

  // model initialize function
  void initialize();

  // model step function
  void step(const real_T fz_touch, real_T (&dis_offset)[6]);

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
  // External inputs
  ExtU_ForceController_T ForceController_U;

  // External outputs
  ExtY_ForceController_T ForceController_Y;

  // private data and function members
 private:
  // Block signals
  B_ForceController_T ForceController_B;

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
//  '<S2>'   : 'ForceController/LPF1'
//  '<S3>'   : 'ForceController/Rate Limiter Dynamic'

#endif                                 // RTW_HEADER_ForceController_h_

//
// File trailer for generated code.
//
// [EOF]
//
