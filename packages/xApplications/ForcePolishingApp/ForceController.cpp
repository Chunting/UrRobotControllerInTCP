//
// File: ForceController.cpp
//
// Code generated for Simulink model 'ForceController'.
//
// Model version                  : 1.207
// Simulink Coder version         : 8.12 (R2017a) 16-Feb-2017
// C/C++ source code generated on : Thu May 11 12:28:57 2017
//
// Target selection: ert.tlc
// Embedded hardware selection: 32-bit Generic
// Code generation objectives: Unspecified
// Validation result: Not run
//
#include "ForceController.h"

// Exported block signals
real_T force_filter;                   // '<Root>/2-order TF'
real_T CV;                             // '<Root>/Saturation'

// Exported block parameters
real_T PID_D = 0.0;                    // Variable: D
                                       //  Referenced by: '<S1>/Derivative Gain'

real_T PID_I = 1.0;                    // Variable: I
                                       //  Referenced by: '<S1>/Integral Gain'

real_T K_lamuda = 0.02;                // Variable: K_lamuda
                                       //  Referenced by: '<Root>/Gain1'

real_T PID_N = 0.0;                    // Variable: N
                                       //  Referenced by: '<S1>/Filter Coefficient'

real_T PID_P = 10.0;                   // Variable: P
                                       //  Referenced by: '<S1>/Proportional Gain'

real_T dead_zone_end = 1.0;            // Variable: dead_zone_end
                                       //  Referenced by: '<Root>/Dead Zone'

real_T dead_zone_start = -1.0;         // Variable: dead_zone_start
                                       //  Referenced by: '<Root>/Dead Zone'

real_T filter_den[2] = { 1.0, -0.9 } ; // Variable: filter_den
                                       //  Referenced by: '<Root>/2-order TF'


real_T filter_num[2] = { 0.0, 0.1 } ;  // Variable: filter_num
                                       //  Referenced by: '<Root>/2-order TF'


real_T saturation_lower_limit = -100.0;// Variable: saturation_lower_limit
                                       //  Referenced by: '<Root>/Saturation'

real_T saturation_upper_limit = 100.0; // Variable: saturation_upper_limit
                                       //  Referenced by: '<Root>/Saturation'

static void rate_scheduler(RT_MODEL_ForceController_T *const ForceController_M);

//
//   This function updates active task flag for each subrate.
// The function is called at model base rate, hence the
// generated code self-manages all its subrates.
//
static void rate_scheduler(RT_MODEL_ForceController_T *const ForceController_M)
{
  // Compute which subrates run during the next base time step.  Subrates
  //  are an integer multiple of the base rate counter.  Therefore, the subtask
  //  counter is reset when it reaches its limit (zero means run).

  (ForceController_M->Timing.TaskCounters.TID[1])++;
  if ((ForceController_M->Timing.TaskCounters.TID[1]) > 3) {// Sample time: [0.008s, 0.0s] 
    ForceController_M->Timing.TaskCounters.TID[1] = 0;
  }
}

// Model step function
void ForceControllerClass::step(const real_T fz_touch, real_T &dis_offset)
{
  real_T rtb_FilterCoefficient;
  real_T rtb_RateLimiter;
  real_T rtb_Sum;
  real_T Sum3;

  // Copy value for root inport '<Root>/Fz_touch' since it is accessed globally
  ForceController_U.Fz_touch = fz_touch;

  // DiscreteTransferFcn: '<Root>/2-order TF'
  force_filter = filter_num[1] * ForceController_DW.uorderTF_states;

  // DeadZone: '<Root>/Dead Zone'
  if (force_filter > dead_zone_end) {
    Sum3 = force_filter - dead_zone_end;
  } else if (force_filter >= dead_zone_start) {
    Sum3 = 0.0;
  } else {
    Sum3 = force_filter - dead_zone_start;
  }

  // End of DeadZone: '<Root>/Dead Zone'

  // Sum: '<Root>/Sum3' incorporates:
  //   Memory: '<Root>/Memory'

  Sum3 -= ForceController_B.Gain1;
  if ((&ForceController_M)->Timing.TaskCounters.TID[1] == 0) {
    // Gain: '<S1>/Filter Coefficient' incorporates:
    //   DiscreteIntegrator: '<S1>/Filter'
    //   Gain: '<S1>/Derivative Gain'
    //   Sum: '<S1>/SumD'

    rtb_FilterCoefficient = (PID_D * Sum3 - ForceController_DW.Filter_DSTATE) *
      PID_N;

    // Sum: '<S1>/Sum' incorporates:
    //   DiscreteIntegrator: '<S1>/Integrator'
    //   Gain: '<S1>/Proportional Gain'

    rtb_Sum = (PID_P * Sum3 + ForceController_DW.Integrator_DSTATE) +
      rtb_FilterCoefficient;

    // RateLimiter: '<Root>/Rate Limiter'
    rtb_RateLimiter = rtb_Sum - ForceController_DW.PrevY;
    if (rtb_RateLimiter > ForceController_P.RateLimiter_RisingLim) {
      rtb_RateLimiter = ForceController_DW.PrevY +
        ForceController_P.RateLimiter_RisingLim;
    } else if (rtb_RateLimiter < ForceController_P.RateLimiter_FallingLim) {
      rtb_RateLimiter = ForceController_DW.PrevY +
        ForceController_P.RateLimiter_FallingLim;
    } else {
      rtb_RateLimiter = rtb_Sum;
    }

    ForceController_DW.PrevY = rtb_RateLimiter;

    // End of RateLimiter: '<Root>/Rate Limiter'

    // Saturate: '<Root>/Saturation'
    if (rtb_RateLimiter > saturation_upper_limit) {
      CV = saturation_upper_limit;
    } else if (rtb_RateLimiter < saturation_lower_limit) {
      CV = saturation_lower_limit;
    } else {
      CV = rtb_RateLimiter;
    }

    // End of Saturate: '<Root>/Saturation'

    // Outport: '<Root>/disOffset' incorporates:
    //   Gain: '<Root>/Gain'

    ForceController_Y.disOffset = ForceController_P.Gain_Gain * CV;

    // Gain: '<S1>/Integral Gain'
    rtb_RateLimiter = PID_I * Sum3;

    // Gain: '<Root>/Gain1'
    ForceController_B.Gain1 = K_lamuda * rtb_Sum;

    // Update for DiscreteIntegrator: '<S1>/Integrator'
    ForceController_DW.Integrator_DSTATE += ForceController_P.Integrator_gainval
      * rtb_RateLimiter;

    // Update for DiscreteIntegrator: '<S1>/Filter'
    ForceController_DW.Filter_DSTATE += ForceController_P.Filter_gainval *
      rtb_FilterCoefficient;
  }

  // Update for DiscreteTransferFcn: '<Root>/2-order TF' incorporates:
  //   Update for Inport: '<Root>/Fz_touch'

  ForceController_DW.uorderTF_states = (ForceController_U.Fz_touch - filter_den
    [1] * ForceController_DW.uorderTF_states) / filter_den[0];
  rate_scheduler((&ForceController_M));

  // Copy value for root outport '<Root>/disOffset' since it is accessed globally 
  dis_offset = ForceController_Y.disOffset;
}

// Model initialize function
void ForceControllerClass::initialize()
{
  // Registration code

  // initialize real-time model
  (void) memset((void *)(&ForceController_M), 0,
                sizeof(RT_MODEL_ForceController_T));

  // block I/O
  (void) memset(((void *) &ForceController_B), 0,
                sizeof(B_ForceController_T));

  // exported global signals
  force_filter = 0.0;
  CV = 0.0;

  // states (dwork)
  (void) memset((void *)&ForceController_DW, 0,
                sizeof(DW_ForceController_T));

  // InitializeConditions for DiscreteTransferFcn: '<Root>/2-order TF'
  ForceController_DW.uorderTF_states = ForceController_P.uorderTF_InitialStates;

  // InitializeConditions for Memory: '<Root>/Memory'
  ForceController_B.Gain1 = ForceController_P.Memory_X0;

  // InitializeConditions for DiscreteIntegrator: '<S1>/Integrator'
  ForceController_DW.Integrator_DSTATE = ForceController_P.Integrator_IC;

  // InitializeConditions for DiscreteIntegrator: '<S1>/Filter'
  ForceController_DW.Filter_DSTATE = ForceController_P.Filter_IC;

  // InitializeConditions for RateLimiter: '<Root>/Rate Limiter'
  ForceController_DW.PrevY = ForceController_P.RateLimiter_IC;
}

// Model terminate function
void ForceControllerClass::terminate()
{
  // (no terminate code required)
}

// Constructor
ForceControllerClass::ForceControllerClass()
{
  static const P_ForceController_T ForceController_P_temp = {
    0.0,                               // Expression: 0
                                       //  Referenced by: '<Root>/2-order TF'

    0.0,                               // Expression: 0
                                       //  Referenced by: '<Root>/Memory'

    0.008,                             // Computed Parameter: Integrator_gainval
                                       //  Referenced by: '<S1>/Integrator'

    0.0,                               // Expression: InitialConditionForIntegrator
                                       //  Referenced by: '<S1>/Integrator'

    0.008,                             // Computed Parameter: Filter_gainval
                                       //  Referenced by: '<S1>/Filter'

    0.0,                               // Expression: InitialConditionForFilter
                                       //  Referenced by: '<S1>/Filter'

    8.0,                               // Computed Parameter: RateLimiter_RisingLim
                                       //  Referenced by: '<Root>/Rate Limiter'

    -8.0,                              // Computed Parameter: RateLimiter_FallingLim
                                       //  Referenced by: '<Root>/Rate Limiter'

    0.0,                               // Expression: 0
                                       //  Referenced by: '<Root>/Rate Limiter'

    0.005                              // Expression: 0.005
                                       //  Referenced by: '<Root>/Gain'

  };                                   // Modifiable parameters

  // Initialize tunable parameters
  ForceController_P = ForceController_P_temp;
}

// Destructor
ForceControllerClass::~ForceControllerClass()
{
  // Currently there is no destructor body generated.
}

// Real-Time Model get method
RT_MODEL_ForceController_T * ForceControllerClass::getRTM()
{
  return (&ForceController_M);
}

//
// File trailer for generated code.
//
// [EOF]
//
