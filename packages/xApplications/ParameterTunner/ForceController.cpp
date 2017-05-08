//
// File: ForceController.cpp
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
#include "ForceController.h"

// Exported block signals
real_T force_filter[6];                // '<Root>/2-order TF'
real_T force_error[6];                 // '<Root>/Sum2'

// Exported block parameters
real_T PID_D[6] = { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 } ;// Variable: D
                                                    //  Referenced by: '<S1>/Derivative Gain'


real_T PID_P[6] = { 12.0, 12.0, 12.0, 12.0, 12.0, 12.0 } ;// Variable: P
                                                          //  Referenced by: '<S1>/Proportional Gain'


real_T dead_zone_end[6] = { 1.0, 1.0, 1.0, 1.0, 1.0, 1.0 } ;// Variable: dead_zone_end
                                                            //  Referenced by: '<Root>/Dead Zone'


real_T dead_zone_start[6] = { -1.0, -1.0, -1.0, -1.0, -1.0, -1.0 } ;// Variable: dead_zone_start
                                                                    //  Referenced by: '<Root>/Dead Zone'


real_T filter_den[2] = { 1.0, -0.9 } ; // Variable: filter_den
                                       //  Referenced by: '<Root>/2-order TF'


real_T filter_num[2] = { 0.0, 0.1 } ;  // Variable: filter_num
                                       //  Referenced by: '<Root>/2-order TF'


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
void ForceControllerClass::step(const real_T (&force_ee)[6], const real_T
  (&gravity_ee)[6], real_T (&poseOffset_ee)[6])
{
  int32_T i;
  real_T rtb_RateLimiter;
  real_T rtb_ProportionalGain;
  real_T rtb_TSamp;
  if ((&ForceController_M)->Timing.TaskCounters.TID[1] == 0) {
    for (i = 0; i < 6; i++) {
      // DiscreteTransferFcn: '<Root>/2-order TF'
      force_filter[i] = filter_num[1] * ForceController_DW.uorderTF_states[i];

      // DeadZone: '<Root>/Dead Zone'
      if (force_filter[i] > dead_zone_end[i]) {
        rtb_RateLimiter = force_filter[i] - dead_zone_end[i];
      } else if (force_filter[i] >= dead_zone_start[i]) {
        rtb_RateLimiter = 0.0;
      } else {
        rtb_RateLimiter = force_filter[i] - dead_zone_start[i];
      }

      // End of DeadZone: '<Root>/Dead Zone'

      // Gain: '<S1>/Proportional Gain'
      rtb_ProportionalGain = PID_P[i] * rtb_RateLimiter;

      // Gain: '<S1>/Derivative Gain'
      rtb_RateLimiter *= PID_D[i];

      // SampleTimeMath: '<S3>/TSamp'
      //
      //  About '<S3>/TSamp':
      //   y = u * K where K = 1 / ( w * Ts )

      rtb_TSamp = rtb_RateLimiter * ForceController_P.TSamp_WtEt;

      // Sum: '<S1>/Sum' incorporates:
      //   Delay: '<S3>/UD'
      //   Sum: '<S3>/Diff'

      rtb_RateLimiter = (rtb_TSamp - ForceController_DW.UD_DSTATE[i]) +
        rtb_ProportionalGain;

      // RateLimiter: '<Root>/Rate Limiter'
      rtb_ProportionalGain = rtb_RateLimiter - ForceController_DW.PrevY[i];
      if (rtb_ProportionalGain > ForceController_P.RateLimiter_RisingLim) {
        rtb_RateLimiter = ForceController_DW.PrevY[i] +
          ForceController_P.RateLimiter_RisingLim;
      } else {
        if (rtb_ProportionalGain < ForceController_P.RateLimiter_FallingLim) {
          rtb_RateLimiter = ForceController_DW.PrevY[i] +
            ForceController_P.RateLimiter_FallingLim;
        }
      }

      ForceController_DW.PrevY[i] = rtb_RateLimiter;

      // End of RateLimiter: '<Root>/Rate Limiter'

      // Outport: '<Root>/poseOffset_ee' incorporates:
      //   Gain: '<Root>/Gain'

      ForceController_Y.poseOffset_ee[i] = ForceController_P.Gain_Gain[i] *
        rtb_RateLimiter;

      // Sum: '<Root>/Sum2' incorporates:
      //   Inport: '<Root>/force_ee'
      //   Inport: '<Root>/gravity_ee'

      force_error[i] = force_ee[i] - gravity_ee[i];

      // Update for DiscreteTransferFcn: '<Root>/2-order TF'
      ForceController_DW.uorderTF_states[i] = (force_error[i] - filter_den[1] *
        ForceController_DW.uorderTF_states[i]) / filter_den[0];

      // Update for Delay: '<S3>/UD'
      ForceController_DW.UD_DSTATE[i] = rtb_TSamp;
    }
  }

  rate_scheduler((&ForceController_M));

  // Copy value for root outport '<Root>/poseOffset_ee' since it is accessed globally 
  {
    int32_T i;
    for (i = 0; i < 6; i++)
      poseOffset_ee[i] = ForceController_Y.poseOffset_ee[i];
  }
}

// Model initialize function
void ForceControllerClass::initialize()
{
  // Registration code

  // initialize real-time model
  (void) memset((void *)(&ForceController_M), 0,
                sizeof(RT_MODEL_ForceController_T));

  // block I/O

  // exported global signals
  {
    int32_T i;
    for (i = 0; i < 6; i++) {
      force_filter[i] = 0.0;
    }
  }

  {
    int32_T i;
    for (i = 0; i < 6; i++) {
      force_error[i] = 0.0;
    }
  }

  // states (dwork)
  (void) memset((void *)&ForceController_DW, 0,
                sizeof(DW_ForceController_T));

  {
    int32_T i;
    for (i = 0; i < 6; i++) {
      // InitializeConditions for DiscreteTransferFcn: '<Root>/2-order TF'
      ForceController_DW.uorderTF_states[i] =
        ForceController_P.uorderTF_InitialStates;

      // InitializeConditions for Delay: '<S3>/UD'
      ForceController_DW.UD_DSTATE[i] = ForceController_P.UD_InitialCondition;

      // InitializeConditions for RateLimiter: '<Root>/Rate Limiter'
      ForceController_DW.PrevY[i] = ForceController_P.RateLimiter_IC;
    }
  }
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

    125.0,                             // Computed Parameter: TSamp_WtEt
                                       //  Referenced by: '<S3>/TSamp'

    0.0,                               // Expression: DifferentiatorICPrevScaledInput
                                       //  Referenced by: '<S3>/UD'

    8.0,                               // Computed Parameter: RateLimiter_RisingLim
                                       //  Referenced by: '<Root>/Rate Limiter'

    -8.0,                              // Computed Parameter: RateLimiter_FallingLim
                                       //  Referenced by: '<Root>/Rate Limiter'

    0.0,                               // Expression: 0
                                       //  Referenced by: '<Root>/Rate Limiter'


    //  Expression: [0.0005,0.0005,0.0005,0.01,0.01,0.01]
    //  Referenced by: '<Root>/Gain'

    { 0.0005, 0.0005, 0.0005, 0.01, 0.01, 0.01 },
    1U                                 // Computed Parameter: UD_DelayLength
                                       //  Referenced by: '<S3>/UD'

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
