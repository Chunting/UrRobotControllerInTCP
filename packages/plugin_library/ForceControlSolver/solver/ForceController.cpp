//
// File: ForceController.cpp
//
// Code generated for Simulink model 'ForceController'.
//
// Model version                  : 1.135
// Simulink Coder version         : 8.11 (R2016b) 25-Aug-2016
// C/C++ source code generated on : Mon Apr 17 17:49:30 2017
//
// Target selection: ert.tlc
// Embedded hardware selection: 32-bit Generic
// Code generation objectives: Unspecified
// Validation result: Not run
//
#include "ForceController.h"
#include "ForceController_private.h"

// Exported block signals
real_T force_error[6];                 // '<Root>/Sum2'
real_T force_filter[6];                // '<Root>/filter'
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
  int32_T memOffset;
  real_T rtb_DeadZone[6];
  real_T rtb_FilterCoefficient[6];
  int32_T i;
  real_T filter_tmp[6];
  real_T rtb_DeadZone_f;
  real_T rtb_FilterCoefficient_p;
  real_T u0;
  for (i = 0; i < 6; i++) {
    // Sum: '<Root>/Sum2' incorporates:
    //   Inport: '<Root>/force_ee'
    //   Inport: '<Root>/gravity_ee'

    force_error[i] = force_ee[i] - gravity_ee[i];

    // DiscreteTransferFcn: '<Root>/filter'
    memOffset = i << 1;
    rtb_DeadZone_f = ((force_error[i] - ForceController_P.filter_den[1] *
                       ForceController_DW.filter_states[memOffset]) -
                      ForceController_DW.filter_states[memOffset + 1] *
                      ForceController_P.filter_den[2]) /
      ForceController_P.filter_den[0];
    rtb_FilterCoefficient_p = rtb_DeadZone_f;
    rtb_DeadZone_f *= ForceController_P.filter_num[0];
    rtb_DeadZone_f += ForceController_P.filter_num[1] *
      ForceController_DW.filter_states[memOffset];
    rtb_DeadZone_f += ForceController_DW.filter_states[memOffset + 1] *
      ForceController_P.filter_num[2];
    force_filter[i] = rtb_DeadZone_f;
    filter_tmp[i] = rtb_FilterCoefficient_p;
  }

  // RateTransition: '<Root>/Rate Transition' incorporates:
  //   DeadZone: '<Root>/Dead Zone'
  //   Gain: '<S1>/Filter Coefficient'

  if ((&ForceController_M)->Timing.TaskCounters.TID[1] == 0) {
    for (i = 0; i < 6; i++) {
      // DeadZone: '<Root>/Dead Zone'
      if (force_filter[i] > ForceController_P.dead_zone_end[i]) {
        rtb_DeadZone_f = force_filter[i] - ForceController_P.dead_zone_end[i];
      } else if (force_filter[i] >= ForceController_P.dead_zone_start[i]) {
        rtb_DeadZone_f = 0.0;
      } else {
        rtb_DeadZone_f = force_filter[i] - ForceController_P.dead_zone_start[i];
      }

      // Gain: '<S1>/Filter Coefficient' incorporates:
      //   DiscreteIntegrator: '<S1>/Filter'
      //   Gain: '<S1>/Derivative Gain'
      //   Sum: '<S1>/SumD'

      rtb_FilterCoefficient_p = (ForceController_P.D[i] * rtb_DeadZone_f -
        ForceController_DW.Filter_DSTATE[i]) * ForceController_P.N[i];

      // Sum: '<S1>/Sum' incorporates:
      //   DiscreteIntegrator: '<S1>/Integrator'
      //   Gain: '<S1>/Proportional Gain'

      u0 = (ForceController_P.P[i] * rtb_DeadZone_f +
            ForceController_DW.Integrator_DSTATE[i]) + rtb_FilterCoefficient_p;

      // Saturate: '<Root>/Saturation'
      if (u0 > ForceController_P.saturation_upper_limit[i]) {
        // Outport: '<Root>/poseOffset_ee'
        ForceController_Y.poseOffset_ee[i] =
          ForceController_P.saturation_upper_limit[i];
      } else if (u0 < ForceController_P.saturation_lower_limit[i]) {
        // Outport: '<Root>/poseOffset_ee'
        ForceController_Y.poseOffset_ee[i] =
          ForceController_P.saturation_lower_limit[i];
      } else {
        // Outport: '<Root>/poseOffset_ee'
        ForceController_Y.poseOffset_ee[i] = u0;
      }

      // End of Saturate: '<Root>/Saturation'

      // Gain: '<S1>/Integral Gain'
      rtb_DeadZone_f *= ForceController_P.I[i];
      rtb_DeadZone[i] = rtb_DeadZone_f;
      rtb_FilterCoefficient[i] = rtb_FilterCoefficient_p;
    }
  }

  // End of RateTransition: '<Root>/Rate Transition'

  // Update for DiscreteTransferFcn: '<Root>/filter'
  for (i = 0; i < 6; i++) {
    memOffset = i << 1;
    ForceController_DW.filter_states[memOffset - -1] =
      ForceController_DW.filter_states[memOffset];
    ForceController_DW.filter_states[memOffset] = filter_tmp[i];
  }

  // End of Update for DiscreteTransferFcn: '<Root>/filter'
  if ((&ForceController_M)->Timing.TaskCounters.TID[1] == 0) {
    for (i = 0; i < 6; i++) {
      // Update for DiscreteIntegrator: '<S1>/Integrator'
      ForceController_DW.Integrator_DSTATE[i] += 0.008 * rtb_DeadZone[i];

      // Update for DiscreteIntegrator: '<S1>/Filter'
      ForceController_DW.Filter_DSTATE[i] += 0.008 * rtb_FilterCoefficient[i];
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
  (void) memset(((void *) &ForceController_B), 0,
                sizeof(B_ForceController_T));

  // exported global signals
  {
    int32_T i;
    for (i = 0; i < 6; i++) {
      force_error[i] = 0.0;
    }
  }

  {
    int32_T i;
    for (i = 0; i < 6; i++) {
      force_filter[i] = 0.0;
    }
  }

  // states (dwork)
  (void) memset((void *)&ForceController_DW, 0,
                sizeof(DW_ForceController_T));
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
    //  Variable: D
    //  Referenced by: '<S1>/Derivative Gain'

    { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 },

    //  Variable: I
    //  Referenced by: '<S1>/Integral Gain'

    { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 },

    //  Variable: N
    //  Referenced by: '<S1>/Filter Coefficient'

    { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 },

    //  Variable: P
    //  Referenced by: '<S1>/Proportional Gain'

    { 1.0, 1.0, 1.0, 1.0, 1.0, 1.0 },

    //  Variable: dead_zone_end
    //  Referenced by: '<Root>/Dead Zone'

    { 0.5, 0.5, 0.5, 0.5, 0.5, 0.5 },

    //  Variable: dead_zone_start
    //  Referenced by: '<Root>/Dead Zone'

    { -0.5, -0.5, -0.5, -0.5, -0.5, -0.5 },

    //  Variable: filter_den
    //  Referenced by: '<Root>/filter'

    { 1.0, 0.3815584570930084, 0.76551678814900237 },

    //  Variable: filter_num
    //  Referenced by: '<Root>/filter'

    { 0.79519990113706318, 0.1868726045543786, 0.48976439578823106 },

    //  Variable: saturation_lower_limit
    //  Referenced by: '<Root>/Saturation'

    { -100.0, -100.0, -100.0, -100.0, -100.0, -100.0 },

    //  Variable: saturation_upper_limit
    //  Referenced by: '<Root>/Saturation'

    { 100.0, 100.0, 100.0, 100.0, 100.0, 100.0 }
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
