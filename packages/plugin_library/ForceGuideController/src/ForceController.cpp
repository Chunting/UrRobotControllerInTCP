//
// File: ForceController.cpp
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
#include "ForceController.h"
#include "ForceController_private.h"

// Model step function
void ForceControllerModelClass::step(const real_T (&force)[6], const real_T
  (&currentPose)[6], real_T (&targetPose)[6])
{
  int32_T i;
  real_T rtb_DeadZone;
  real_T rtb_FilterCoefficient;
  real_T u0;

  // Copy value for root inport '<Root>/forceIn' since it is accessed globally
  {
    int32_T i;
    for (i = 0; i < 6; i++)
      ForceController_U.force[i] = force[i];
  }

  for (i = 0; i < 6; i++) {
    // DeadZone: '<Root>/Dead Zone' incorporates:
    //   DiscreteTransferFcn: '<Root>/filter'

    if (ForceController_DW.filter_states[i] > 0.5) {
      rtb_DeadZone = ForceController_DW.filter_states[i] - 0.5;
    } else if (ForceController_DW.filter_states[i] >= -0.5) {
      rtb_DeadZone = 0.0;
    } else {
      rtb_DeadZone = ForceController_DW.filter_states[i] - -0.5;
    }

    // End of DeadZone: '<Root>/Dead Zone'

    // Gain: '<S1>/Filter Coefficient' incorporates:
    //   DiscreteIntegrator: '<S1>/Filter'
    //   Gain: '<S1>/Derivative Gain'
    //   Sum: '<S1>/SumD'

    rtb_FilterCoefficient = (ForceController_P.pid.D * rtb_DeadZone -
      ForceController_DW.Filter_DSTATE[i]) * ForceController_P.pid.N;

    // Sum: '<S1>/Sum' incorporates:
    //   DiscreteIntegrator: '<S1>/Integrator'
    //   Gain: '<S1>/Proportional Gain'

    u0 = (ForceController_P.pid.P * rtb_DeadZone +
          ForceController_DW.Integrator_DSTATE[i]) + rtb_FilterCoefficient;

    // Saturate: '<Root>/Saturation'
    if (u0 > 0.5) {
      u0 = 0.5;
    } else {
      if (u0 < -0.5) {
        u0 = -0.5;
      }
    }

    // End of Saturate: '<Root>/Saturation'

    // Outport: '<Root>/targetPose' incorporates:
    //   Inport: '<Root>/CurrentPose'
    //   Sum: '<Root>/Sum1'

    targetPose[i] = u0 + currentPose[i];

    // Update for DiscreteTransferFcn: '<Root>/filter' incorporates:
    //   Update for Inport: '<Root>/forceIn'

    ForceController_DW.filter_states[i] = ForceController_U.force[i] - 0.5 *
      ForceController_DW.filter_states[i];

    // Update for DiscreteIntegrator: '<S1>/Integrator' incorporates:
    //   Gain: '<S1>/Integral Gain'

    ForceController_DW.Integrator_DSTATE[i] += ForceController_P.pid.I *
      rtb_DeadZone * 0.001;

    // Update for DiscreteIntegrator: '<S1>/Filter'
    ForceController_DW.Filter_DSTATE[i] += 0.001 * rtb_FilterCoefficient;
  }
}

// Model initialize function
void ForceControllerModelClass::initialize()
{
  // Registration code

  // states (dwork)
  (void) memset((void *)&ForceController_DW, 0,
                sizeof(DW_ForceController_T));
}

// Model terminate function
void ForceControllerModelClass::terminate()
{
  // (no terminate code required)
}

// Constructor
ForceControllerModelClass::ForceControllerModelClass()
{
  static const P_ForceController_T ForceController_P_temp = {
    {
      2.0,
      22.0,
      0.05,
      1000.0
    }                                  // Variable: pid
                                       //  Referenced by:
                                       //    '<S1>/Derivative Gain'
                                       //    '<S1>/Filter Coefficient'
                                       //    '<S1>/Integral Gain'
                                       //    '<S1>/Proportional Gain'

  };                                   // Modifiable parameters

  // Initialize tunable parameters
  ForceController_P = ForceController_P_temp;
}

// Destructor
ForceControllerModelClass::~ForceControllerModelClass()
{
  // Currently there is no destructor body generated.
}

//
// File trailer for generated code.
//
// [EOF]
//
