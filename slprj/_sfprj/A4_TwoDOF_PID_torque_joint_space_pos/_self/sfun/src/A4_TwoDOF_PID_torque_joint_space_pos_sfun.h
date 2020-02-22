#ifndef __A4_TwoDOF_PID_torque_joint_space_pos_sfun_h__
#define __A4_TwoDOF_PID_torque_joint_space_pos_sfun_h__

/* Include files */
#define S_FUNCTION_NAME                sf_sfun
#include "sf_runtime/sfc_sf.h"
#include "sf_runtime/sfc_mex.h"
#include "rtwtypes.h"
#include "multiword_types.h"
#include "sf_runtime/sfcdebug.h"
#define rtInf                          (mxGetInf())
#define rtMinusInf                     (-(mxGetInf()))
#define rtNaN                          (mxGetNaN())
#define rtIsNaN(X)                     ((int)mxIsNaN(X))
#define rtIsInf(X)                     ((int)mxIsInf(X))

struct SfDebugInstanceStruct;
extern struct SfDebugInstanceStruct* sfGlobalDebugInstanceStruct;

/* Type Definitions */

/* Named Constants */

/* Variable Declarations */
extern uint32_T _A4_TwoDOF_PID_torque_joint_space_posMachineNumber_;

/* Variable Definitions */

/* Function Declarations */
extern void A4_TwoDOF_PID_torque_joint_space_pos_initializer(void);
extern void A4_TwoDOF_PID_torque_joint_space_pos_terminator(void);

/* Function Definitions */

/* We load infoStruct for rtw_optimation_info on demand in mdlSetWorkWidths and
   free it immediately in mdlStart. Given that this is machine-wide as
   opposed to chart specific, we use NULL check to make sure it gets loaded
   and unloaded once per machine even though the  methods mdlSetWorkWidths/mdlStart
   are chart/instance specific. The following methods abstract this out. */
extern mxArray* load_A4_TwoDOF_PID_torque_joint_space_pos_optimization_info(void);
extern void unload_A4_TwoDOF_PID_torque_joint_space_pos_optimization_info(void);

#endif
