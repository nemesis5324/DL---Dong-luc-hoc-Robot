/* Include files */

#include "A3_TwoDOF_PID_torque_joint_space_sfun.h"
#include "A3_TwoDOF_PID_torque_joint_space_sfun_debug_macros.h"
#include "c3_A3_TwoDOF_PID_torque_joint_space.h"
#include "c4_A3_TwoDOF_PID_torque_joint_space.h"

/* Type Definitions */

/* Named Constants */

/* Variable Declarations */

/* Variable Definitions */
uint32_T _A3_TwoDOF_PID_torque_joint_spaceMachineNumber_;

/* Function Declarations */

/* Function Definitions */
void A3_TwoDOF_PID_torque_joint_space_initializer(void)
{
}

void A3_TwoDOF_PID_torque_joint_space_terminator(void)
{
}

/* SFunction Glue Code */
unsigned int sf_A3_TwoDOF_PID_torque_joint_space_method_dispatcher(SimStruct
  *simstructPtr, unsigned int chartFileNumber, const char* specsCksum, int_T
  method, void *data)
{
  if (chartFileNumber==3) {
    c3_A3_TwoDOF_PID_torque_joint_space_method_dispatcher(simstructPtr, method,
      data);
    return 1;
  }

  if (chartFileNumber==4) {
    c4_A3_TwoDOF_PID_torque_joint_space_method_dispatcher(simstructPtr, method,
      data);
    return 1;
  }

  return 0;
}

unsigned int sf_A3_TwoDOF_PID_torque_joint_space_process_check_sum_call( int
  nlhs, mxArray * plhs[], int nrhs, const mxArray * prhs[] )
{

#ifdef MATLAB_MEX_FILE

  char commandName[20];
  if (nrhs<1 || !mxIsChar(prhs[0]) )
    return 0;

  /* Possible call to get the checksum */
  mxGetString(prhs[0], commandName,sizeof(commandName)/sizeof(char));
  commandName[(sizeof(commandName)/sizeof(char)-1)] = '\0';
  if (strcmp(commandName,"sf_get_check_sum"))
    return 0;
  plhs[0] = mxCreateDoubleMatrix( 1,4,mxREAL);
  if (nrhs>1 && mxIsChar(prhs[1])) {
    mxGetString(prhs[1], commandName,sizeof(commandName)/sizeof(char));
    commandName[(sizeof(commandName)/sizeof(char)-1)] = '\0';
    if (!strcmp(commandName,"machine")) {
      ((real_T *)mxGetPr((plhs[0])))[0] = (real_T)(998940455U);
      ((real_T *)mxGetPr((plhs[0])))[1] = (real_T)(832406817U);
      ((real_T *)mxGetPr((plhs[0])))[2] = (real_T)(1267829654U);
      ((real_T *)mxGetPr((plhs[0])))[3] = (real_T)(2624606725U);
    } else if (!strcmp(commandName,"exportedFcn")) {
      ((real_T *)mxGetPr((plhs[0])))[0] = (real_T)(0U);
      ((real_T *)mxGetPr((plhs[0])))[1] = (real_T)(0U);
      ((real_T *)mxGetPr((plhs[0])))[2] = (real_T)(0U);
      ((real_T *)mxGetPr((plhs[0])))[3] = (real_T)(0U);
    } else if (!strcmp(commandName,"makefile")) {
      ((real_T *)mxGetPr((plhs[0])))[0] = (real_T)(1634458114U);
      ((real_T *)mxGetPr((plhs[0])))[1] = (real_T)(1007993977U);
      ((real_T *)mxGetPr((plhs[0])))[2] = (real_T)(244825291U);
      ((real_T *)mxGetPr((plhs[0])))[3] = (real_T)(2335962567U);
    } else if (nrhs==3 && !strcmp(commandName,"chart")) {
      unsigned int chartFileNumber;
      chartFileNumber = (unsigned int)mxGetScalar(prhs[2]);
      switch (chartFileNumber) {
       case 3:
        {
          extern void sf_c3_A3_TwoDOF_PID_torque_joint_space_get_check_sum
            (mxArray *plhs[]);
          sf_c3_A3_TwoDOF_PID_torque_joint_space_get_check_sum(plhs);
          break;
        }

       case 4:
        {
          extern void sf_c4_A3_TwoDOF_PID_torque_joint_space_get_check_sum
            (mxArray *plhs[]);
          sf_c4_A3_TwoDOF_PID_torque_joint_space_get_check_sum(plhs);
          break;
        }

       default:
        ((real_T *)mxGetPr((plhs[0])))[0] = (real_T)(0.0);
        ((real_T *)mxGetPr((plhs[0])))[1] = (real_T)(0.0);
        ((real_T *)mxGetPr((plhs[0])))[2] = (real_T)(0.0);
        ((real_T *)mxGetPr((plhs[0])))[3] = (real_T)(0.0);
      }
    } else if (!strcmp(commandName,"target")) {
      ((real_T *)mxGetPr((plhs[0])))[0] = (real_T)(3031367619U);
      ((real_T *)mxGetPr((plhs[0])))[1] = (real_T)(4001028638U);
      ((real_T *)mxGetPr((plhs[0])))[2] = (real_T)(3978939492U);
      ((real_T *)mxGetPr((plhs[0])))[3] = (real_T)(838979348U);
    } else {
      return 0;
    }
  } else {
    ((real_T *)mxGetPr((plhs[0])))[0] = (real_T)(606797614U);
    ((real_T *)mxGetPr((plhs[0])))[1] = (real_T)(3354328912U);
    ((real_T *)mxGetPr((plhs[0])))[2] = (real_T)(503589600U);
    ((real_T *)mxGetPr((plhs[0])))[3] = (real_T)(3595514469U);
  }

  return 1;

#else

  return 0;

#endif

}

unsigned int sf_A3_TwoDOF_PID_torque_joint_space_autoinheritance_info( int nlhs,
  mxArray * plhs[], int nrhs, const mxArray * prhs[] )
{

#ifdef MATLAB_MEX_FILE

  char commandName[32];
  char aiChksum[64];
  if (nrhs<3 || !mxIsChar(prhs[0]) )
    return 0;

  /* Possible call to get the autoinheritance_info */
  mxGetString(prhs[0], commandName,sizeof(commandName)/sizeof(char));
  commandName[(sizeof(commandName)/sizeof(char)-1)] = '\0';
  if (strcmp(commandName,"get_autoinheritance_info"))
    return 0;
  mxGetString(prhs[2], aiChksum,sizeof(aiChksum)/sizeof(char));
  aiChksum[(sizeof(aiChksum)/sizeof(char)-1)] = '\0';

  {
    unsigned int chartFileNumber;
    chartFileNumber = (unsigned int)mxGetScalar(prhs[1]);
    switch (chartFileNumber) {
     case 3:
      {
        if (strcmp(aiChksum, "IyAUguwxPcT8LOralLfKFH") == 0) {
          extern mxArray
            *sf_c3_A3_TwoDOF_PID_torque_joint_space_get_autoinheritance_info
            (void);
          plhs[0] =
            sf_c3_A3_TwoDOF_PID_torque_joint_space_get_autoinheritance_info();
          break;
        }

        plhs[0] = mxCreateDoubleMatrix(0,0,mxREAL);
        break;
      }

     case 4:
      {
        if (strcmp(aiChksum, "WXsyV0vsOas448tqAlkFGE") == 0) {
          extern mxArray
            *sf_c4_A3_TwoDOF_PID_torque_joint_space_get_autoinheritance_info
            (void);
          plhs[0] =
            sf_c4_A3_TwoDOF_PID_torque_joint_space_get_autoinheritance_info();
          break;
        }

        plhs[0] = mxCreateDoubleMatrix(0,0,mxREAL);
        break;
      }

     default:
      plhs[0] = mxCreateDoubleMatrix(0,0,mxREAL);
    }
  }

  return 1;

#else

  return 0;

#endif

}

unsigned int sf_A3_TwoDOF_PID_torque_joint_space_get_eml_resolved_functions_info
  ( int nlhs, mxArray * plhs[], int nrhs, const mxArray * prhs[] )
{

#ifdef MATLAB_MEX_FILE

  char commandName[64];
  if (nrhs<2 || !mxIsChar(prhs[0]))
    return 0;

  /* Possible call to get the get_eml_resolved_functions_info */
  mxGetString(prhs[0], commandName,sizeof(commandName)/sizeof(char));
  commandName[(sizeof(commandName)/sizeof(char)-1)] = '\0';
  if (strcmp(commandName,"get_eml_resolved_functions_info"))
    return 0;

  {
    unsigned int chartFileNumber;
    chartFileNumber = (unsigned int)mxGetScalar(prhs[1]);
    switch (chartFileNumber) {
     case 3:
      {
        extern const mxArray
          *sf_c3_A3_TwoDOF_PID_torque_joint_space_get_eml_resolved_functions_info
          (void);
        mxArray *persistentMxArray = (mxArray *)
          sf_c3_A3_TwoDOF_PID_torque_joint_space_get_eml_resolved_functions_info
          ();
        plhs[0] = mxDuplicateArray(persistentMxArray);
        mxDestroyArray(persistentMxArray);
        break;
      }

     case 4:
      {
        extern const mxArray
          *sf_c4_A3_TwoDOF_PID_torque_joint_space_get_eml_resolved_functions_info
          (void);
        mxArray *persistentMxArray = (mxArray *)
          sf_c4_A3_TwoDOF_PID_torque_joint_space_get_eml_resolved_functions_info
          ();
        plhs[0] = mxDuplicateArray(persistentMxArray);
        mxDestroyArray(persistentMxArray);
        break;
      }

     default:
      plhs[0] = mxCreateDoubleMatrix(0,0,mxREAL);
    }
  }

  return 1;

#else

  return 0;

#endif

}

unsigned int sf_A3_TwoDOF_PID_torque_joint_space_third_party_uses_info( int nlhs,
  mxArray * plhs[], int nrhs, const mxArray * prhs[] )
{
  char commandName[64];
  char tpChksum[64];
  if (nrhs<3 || !mxIsChar(prhs[0]))
    return 0;

  /* Possible call to get the third_party_uses_info */
  mxGetString(prhs[0], commandName,sizeof(commandName)/sizeof(char));
  commandName[(sizeof(commandName)/sizeof(char)-1)] = '\0';
  mxGetString(prhs[2], tpChksum,sizeof(tpChksum)/sizeof(char));
  tpChksum[(sizeof(tpChksum)/sizeof(char)-1)] = '\0';
  if (strcmp(commandName,"get_third_party_uses_info"))
    return 0;

  {
    unsigned int chartFileNumber;
    chartFileNumber = (unsigned int)mxGetScalar(prhs[1]);
    switch (chartFileNumber) {
     case 3:
      {
        if (strcmp(tpChksum, "4c9Iu3XWf4Mlf5egnrOAbC") == 0) {
          extern mxArray
            *sf_c3_A3_TwoDOF_PID_torque_joint_space_third_party_uses_info(void);
          plhs[0] = sf_c3_A3_TwoDOF_PID_torque_joint_space_third_party_uses_info
            ();
          break;
        }
      }

     case 4:
      {
        if (strcmp(tpChksum, "dEmdhQnf3UfFB6JKxBxGV") == 0) {
          extern mxArray
            *sf_c4_A3_TwoDOF_PID_torque_joint_space_third_party_uses_info(void);
          plhs[0] = sf_c4_A3_TwoDOF_PID_torque_joint_space_third_party_uses_info
            ();
          break;
        }
      }

     default:
      plhs[0] = mxCreateDoubleMatrix(0,0,mxREAL);
    }
  }

  return 1;
}

unsigned int sf_A3_TwoDOF_PID_torque_joint_space_updateBuildInfo_args_info( int
  nlhs, mxArray * plhs[], int nrhs, const mxArray * prhs[] )
{
  char commandName[64];
  char tpChksum[64];
  if (nrhs<3 || !mxIsChar(prhs[0]))
    return 0;

  /* Possible call to get the updateBuildInfo_args_info */
  mxGetString(prhs[0], commandName,sizeof(commandName)/sizeof(char));
  commandName[(sizeof(commandName)/sizeof(char)-1)] = '\0';
  mxGetString(prhs[2], tpChksum,sizeof(tpChksum)/sizeof(char));
  tpChksum[(sizeof(tpChksum)/sizeof(char)-1)] = '\0';
  if (strcmp(commandName,"get_updateBuildInfo_args_info"))
    return 0;

  {
    unsigned int chartFileNumber;
    chartFileNumber = (unsigned int)mxGetScalar(prhs[1]);
    switch (chartFileNumber) {
     case 3:
      {
        if (strcmp(tpChksum, "4c9Iu3XWf4Mlf5egnrOAbC") == 0) {
          extern mxArray
            *sf_c3_A3_TwoDOF_PID_torque_joint_space_updateBuildInfo_args_info
            (void);
          plhs[0] =
            sf_c3_A3_TwoDOF_PID_torque_joint_space_updateBuildInfo_args_info();
          break;
        }
      }

     case 4:
      {
        if (strcmp(tpChksum, "dEmdhQnf3UfFB6JKxBxGV") == 0) {
          extern mxArray
            *sf_c4_A3_TwoDOF_PID_torque_joint_space_updateBuildInfo_args_info
            (void);
          plhs[0] =
            sf_c4_A3_TwoDOF_PID_torque_joint_space_updateBuildInfo_args_info();
          break;
        }
      }

     default:
      plhs[0] = mxCreateDoubleMatrix(0,0,mxREAL);
    }
  }

  return 1;
}

void A3_TwoDOF_PID_torque_joint_space_debug_initialize(struct
  SfDebugInstanceStruct* debugInstance)
{
  _A3_TwoDOF_PID_torque_joint_spaceMachineNumber_ = sf_debug_initialize_machine
    (debugInstance,"A3_TwoDOF_PID_torque_joint_space","sfun",0,2,0,0,0);
  sf_debug_set_machine_event_thresholds(debugInstance,
    _A3_TwoDOF_PID_torque_joint_spaceMachineNumber_,0,0);
  sf_debug_set_machine_data_thresholds(debugInstance,
    _A3_TwoDOF_PID_torque_joint_spaceMachineNumber_,0);
}

void A3_TwoDOF_PID_torque_joint_space_register_exported_symbols(SimStruct* S)
{
}

static mxArray* sRtwOptimizationInfoStruct= NULL;
mxArray* load_A3_TwoDOF_PID_torque_joint_space_optimization_info(void)
{
  if (sRtwOptimizationInfoStruct==NULL) {
    sRtwOptimizationInfoStruct = sf_load_rtw_optimization_info(
      "A3_TwoDOF_PID_torque_joint_space", "A3_TwoDOF_PID_torque_joint_space");
    mexMakeArrayPersistent(sRtwOptimizationInfoStruct);
  }

  return(sRtwOptimizationInfoStruct);
}

void unload_A3_TwoDOF_PID_torque_joint_space_optimization_info(void)
{
  if (sRtwOptimizationInfoStruct!=NULL) {
    mxDestroyArray(sRtwOptimizationInfoStruct);
    sRtwOptimizationInfoStruct = NULL;
  }
}
