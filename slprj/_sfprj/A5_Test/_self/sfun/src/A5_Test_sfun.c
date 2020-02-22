/* Include files */

#include "A5_Test_sfun.h"
#include "A5_Test_sfun_debug_macros.h"
#include "c1_A5_Test.h"
#include "c3_A5_Test.h"
#include "c4_A5_Test.h"
#include "c5_A5_Test.h"
#include "c9_A5_Test.h"

/* Type Definitions */

/* Named Constants */

/* Variable Declarations */

/* Variable Definitions */
uint32_T _A5_TestMachineNumber_;

/* Function Declarations */

/* Function Definitions */
void A5_Test_initializer(void)
{
}

void A5_Test_terminator(void)
{
}

/* SFunction Glue Code */
unsigned int sf_A5_Test_method_dispatcher(SimStruct *simstructPtr, unsigned int
  chartFileNumber, const char* specsCksum, int_T method, void *data)
{
  if (chartFileNumber==1) {
    c1_A5_Test_method_dispatcher(simstructPtr, method, data);
    return 1;
  }

  if (chartFileNumber==3) {
    c3_A5_Test_method_dispatcher(simstructPtr, method, data);
    return 1;
  }

  if (chartFileNumber==4) {
    c4_A5_Test_method_dispatcher(simstructPtr, method, data);
    return 1;
  }

  if (chartFileNumber==5) {
    c5_A5_Test_method_dispatcher(simstructPtr, method, data);
    return 1;
  }

  if (chartFileNumber==9) {
    c9_A5_Test_method_dispatcher(simstructPtr, method, data);
    return 1;
  }

  return 0;
}

unsigned int sf_A5_Test_process_check_sum_call( int nlhs, mxArray * plhs[], int
  nrhs, const mxArray * prhs[] )
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
      ((real_T *)mxGetPr((plhs[0])))[0] = (real_T)(3438245244U);
      ((real_T *)mxGetPr((plhs[0])))[1] = (real_T)(2143018148U);
      ((real_T *)mxGetPr((plhs[0])))[2] = (real_T)(4235761239U);
      ((real_T *)mxGetPr((plhs[0])))[3] = (real_T)(4167470578U);
    } else if (!strcmp(commandName,"exportedFcn")) {
      ((real_T *)mxGetPr((plhs[0])))[0] = (real_T)(0U);
      ((real_T *)mxGetPr((plhs[0])))[1] = (real_T)(0U);
      ((real_T *)mxGetPr((plhs[0])))[2] = (real_T)(0U);
      ((real_T *)mxGetPr((plhs[0])))[3] = (real_T)(0U);
    } else if (!strcmp(commandName,"makefile")) {
      ((real_T *)mxGetPr((plhs[0])))[0] = (real_T)(2014545357U);
      ((real_T *)mxGetPr((plhs[0])))[1] = (real_T)(2407931646U);
      ((real_T *)mxGetPr((plhs[0])))[2] = (real_T)(3053603125U);
      ((real_T *)mxGetPr((plhs[0])))[3] = (real_T)(1032017953U);
    } else if (nrhs==3 && !strcmp(commandName,"chart")) {
      unsigned int chartFileNumber;
      chartFileNumber = (unsigned int)mxGetScalar(prhs[2]);
      switch (chartFileNumber) {
       case 1:
        {
          extern void sf_c1_A5_Test_get_check_sum(mxArray *plhs[]);
          sf_c1_A5_Test_get_check_sum(plhs);
          break;
        }

       case 3:
        {
          extern void sf_c3_A5_Test_get_check_sum(mxArray *plhs[]);
          sf_c3_A5_Test_get_check_sum(plhs);
          break;
        }

       case 4:
        {
          extern void sf_c4_A5_Test_get_check_sum(mxArray *plhs[]);
          sf_c4_A5_Test_get_check_sum(plhs);
          break;
        }

       case 5:
        {
          extern void sf_c5_A5_Test_get_check_sum(mxArray *plhs[]);
          sf_c5_A5_Test_get_check_sum(plhs);
          break;
        }

       case 9:
        {
          extern void sf_c9_A5_Test_get_check_sum(mxArray *plhs[]);
          sf_c9_A5_Test_get_check_sum(plhs);
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
    ((real_T *)mxGetPr((plhs[0])))[0] = (real_T)(2941168782U);
    ((real_T *)mxGetPr((plhs[0])))[1] = (real_T)(1351440822U);
    ((real_T *)mxGetPr((plhs[0])))[2] = (real_T)(3813517601U);
    ((real_T *)mxGetPr((plhs[0])))[3] = (real_T)(3130354002U);
  }

  return 1;

#else

  return 0;

#endif

}

unsigned int sf_A5_Test_autoinheritance_info( int nlhs, mxArray * plhs[], int
  nrhs, const mxArray * prhs[] )
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
     case 1:
      {
        if (strcmp(aiChksum, "yjrESo6OQKHEJVyoSHZ9OC") == 0) {
          extern mxArray *sf_c1_A5_Test_get_autoinheritance_info(void);
          plhs[0] = sf_c1_A5_Test_get_autoinheritance_info();
          break;
        }

        plhs[0] = mxCreateDoubleMatrix(0,0,mxREAL);
        break;
      }

     case 3:
      {
        if (strcmp(aiChksum, "IyAUguwxPcT8LOralLfKFH") == 0) {
          extern mxArray *sf_c3_A5_Test_get_autoinheritance_info(void);
          plhs[0] = sf_c3_A5_Test_get_autoinheritance_info();
          break;
        }

        plhs[0] = mxCreateDoubleMatrix(0,0,mxREAL);
        break;
      }

     case 4:
      {
        if (strcmp(aiChksum, "k0bxo2jD5MCTi4oYIjJtBE") == 0) {
          extern mxArray *sf_c4_A5_Test_get_autoinheritance_info(void);
          plhs[0] = sf_c4_A5_Test_get_autoinheritance_info();
          break;
        }

        plhs[0] = mxCreateDoubleMatrix(0,0,mxREAL);
        break;
      }

     case 5:
      {
        if (strcmp(aiChksum, "IyAUguwxPcT8LOralLfKFH") == 0) {
          extern mxArray *sf_c5_A5_Test_get_autoinheritance_info(void);
          plhs[0] = sf_c5_A5_Test_get_autoinheritance_info();
          break;
        }

        plhs[0] = mxCreateDoubleMatrix(0,0,mxREAL);
        break;
      }

     case 9:
      {
        if (strcmp(aiChksum, "NQiWieFnKxKw6etkYDBDlF") == 0) {
          extern mxArray *sf_c9_A5_Test_get_autoinheritance_info(void);
          plhs[0] = sf_c9_A5_Test_get_autoinheritance_info();
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

unsigned int sf_A5_Test_get_eml_resolved_functions_info( int nlhs, mxArray *
  plhs[], int nrhs, const mxArray * prhs[] )
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
     case 1:
      {
        extern const mxArray *sf_c1_A5_Test_get_eml_resolved_functions_info(void);
        mxArray *persistentMxArray = (mxArray *)
          sf_c1_A5_Test_get_eml_resolved_functions_info();
        plhs[0] = mxDuplicateArray(persistentMxArray);
        mxDestroyArray(persistentMxArray);
        break;
      }

     case 3:
      {
        extern const mxArray *sf_c3_A5_Test_get_eml_resolved_functions_info(void);
        mxArray *persistentMxArray = (mxArray *)
          sf_c3_A5_Test_get_eml_resolved_functions_info();
        plhs[0] = mxDuplicateArray(persistentMxArray);
        mxDestroyArray(persistentMxArray);
        break;
      }

     case 4:
      {
        extern const mxArray *sf_c4_A5_Test_get_eml_resolved_functions_info(void);
        mxArray *persistentMxArray = (mxArray *)
          sf_c4_A5_Test_get_eml_resolved_functions_info();
        plhs[0] = mxDuplicateArray(persistentMxArray);
        mxDestroyArray(persistentMxArray);
        break;
      }

     case 5:
      {
        extern const mxArray *sf_c5_A5_Test_get_eml_resolved_functions_info(void);
        mxArray *persistentMxArray = (mxArray *)
          sf_c5_A5_Test_get_eml_resolved_functions_info();
        plhs[0] = mxDuplicateArray(persistentMxArray);
        mxDestroyArray(persistentMxArray);
        break;
      }

     case 9:
      {
        extern const mxArray *sf_c9_A5_Test_get_eml_resolved_functions_info(void);
        mxArray *persistentMxArray = (mxArray *)
          sf_c9_A5_Test_get_eml_resolved_functions_info();
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

unsigned int sf_A5_Test_third_party_uses_info( int nlhs, mxArray * plhs[], int
  nrhs, const mxArray * prhs[] )
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
     case 1:
      {
        if (strcmp(tpChksum, "y6E8SD3ScV9NGPer1QyCaG") == 0) {
          extern mxArray *sf_c1_A5_Test_third_party_uses_info(void);
          plhs[0] = sf_c1_A5_Test_third_party_uses_info();
          break;
        }
      }

     case 3:
      {
        if (strcmp(tpChksum, "4c9Iu3XWf4Mlf5egnrOAbC") == 0) {
          extern mxArray *sf_c3_A5_Test_third_party_uses_info(void);
          plhs[0] = sf_c3_A5_Test_third_party_uses_info();
          break;
        }
      }

     case 4:
      {
        if (strcmp(tpChksum, "0u9Hbmvv5FcreUp8WKevoB") == 0) {
          extern mxArray *sf_c4_A5_Test_third_party_uses_info(void);
          plhs[0] = sf_c4_A5_Test_third_party_uses_info();
          break;
        }
      }

     case 5:
      {
        if (strcmp(tpChksum, "4c9Iu3XWf4Mlf5egnrOAbC") == 0) {
          extern mxArray *sf_c5_A5_Test_third_party_uses_info(void);
          plhs[0] = sf_c5_A5_Test_third_party_uses_info();
          break;
        }
      }

     case 9:
      {
        if (strcmp(tpChksum, "hdTIvdWqbWcmNL7S0dnGLE") == 0) {
          extern mxArray *sf_c9_A5_Test_third_party_uses_info(void);
          plhs[0] = sf_c9_A5_Test_third_party_uses_info();
          break;
        }
      }

     default:
      plhs[0] = mxCreateDoubleMatrix(0,0,mxREAL);
    }
  }

  return 1;
}

unsigned int sf_A5_Test_updateBuildInfo_args_info( int nlhs, mxArray * plhs[],
  int nrhs, const mxArray * prhs[] )
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
     case 1:
      {
        if (strcmp(tpChksum, "y6E8SD3ScV9NGPer1QyCaG") == 0) {
          extern mxArray *sf_c1_A5_Test_updateBuildInfo_args_info(void);
          plhs[0] = sf_c1_A5_Test_updateBuildInfo_args_info();
          break;
        }
      }

     case 3:
      {
        if (strcmp(tpChksum, "4c9Iu3XWf4Mlf5egnrOAbC") == 0) {
          extern mxArray *sf_c3_A5_Test_updateBuildInfo_args_info(void);
          plhs[0] = sf_c3_A5_Test_updateBuildInfo_args_info();
          break;
        }
      }

     case 4:
      {
        if (strcmp(tpChksum, "0u9Hbmvv5FcreUp8WKevoB") == 0) {
          extern mxArray *sf_c4_A5_Test_updateBuildInfo_args_info(void);
          plhs[0] = sf_c4_A5_Test_updateBuildInfo_args_info();
          break;
        }
      }

     case 5:
      {
        if (strcmp(tpChksum, "4c9Iu3XWf4Mlf5egnrOAbC") == 0) {
          extern mxArray *sf_c5_A5_Test_updateBuildInfo_args_info(void);
          plhs[0] = sf_c5_A5_Test_updateBuildInfo_args_info();
          break;
        }
      }

     case 9:
      {
        if (strcmp(tpChksum, "hdTIvdWqbWcmNL7S0dnGLE") == 0) {
          extern mxArray *sf_c9_A5_Test_updateBuildInfo_args_info(void);
          plhs[0] = sf_c9_A5_Test_updateBuildInfo_args_info();
          break;
        }
      }

     default:
      plhs[0] = mxCreateDoubleMatrix(0,0,mxREAL);
    }
  }

  return 1;
}

void A5_Test_debug_initialize(struct SfDebugInstanceStruct* debugInstance)
{
  _A5_TestMachineNumber_ = sf_debug_initialize_machine(debugInstance,"A5_Test",
    "sfun",0,5,0,0,0);
  sf_debug_set_machine_event_thresholds(debugInstance,_A5_TestMachineNumber_,0,0);
  sf_debug_set_machine_data_thresholds(debugInstance,_A5_TestMachineNumber_,0);
}

void A5_Test_register_exported_symbols(SimStruct* S)
{
}

static mxArray* sRtwOptimizationInfoStruct= NULL;
mxArray* load_A5_Test_optimization_info(void)
{
  if (sRtwOptimizationInfoStruct==NULL) {
    sRtwOptimizationInfoStruct = sf_load_rtw_optimization_info("A5_Test",
      "A5_Test");
    mexMakeArrayPersistent(sRtwOptimizationInfoStruct);
  }

  return(sRtwOptimizationInfoStruct);
}

void unload_A5_Test_optimization_info(void)
{
  if (sRtwOptimizationInfoStruct!=NULL) {
    mxDestroyArray(sRtwOptimizationInfoStruct);
    sRtwOptimizationInfoStruct = NULL;
  }
}
