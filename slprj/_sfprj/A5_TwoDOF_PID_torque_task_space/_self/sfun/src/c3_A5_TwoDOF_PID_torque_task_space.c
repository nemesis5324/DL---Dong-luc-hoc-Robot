/* Include files */

#include <stddef.h>
#include "blas.h"
#include "A5_TwoDOF_PID_torque_task_space_sfun.h"
#include "c3_A5_TwoDOF_PID_torque_task_space.h"
#include "mwmathutil.h"
#define CHARTINSTANCE_CHARTNUMBER      (chartInstance->chartNumber)
#define CHARTINSTANCE_INSTANCENUMBER   (chartInstance->instanceNumber)
#include "A5_TwoDOF_PID_torque_task_space_sfun_debug_macros.h"
#define _SF_MEX_LISTEN_FOR_CTRL_C(S)   sf_mex_listen_for_ctrl_c(sfGlobalDebugInstanceStruct,S);

/* Type Definitions */

/* Named Constants */
#define CALL_EVENT                     (-1)

/* Variable Declarations */

/* Variable Definitions */
static real_T _sfTime_;
static const char * c3_debug_family_names[8] = { "L1", "L2", "X1", "X2",
  "nargin", "nargout", "Q", "X" };

/* Function Declarations */
static void initialize_c3_A5_TwoDOF_PID_torque_task_space
  (SFc3_A5_TwoDOF_PID_torque_task_spaceInstanceStruct *chartInstance);
static void initialize_params_c3_A5_TwoDOF_PID_torque_task_space
  (SFc3_A5_TwoDOF_PID_torque_task_spaceInstanceStruct *chartInstance);
static void enable_c3_A5_TwoDOF_PID_torque_task_space
  (SFc3_A5_TwoDOF_PID_torque_task_spaceInstanceStruct *chartInstance);
static void disable_c3_A5_TwoDOF_PID_torque_task_space
  (SFc3_A5_TwoDOF_PID_torque_task_spaceInstanceStruct *chartInstance);
static void c3_update_debugger_state_c3_A5_TwoDOF_PID_torque_task_space
  (SFc3_A5_TwoDOF_PID_torque_task_spaceInstanceStruct *chartInstance);
static const mxArray *get_sim_state_c3_A5_TwoDOF_PID_torque_task_space
  (SFc3_A5_TwoDOF_PID_torque_task_spaceInstanceStruct *chartInstance);
static void set_sim_state_c3_A5_TwoDOF_PID_torque_task_space
  (SFc3_A5_TwoDOF_PID_torque_task_spaceInstanceStruct *chartInstance, const
   mxArray *c3_st);
static void finalize_c3_A5_TwoDOF_PID_torque_task_space
  (SFc3_A5_TwoDOF_PID_torque_task_spaceInstanceStruct *chartInstance);
static void sf_gateway_c3_A5_TwoDOF_PID_torque_task_space
  (SFc3_A5_TwoDOF_PID_torque_task_spaceInstanceStruct *chartInstance);
static void initSimStructsc3_A5_TwoDOF_PID_torque_task_space
  (SFc3_A5_TwoDOF_PID_torque_task_spaceInstanceStruct *chartInstance);
static void init_script_number_translation(uint32_T c3_machineNumber, uint32_T
  c3_chartNumber, uint32_T c3_instanceNumber);
static const mxArray *c3_sf_marshallOut(void *chartInstanceVoid, void *c3_inData);
static void c3_emlrt_marshallIn
  (SFc3_A5_TwoDOF_PID_torque_task_spaceInstanceStruct *chartInstance, const
   mxArray *c3_X, const char_T *c3_identifier, real_T c3_y[2]);
static void c3_b_emlrt_marshallIn
  (SFc3_A5_TwoDOF_PID_torque_task_spaceInstanceStruct *chartInstance, const
   mxArray *c3_u, const emlrtMsgIdentifier *c3_parentId, real_T c3_y[2]);
static void c3_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c3_mxArrayInData, const char_T *c3_varName, void *c3_outData);
static const mxArray *c3_b_sf_marshallOut(void *chartInstanceVoid, void
  *c3_inData);
static real_T c3_c_emlrt_marshallIn
  (SFc3_A5_TwoDOF_PID_torque_task_spaceInstanceStruct *chartInstance, const
   mxArray *c3_u, const emlrtMsgIdentifier *c3_parentId);
static void c3_b_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c3_mxArrayInData, const char_T *c3_varName, void *c3_outData);
static void c3_info_helper(const mxArray **c3_info);
static const mxArray *c3_emlrt_marshallOut(const char * c3_u);
static const mxArray *c3_b_emlrt_marshallOut(const uint32_T c3_u);
static const mxArray *c3_c_sf_marshallOut(void *chartInstanceVoid, void
  *c3_inData);
static int32_T c3_d_emlrt_marshallIn
  (SFc3_A5_TwoDOF_PID_torque_task_spaceInstanceStruct *chartInstance, const
   mxArray *c3_u, const emlrtMsgIdentifier *c3_parentId);
static void c3_c_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c3_mxArrayInData, const char_T *c3_varName, void *c3_outData);
static uint8_T c3_e_emlrt_marshallIn
  (SFc3_A5_TwoDOF_PID_torque_task_spaceInstanceStruct *chartInstance, const
   mxArray *c3_b_is_active_c3_A5_TwoDOF_PID_torque_task_space, const char_T
   *c3_identifier);
static uint8_T c3_f_emlrt_marshallIn
  (SFc3_A5_TwoDOF_PID_torque_task_spaceInstanceStruct *chartInstance, const
   mxArray *c3_u, const emlrtMsgIdentifier *c3_parentId);
static void init_dsm_address_info
  (SFc3_A5_TwoDOF_PID_torque_task_spaceInstanceStruct *chartInstance);

/* Function Definitions */
static void initialize_c3_A5_TwoDOF_PID_torque_task_space
  (SFc3_A5_TwoDOF_PID_torque_task_spaceInstanceStruct *chartInstance)
{
  chartInstance->c3_sfEvent = CALL_EVENT;
  _sfTime_ = sf_get_time(chartInstance->S);
  chartInstance->c3_is_active_c3_A5_TwoDOF_PID_torque_task_space = 0U;
}

static void initialize_params_c3_A5_TwoDOF_PID_torque_task_space
  (SFc3_A5_TwoDOF_PID_torque_task_spaceInstanceStruct *chartInstance)
{
  (void)chartInstance;
}

static void enable_c3_A5_TwoDOF_PID_torque_task_space
  (SFc3_A5_TwoDOF_PID_torque_task_spaceInstanceStruct *chartInstance)
{
  _sfTime_ = sf_get_time(chartInstance->S);
}

static void disable_c3_A5_TwoDOF_PID_torque_task_space
  (SFc3_A5_TwoDOF_PID_torque_task_spaceInstanceStruct *chartInstance)
{
  _sfTime_ = sf_get_time(chartInstance->S);
}

static void c3_update_debugger_state_c3_A5_TwoDOF_PID_torque_task_space
  (SFc3_A5_TwoDOF_PID_torque_task_spaceInstanceStruct *chartInstance)
{
  (void)chartInstance;
}

static const mxArray *get_sim_state_c3_A5_TwoDOF_PID_torque_task_space
  (SFc3_A5_TwoDOF_PID_torque_task_spaceInstanceStruct *chartInstance)
{
  const mxArray *c3_st;
  const mxArray *c3_y = NULL;
  int32_T c3_i0;
  real_T c3_u[2];
  const mxArray *c3_b_y = NULL;
  uint8_T c3_hoistedGlobal;
  uint8_T c3_b_u;
  const mxArray *c3_c_y = NULL;
  real_T (*c3_X)[2];
  c3_X = (real_T (*)[2])ssGetOutputPortSignal(chartInstance->S, 1);
  c3_st = NULL;
  c3_st = NULL;
  c3_y = NULL;
  sf_mex_assign(&c3_y, sf_mex_createcellmatrix(2, 1), false);
  for (c3_i0 = 0; c3_i0 < 2; c3_i0++) {
    c3_u[c3_i0] = (*c3_X)[c3_i0];
  }

  c3_b_y = NULL;
  sf_mex_assign(&c3_b_y, sf_mex_create("y", c3_u, 0, 0U, 1U, 0U, 1, 2), false);
  sf_mex_setcell(c3_y, 0, c3_b_y);
  c3_hoistedGlobal =
    chartInstance->c3_is_active_c3_A5_TwoDOF_PID_torque_task_space;
  c3_b_u = c3_hoistedGlobal;
  c3_c_y = NULL;
  sf_mex_assign(&c3_c_y, sf_mex_create("y", &c3_b_u, 3, 0U, 0U, 0U, 0), false);
  sf_mex_setcell(c3_y, 1, c3_c_y);
  sf_mex_assign(&c3_st, c3_y, false);
  return c3_st;
}

static void set_sim_state_c3_A5_TwoDOF_PID_torque_task_space
  (SFc3_A5_TwoDOF_PID_torque_task_spaceInstanceStruct *chartInstance, const
   mxArray *c3_st)
{
  const mxArray *c3_u;
  real_T c3_dv0[2];
  int32_T c3_i1;
  real_T (*c3_X)[2];
  c3_X = (real_T (*)[2])ssGetOutputPortSignal(chartInstance->S, 1);
  chartInstance->c3_doneDoubleBufferReInit = true;
  c3_u = sf_mex_dup(c3_st);
  c3_emlrt_marshallIn(chartInstance, sf_mex_dup(sf_mex_getcell(c3_u, 0)), "X",
                      c3_dv0);
  for (c3_i1 = 0; c3_i1 < 2; c3_i1++) {
    (*c3_X)[c3_i1] = c3_dv0[c3_i1];
  }

  chartInstance->c3_is_active_c3_A5_TwoDOF_PID_torque_task_space =
    c3_e_emlrt_marshallIn(chartInstance, sf_mex_dup(sf_mex_getcell(c3_u, 1)),
    "is_active_c3_A5_TwoDOF_PID_torque_task_space");
  sf_mex_destroy(&c3_u);
  c3_update_debugger_state_c3_A5_TwoDOF_PID_torque_task_space(chartInstance);
  sf_mex_destroy(&c3_st);
}

static void finalize_c3_A5_TwoDOF_PID_torque_task_space
  (SFc3_A5_TwoDOF_PID_torque_task_spaceInstanceStruct *chartInstance)
{
  (void)chartInstance;
}

static void sf_gateway_c3_A5_TwoDOF_PID_torque_task_space
  (SFc3_A5_TwoDOF_PID_torque_task_spaceInstanceStruct *chartInstance)
{
  int32_T c3_i2;
  int32_T c3_i3;
  real_T c3_Q[2];
  uint32_T c3_debug_family_var_map[8];
  real_T c3_L1;
  real_T c3_L2;
  real_T c3_X1;
  real_T c3_X2;
  real_T c3_nargin = 1.0;
  real_T c3_nargout = 1.0;
  real_T c3_X[2];
  real_T c3_x;
  real_T c3_b_x;
  real_T c3_c_x;
  real_T c3_d_x;
  real_T c3_e_x;
  real_T c3_f_x;
  real_T c3_g_x;
  real_T c3_h_x;
  int32_T c3_i4;
  int32_T c3_i5;
  real_T (*c3_b_X)[2];
  real_T (*c3_b_Q)[2];
  c3_b_X = (real_T (*)[2])ssGetOutputPortSignal(chartInstance->S, 1);
  c3_b_Q = (real_T (*)[2])ssGetInputPortSignal(chartInstance->S, 0);
  _SFD_SYMBOL_SCOPE_PUSH(0U, 0U);
  _sfTime_ = sf_get_time(chartInstance->S);
  _SFD_CC_CALL(CHART_ENTER_SFUNCTION_TAG, 0U, chartInstance->c3_sfEvent);
  for (c3_i2 = 0; c3_i2 < 2; c3_i2++) {
    _SFD_DATA_RANGE_CHECK((*c3_b_Q)[c3_i2], 0U);
  }

  chartInstance->c3_sfEvent = CALL_EVENT;
  _SFD_CC_CALL(CHART_ENTER_DURING_FUNCTION_TAG, 0U, chartInstance->c3_sfEvent);
  for (c3_i3 = 0; c3_i3 < 2; c3_i3++) {
    c3_Q[c3_i3] = (*c3_b_Q)[c3_i3];
  }

  _SFD_SYMBOL_SCOPE_PUSH_EML(0U, 8U, 8U, c3_debug_family_names,
    c3_debug_family_var_map);
  _SFD_SYMBOL_SCOPE_ADD_EML(&c3_L1, 0U, c3_b_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML(&c3_L2, 1U, c3_b_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c3_X1, 2U, c3_b_sf_marshallOut,
    c3_b_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c3_X2, 3U, c3_b_sf_marshallOut,
    c3_b_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c3_nargin, 4U, c3_b_sf_marshallOut,
    c3_b_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c3_nargout, 5U, c3_b_sf_marshallOut,
    c3_b_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML(c3_Q, 6U, c3_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c3_X, 7U, c3_sf_marshallOut,
    c3_sf_marshallIn);
  CV_EML_FCN(0, 0);
  _SFD_EML_CALL(0U, chartInstance->c3_sfEvent, 4);
  c3_L1 = 0.2;
  _SFD_EML_CALL(0U, chartInstance->c3_sfEvent, 5);
  c3_L2 = 0.1442;
  _SFD_EML_CALL(0U, chartInstance->c3_sfEvent, 6);
  c3_x = c3_Q[0];
  c3_b_x = c3_x;
  c3_b_x = muDoubleScalarCos(c3_b_x);
  c3_c_x = c3_Q[0] + c3_Q[1];
  c3_d_x = c3_c_x;
  c3_d_x = muDoubleScalarCos(c3_d_x);
  c3_X1 = 0.2 * c3_b_x + 0.1442 * c3_d_x;
  _SFD_EML_CALL(0U, chartInstance->c3_sfEvent, 7);
  c3_e_x = c3_Q[0];
  c3_f_x = c3_e_x;
  c3_f_x = muDoubleScalarSin(c3_f_x);
  c3_g_x = c3_Q[0] + c3_Q[1];
  c3_h_x = c3_g_x;
  c3_h_x = muDoubleScalarSin(c3_h_x);
  c3_X2 = 0.2 * c3_f_x + 0.1442 * c3_h_x;
  _SFD_EML_CALL(0U, chartInstance->c3_sfEvent, 8);
  c3_X[0] = c3_X1;
  c3_X[1] = c3_X2;
  _SFD_EML_CALL(0U, chartInstance->c3_sfEvent, -8);
  _SFD_SYMBOL_SCOPE_POP();
  for (c3_i4 = 0; c3_i4 < 2; c3_i4++) {
    (*c3_b_X)[c3_i4] = c3_X[c3_i4];
  }

  _SFD_CC_CALL(EXIT_OUT_OF_FUNCTION_TAG, 0U, chartInstance->c3_sfEvent);
  _SFD_SYMBOL_SCOPE_POP();
  _SFD_CHECK_FOR_STATE_INCONSISTENCY
    (_A5_TwoDOF_PID_torque_task_spaceMachineNumber_, chartInstance->chartNumber,
     chartInstance->instanceNumber);
  for (c3_i5 = 0; c3_i5 < 2; c3_i5++) {
    _SFD_DATA_RANGE_CHECK((*c3_b_X)[c3_i5], 1U);
  }
}

static void initSimStructsc3_A5_TwoDOF_PID_torque_task_space
  (SFc3_A5_TwoDOF_PID_torque_task_spaceInstanceStruct *chartInstance)
{
  (void)chartInstance;
}

static void init_script_number_translation(uint32_T c3_machineNumber, uint32_T
  c3_chartNumber, uint32_T c3_instanceNumber)
{
  (void)c3_machineNumber;
  (void)c3_chartNumber;
  (void)c3_instanceNumber;
}

static const mxArray *c3_sf_marshallOut(void *chartInstanceVoid, void *c3_inData)
{
  const mxArray *c3_mxArrayOutData = NULL;
  int32_T c3_i6;
  real_T c3_b_inData[2];
  int32_T c3_i7;
  real_T c3_u[2];
  const mxArray *c3_y = NULL;
  SFc3_A5_TwoDOF_PID_torque_task_spaceInstanceStruct *chartInstance;
  chartInstance = (SFc3_A5_TwoDOF_PID_torque_task_spaceInstanceStruct *)
    chartInstanceVoid;
  c3_mxArrayOutData = NULL;
  for (c3_i6 = 0; c3_i6 < 2; c3_i6++) {
    c3_b_inData[c3_i6] = (*(real_T (*)[2])c3_inData)[c3_i6];
  }

  for (c3_i7 = 0; c3_i7 < 2; c3_i7++) {
    c3_u[c3_i7] = c3_b_inData[c3_i7];
  }

  c3_y = NULL;
  sf_mex_assign(&c3_y, sf_mex_create("y", c3_u, 0, 0U, 1U, 0U, 1, 2), false);
  sf_mex_assign(&c3_mxArrayOutData, c3_y, false);
  return c3_mxArrayOutData;
}

static void c3_emlrt_marshallIn
  (SFc3_A5_TwoDOF_PID_torque_task_spaceInstanceStruct *chartInstance, const
   mxArray *c3_X, const char_T *c3_identifier, real_T c3_y[2])
{
  emlrtMsgIdentifier c3_thisId;
  c3_thisId.fIdentifier = c3_identifier;
  c3_thisId.fParent = NULL;
  c3_b_emlrt_marshallIn(chartInstance, sf_mex_dup(c3_X), &c3_thisId, c3_y);
  sf_mex_destroy(&c3_X);
}

static void c3_b_emlrt_marshallIn
  (SFc3_A5_TwoDOF_PID_torque_task_spaceInstanceStruct *chartInstance, const
   mxArray *c3_u, const emlrtMsgIdentifier *c3_parentId, real_T c3_y[2])
{
  real_T c3_dv1[2];
  int32_T c3_i8;
  (void)chartInstance;
  sf_mex_import(c3_parentId, sf_mex_dup(c3_u), c3_dv1, 1, 0, 0U, 1, 0U, 1, 2);
  for (c3_i8 = 0; c3_i8 < 2; c3_i8++) {
    c3_y[c3_i8] = c3_dv1[c3_i8];
  }

  sf_mex_destroy(&c3_u);
}

static void c3_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c3_mxArrayInData, const char_T *c3_varName, void *c3_outData)
{
  const mxArray *c3_X;
  const char_T *c3_identifier;
  emlrtMsgIdentifier c3_thisId;
  real_T c3_y[2];
  int32_T c3_i9;
  SFc3_A5_TwoDOF_PID_torque_task_spaceInstanceStruct *chartInstance;
  chartInstance = (SFc3_A5_TwoDOF_PID_torque_task_spaceInstanceStruct *)
    chartInstanceVoid;
  c3_X = sf_mex_dup(c3_mxArrayInData);
  c3_identifier = c3_varName;
  c3_thisId.fIdentifier = c3_identifier;
  c3_thisId.fParent = NULL;
  c3_b_emlrt_marshallIn(chartInstance, sf_mex_dup(c3_X), &c3_thisId, c3_y);
  sf_mex_destroy(&c3_X);
  for (c3_i9 = 0; c3_i9 < 2; c3_i9++) {
    (*(real_T (*)[2])c3_outData)[c3_i9] = c3_y[c3_i9];
  }

  sf_mex_destroy(&c3_mxArrayInData);
}

static const mxArray *c3_b_sf_marshallOut(void *chartInstanceVoid, void
  *c3_inData)
{
  const mxArray *c3_mxArrayOutData = NULL;
  real_T c3_u;
  const mxArray *c3_y = NULL;
  SFc3_A5_TwoDOF_PID_torque_task_spaceInstanceStruct *chartInstance;
  chartInstance = (SFc3_A5_TwoDOF_PID_torque_task_spaceInstanceStruct *)
    chartInstanceVoid;
  c3_mxArrayOutData = NULL;
  c3_u = *(real_T *)c3_inData;
  c3_y = NULL;
  sf_mex_assign(&c3_y, sf_mex_create("y", &c3_u, 0, 0U, 0U, 0U, 0), false);
  sf_mex_assign(&c3_mxArrayOutData, c3_y, false);
  return c3_mxArrayOutData;
}

static real_T c3_c_emlrt_marshallIn
  (SFc3_A5_TwoDOF_PID_torque_task_spaceInstanceStruct *chartInstance, const
   mxArray *c3_u, const emlrtMsgIdentifier *c3_parentId)
{
  real_T c3_y;
  real_T c3_d0;
  (void)chartInstance;
  sf_mex_import(c3_parentId, sf_mex_dup(c3_u), &c3_d0, 1, 0, 0U, 0, 0U, 0);
  c3_y = c3_d0;
  sf_mex_destroy(&c3_u);
  return c3_y;
}

static void c3_b_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c3_mxArrayInData, const char_T *c3_varName, void *c3_outData)
{
  const mxArray *c3_nargout;
  const char_T *c3_identifier;
  emlrtMsgIdentifier c3_thisId;
  real_T c3_y;
  SFc3_A5_TwoDOF_PID_torque_task_spaceInstanceStruct *chartInstance;
  chartInstance = (SFc3_A5_TwoDOF_PID_torque_task_spaceInstanceStruct *)
    chartInstanceVoid;
  c3_nargout = sf_mex_dup(c3_mxArrayInData);
  c3_identifier = c3_varName;
  c3_thisId.fIdentifier = c3_identifier;
  c3_thisId.fParent = NULL;
  c3_y = c3_c_emlrt_marshallIn(chartInstance, sf_mex_dup(c3_nargout), &c3_thisId);
  sf_mex_destroy(&c3_nargout);
  *(real_T *)c3_outData = c3_y;
  sf_mex_destroy(&c3_mxArrayInData);
}

const mxArray
  *sf_c3_A5_TwoDOF_PID_torque_task_space_get_eml_resolved_functions_info(void)
{
  const mxArray *c3_nameCaptureInfo = NULL;
  c3_nameCaptureInfo = NULL;
  sf_mex_assign(&c3_nameCaptureInfo, sf_mex_createstruct("structure", 2, 4, 1),
                false);
  c3_info_helper(&c3_nameCaptureInfo);
  sf_mex_emlrtNameCapturePostProcessR2012a(&c3_nameCaptureInfo);
  return c3_nameCaptureInfo;
}

static void c3_info_helper(const mxArray **c3_info)
{
  const mxArray *c3_rhs0 = NULL;
  const mxArray *c3_lhs0 = NULL;
  const mxArray *c3_rhs1 = NULL;
  const mxArray *c3_lhs1 = NULL;
  const mxArray *c3_rhs2 = NULL;
  const mxArray *c3_lhs2 = NULL;
  const mxArray *c3_rhs3 = NULL;
  const mxArray *c3_lhs3 = NULL;
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(""), "context", "context", 0);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("cos"), "name", "name", 0);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 0);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/cos.m"), "resolved",
                  "resolved", 0);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(1343837572U), "fileTimeLo",
                  "fileTimeLo", 0);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 0);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 0);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 0);
  sf_mex_assign(&c3_rhs0, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c3_lhs0, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_rhs0), "rhs", "rhs", 0);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_lhs0), "lhs", "lhs", 0);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/cos.m"), "context",
                  "context", 1);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("eml_scalar_cos"), "name",
                  "name", 1);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 1);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/eml_scalar_cos.m"),
                  "resolved", "resolved", 1);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(1286825922U), "fileTimeLo",
                  "fileTimeLo", 1);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 1);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 1);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 1);
  sf_mex_assign(&c3_rhs1, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c3_lhs1, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_rhs1), "rhs", "rhs", 1);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_lhs1), "lhs", "lhs", 1);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(""), "context", "context", 2);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("sin"), "name", "name", 2);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 2);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/sin.m"), "resolved",
                  "resolved", 2);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(1343837586U), "fileTimeLo",
                  "fileTimeLo", 2);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 2);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 2);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 2);
  sf_mex_assign(&c3_rhs2, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c3_lhs2, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_rhs2), "rhs", "rhs", 2);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_lhs2), "lhs", "lhs", 2);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/sin.m"), "context",
                  "context", 3);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("eml_scalar_sin"), "name",
                  "name", 3);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 3);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/eml_scalar_sin.m"),
                  "resolved", "resolved", 3);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(1286825936U), "fileTimeLo",
                  "fileTimeLo", 3);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 3);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 3);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 3);
  sf_mex_assign(&c3_rhs3, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c3_lhs3, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_rhs3), "rhs", "rhs", 3);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_lhs3), "lhs", "lhs", 3);
  sf_mex_destroy(&c3_rhs0);
  sf_mex_destroy(&c3_lhs0);
  sf_mex_destroy(&c3_rhs1);
  sf_mex_destroy(&c3_lhs1);
  sf_mex_destroy(&c3_rhs2);
  sf_mex_destroy(&c3_lhs2);
  sf_mex_destroy(&c3_rhs3);
  sf_mex_destroy(&c3_lhs3);
}

static const mxArray *c3_emlrt_marshallOut(const char * c3_u)
{
  const mxArray *c3_y = NULL;
  c3_y = NULL;
  sf_mex_assign(&c3_y, sf_mex_create("y", c3_u, 15, 0U, 0U, 0U, 2, 1, strlen
    (c3_u)), false);
  return c3_y;
}

static const mxArray *c3_b_emlrt_marshallOut(const uint32_T c3_u)
{
  const mxArray *c3_y = NULL;
  c3_y = NULL;
  sf_mex_assign(&c3_y, sf_mex_create("y", &c3_u, 7, 0U, 0U, 0U, 0), false);
  return c3_y;
}

static const mxArray *c3_c_sf_marshallOut(void *chartInstanceVoid, void
  *c3_inData)
{
  const mxArray *c3_mxArrayOutData = NULL;
  int32_T c3_u;
  const mxArray *c3_y = NULL;
  SFc3_A5_TwoDOF_PID_torque_task_spaceInstanceStruct *chartInstance;
  chartInstance = (SFc3_A5_TwoDOF_PID_torque_task_spaceInstanceStruct *)
    chartInstanceVoid;
  c3_mxArrayOutData = NULL;
  c3_u = *(int32_T *)c3_inData;
  c3_y = NULL;
  sf_mex_assign(&c3_y, sf_mex_create("y", &c3_u, 6, 0U, 0U, 0U, 0), false);
  sf_mex_assign(&c3_mxArrayOutData, c3_y, false);
  return c3_mxArrayOutData;
}

static int32_T c3_d_emlrt_marshallIn
  (SFc3_A5_TwoDOF_PID_torque_task_spaceInstanceStruct *chartInstance, const
   mxArray *c3_u, const emlrtMsgIdentifier *c3_parentId)
{
  int32_T c3_y;
  int32_T c3_i10;
  (void)chartInstance;
  sf_mex_import(c3_parentId, sf_mex_dup(c3_u), &c3_i10, 1, 6, 0U, 0, 0U, 0);
  c3_y = c3_i10;
  sf_mex_destroy(&c3_u);
  return c3_y;
}

static void c3_c_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c3_mxArrayInData, const char_T *c3_varName, void *c3_outData)
{
  const mxArray *c3_b_sfEvent;
  const char_T *c3_identifier;
  emlrtMsgIdentifier c3_thisId;
  int32_T c3_y;
  SFc3_A5_TwoDOF_PID_torque_task_spaceInstanceStruct *chartInstance;
  chartInstance = (SFc3_A5_TwoDOF_PID_torque_task_spaceInstanceStruct *)
    chartInstanceVoid;
  c3_b_sfEvent = sf_mex_dup(c3_mxArrayInData);
  c3_identifier = c3_varName;
  c3_thisId.fIdentifier = c3_identifier;
  c3_thisId.fParent = NULL;
  c3_y = c3_d_emlrt_marshallIn(chartInstance, sf_mex_dup(c3_b_sfEvent),
    &c3_thisId);
  sf_mex_destroy(&c3_b_sfEvent);
  *(int32_T *)c3_outData = c3_y;
  sf_mex_destroy(&c3_mxArrayInData);
}

static uint8_T c3_e_emlrt_marshallIn
  (SFc3_A5_TwoDOF_PID_torque_task_spaceInstanceStruct *chartInstance, const
   mxArray *c3_b_is_active_c3_A5_TwoDOF_PID_torque_task_space, const char_T
   *c3_identifier)
{
  uint8_T c3_y;
  emlrtMsgIdentifier c3_thisId;
  c3_thisId.fIdentifier = c3_identifier;
  c3_thisId.fParent = NULL;
  c3_y = c3_f_emlrt_marshallIn(chartInstance, sf_mex_dup
    (c3_b_is_active_c3_A5_TwoDOF_PID_torque_task_space), &c3_thisId);
  sf_mex_destroy(&c3_b_is_active_c3_A5_TwoDOF_PID_torque_task_space);
  return c3_y;
}

static uint8_T c3_f_emlrt_marshallIn
  (SFc3_A5_TwoDOF_PID_torque_task_spaceInstanceStruct *chartInstance, const
   mxArray *c3_u, const emlrtMsgIdentifier *c3_parentId)
{
  uint8_T c3_y;
  uint8_T c3_u0;
  (void)chartInstance;
  sf_mex_import(c3_parentId, sf_mex_dup(c3_u), &c3_u0, 1, 3, 0U, 0, 0U, 0);
  c3_y = c3_u0;
  sf_mex_destroy(&c3_u);
  return c3_y;
}

static void init_dsm_address_info
  (SFc3_A5_TwoDOF_PID_torque_task_spaceInstanceStruct *chartInstance)
{
  (void)chartInstance;
}

/* SFunction Glue Code */
#ifdef utFree
#undef utFree
#endif

#ifdef utMalloc
#undef utMalloc
#endif

#ifdef __cplusplus

extern "C" void *utMalloc(size_t size);
extern "C" void utFree(void*);

#else

extern void *utMalloc(size_t size);
extern void utFree(void*);

#endif

void sf_c3_A5_TwoDOF_PID_torque_task_space_get_check_sum(mxArray *plhs[])
{
  ((real_T *)mxGetPr((plhs[0])))[0] = (real_T)(1856291366U);
  ((real_T *)mxGetPr((plhs[0])))[1] = (real_T)(2866118667U);
  ((real_T *)mxGetPr((plhs[0])))[2] = (real_T)(277172775U);
  ((real_T *)mxGetPr((plhs[0])))[3] = (real_T)(3536106738U);
}

mxArray *sf_c3_A5_TwoDOF_PID_torque_task_space_get_autoinheritance_info(void)
{
  const char *autoinheritanceFields[] = { "checksum", "inputs", "parameters",
    "outputs", "locals" };

  mxArray *mxAutoinheritanceInfo = mxCreateStructMatrix(1,1,5,
    autoinheritanceFields);

  {
    mxArray *mxChecksum = mxCreateString("yjrESo6OQKHEJVyoSHZ9OC");
    mxSetField(mxAutoinheritanceInfo,0,"checksum",mxChecksum);
  }

  {
    const char *dataFields[] = { "size", "type", "complexity" };

    mxArray *mxData = mxCreateStructMatrix(1,1,3,dataFields);

    {
      mxArray *mxSize = mxCreateDoubleMatrix(1,2,mxREAL);
      double *pr = mxGetPr(mxSize);
      pr[0] = (double)(2);
      pr[1] = (double)(1);
      mxSetField(mxData,0,"size",mxSize);
    }

    {
      const char *typeFields[] = { "base", "fixpt" };

      mxArray *mxType = mxCreateStructMatrix(1,1,2,typeFields);
      mxSetField(mxType,0,"base",mxCreateDoubleScalar(10));
      mxSetField(mxType,0,"fixpt",mxCreateDoubleMatrix(0,0,mxREAL));
      mxSetField(mxData,0,"type",mxType);
    }

    mxSetField(mxData,0,"complexity",mxCreateDoubleScalar(0));
    mxSetField(mxAutoinheritanceInfo,0,"inputs",mxData);
  }

  {
    mxSetField(mxAutoinheritanceInfo,0,"parameters",mxCreateDoubleMatrix(0,0,
                mxREAL));
  }

  {
    const char *dataFields[] = { "size", "type", "complexity" };

    mxArray *mxData = mxCreateStructMatrix(1,1,3,dataFields);

    {
      mxArray *mxSize = mxCreateDoubleMatrix(1,2,mxREAL);
      double *pr = mxGetPr(mxSize);
      pr[0] = (double)(2);
      pr[1] = (double)(1);
      mxSetField(mxData,0,"size",mxSize);
    }

    {
      const char *typeFields[] = { "base", "fixpt" };

      mxArray *mxType = mxCreateStructMatrix(1,1,2,typeFields);
      mxSetField(mxType,0,"base",mxCreateDoubleScalar(10));
      mxSetField(mxType,0,"fixpt",mxCreateDoubleMatrix(0,0,mxREAL));
      mxSetField(mxData,0,"type",mxType);
    }

    mxSetField(mxData,0,"complexity",mxCreateDoubleScalar(0));
    mxSetField(mxAutoinheritanceInfo,0,"outputs",mxData);
  }

  {
    mxSetField(mxAutoinheritanceInfo,0,"locals",mxCreateDoubleMatrix(0,0,mxREAL));
  }

  return(mxAutoinheritanceInfo);
}

mxArray *sf_c3_A5_TwoDOF_PID_torque_task_space_third_party_uses_info(void)
{
  mxArray * mxcell3p = mxCreateCellMatrix(1,0);
  return(mxcell3p);
}

mxArray *sf_c3_A5_TwoDOF_PID_torque_task_space_updateBuildInfo_args_info(void)
{
  mxArray *mxBIArgs = mxCreateCellMatrix(1,0);
  return mxBIArgs;
}

static const mxArray *sf_get_sim_state_info_c3_A5_TwoDOF_PID_torque_task_space
  (void)
{
  const char *infoFields[] = { "chartChecksum", "varInfo" };

  mxArray *mxInfo = mxCreateStructMatrix(1, 1, 2, infoFields);
  const char *infoEncStr[] = {
    "100 S1x2'type','srcId','name','auxInfo'{{M[1],M[5],T\"X\",},{M[8],M[0],T\"is_active_c3_A5_TwoDOF_PID_torque_task_space\",}}"
  };

  mxArray *mxVarInfo = sf_mex_decode_encoded_mx_struct_array(infoEncStr, 2, 10);
  mxArray *mxChecksum = mxCreateDoubleMatrix(1, 4, mxREAL);
  sf_c3_A5_TwoDOF_PID_torque_task_space_get_check_sum(&mxChecksum);
  mxSetField(mxInfo, 0, infoFields[0], mxChecksum);
  mxSetField(mxInfo, 0, infoFields[1], mxVarInfo);
  return mxInfo;
}

static void chart_debug_initialization(SimStruct *S, unsigned int
  fullDebuggerInitialization)
{
  if (!sim_mode_is_rtw_gen(S)) {
    SFc3_A5_TwoDOF_PID_torque_task_spaceInstanceStruct *chartInstance;
    ChartRunTimeInfo * crtInfo = (ChartRunTimeInfo *)(ssGetUserData(S));
    ChartInfoStruct * chartInfo = (ChartInfoStruct *)(crtInfo->instanceInfo);
    chartInstance = (SFc3_A5_TwoDOF_PID_torque_task_spaceInstanceStruct *)
      chartInfo->chartInstance;
    if (ssIsFirstInitCond(S) && fullDebuggerInitialization==1) {
      /* do this only if simulation is starting */
      {
        unsigned int chartAlreadyPresent;
        chartAlreadyPresent = sf_debug_initialize_chart
          (sfGlobalDebugInstanceStruct,
           _A5_TwoDOF_PID_torque_task_spaceMachineNumber_,
           3,
           1,
           1,
           0,
           2,
           0,
           0,
           0,
           0,
           0,
           &(chartInstance->chartNumber),
           &(chartInstance->instanceNumber),
           (void *)S);

        /* Each instance must initialize ist own list of scripts */
        init_script_number_translation
          (_A5_TwoDOF_PID_torque_task_spaceMachineNumber_,
           chartInstance->chartNumber,chartInstance->instanceNumber);
        if (chartAlreadyPresent==0) {
          /* this is the first instance */
          sf_debug_set_chart_disable_implicit_casting
            (sfGlobalDebugInstanceStruct,
             _A5_TwoDOF_PID_torque_task_spaceMachineNumber_,
             chartInstance->chartNumber,1);
          sf_debug_set_chart_event_thresholds(sfGlobalDebugInstanceStruct,
            _A5_TwoDOF_PID_torque_task_spaceMachineNumber_,
            chartInstance->chartNumber,
            0,
            0,
            0);
          _SFD_SET_DATA_PROPS(0,1,1,0,"Q");
          _SFD_SET_DATA_PROPS(1,2,0,1,"X");
          _SFD_STATE_INFO(0,0,2);
          _SFD_CH_SUBSTATE_COUNT(0);
          _SFD_CH_SUBSTATE_DECOMP(0);
        }

        _SFD_CV_INIT_CHART(0,0,0,0);

        {
          _SFD_CV_INIT_STATE(0,0,0,0,0,0,NULL,NULL);
        }

        _SFD_CV_INIT_TRANS(0,0,NULL,NULL,0,NULL);

        /* Initialization of MATLAB Function Model Coverage */
        _SFD_CV_INIT_EML(0,1,1,0,0,0,0,0,0,0,0);
        _SFD_CV_INIT_EML_FCN(0,0,"eML_blk_kernel",0,-1,159);

        {
          unsigned int dimVector[1];
          dimVector[0]= 2;
          _SFD_SET_DATA_COMPILED_PROPS(0,SF_DOUBLE,1,&(dimVector[0]),0,0,0,0.0,
            1.0,0,0,(MexFcnForType)c3_sf_marshallOut,(MexInFcnForType)NULL);
        }

        {
          unsigned int dimVector[1];
          dimVector[0]= 2;
          _SFD_SET_DATA_COMPILED_PROPS(1,SF_DOUBLE,1,&(dimVector[0]),0,0,0,0.0,
            1.0,0,0,(MexFcnForType)c3_sf_marshallOut,(MexInFcnForType)
            c3_sf_marshallIn);
        }

        {
          real_T (*c3_Q)[2];
          real_T (*c3_X)[2];
          c3_X = (real_T (*)[2])ssGetOutputPortSignal(chartInstance->S, 1);
          c3_Q = (real_T (*)[2])ssGetInputPortSignal(chartInstance->S, 0);
          _SFD_SET_DATA_VALUE_PTR(0U, *c3_Q);
          _SFD_SET_DATA_VALUE_PTR(1U, *c3_X);
        }
      }
    } else {
      sf_debug_reset_current_state_configuration(sfGlobalDebugInstanceStruct,
        _A5_TwoDOF_PID_torque_task_spaceMachineNumber_,
        chartInstance->chartNumber,chartInstance->instanceNumber);
    }
  }
}

static const char* sf_get_instance_specialization(void)
{
  return "y6E8SD3ScV9NGPer1QyCaG";
}

static void sf_opaque_initialize_c3_A5_TwoDOF_PID_torque_task_space(void
  *chartInstanceVar)
{
  chart_debug_initialization
    (((SFc3_A5_TwoDOF_PID_torque_task_spaceInstanceStruct*) chartInstanceVar)->S,
     0);
  initialize_params_c3_A5_TwoDOF_PID_torque_task_space
    ((SFc3_A5_TwoDOF_PID_torque_task_spaceInstanceStruct*) chartInstanceVar);
  initialize_c3_A5_TwoDOF_PID_torque_task_space
    ((SFc3_A5_TwoDOF_PID_torque_task_spaceInstanceStruct*) chartInstanceVar);
}

static void sf_opaque_enable_c3_A5_TwoDOF_PID_torque_task_space(void
  *chartInstanceVar)
{
  enable_c3_A5_TwoDOF_PID_torque_task_space
    ((SFc3_A5_TwoDOF_PID_torque_task_spaceInstanceStruct*) chartInstanceVar);
}

static void sf_opaque_disable_c3_A5_TwoDOF_PID_torque_task_space(void
  *chartInstanceVar)
{
  disable_c3_A5_TwoDOF_PID_torque_task_space
    ((SFc3_A5_TwoDOF_PID_torque_task_spaceInstanceStruct*) chartInstanceVar);
}

static void sf_opaque_gateway_c3_A5_TwoDOF_PID_torque_task_space(void
  *chartInstanceVar)
{
  sf_gateway_c3_A5_TwoDOF_PID_torque_task_space
    ((SFc3_A5_TwoDOF_PID_torque_task_spaceInstanceStruct*) chartInstanceVar);
}

extern const mxArray*
  sf_internal_get_sim_state_c3_A5_TwoDOF_PID_torque_task_space(SimStruct* S)
{
  ChartRunTimeInfo * crtInfo = (ChartRunTimeInfo *)(ssGetUserData(S));
  ChartInfoStruct * chartInfo = (ChartInfoStruct *)(crtInfo->instanceInfo);
  mxArray *plhs[1] = { NULL };

  mxArray *prhs[4];
  int mxError = 0;
  prhs[0] = mxCreateString("chart_simctx_raw2high");
  prhs[1] = mxCreateDoubleScalar(ssGetSFuncBlockHandle(S));
  prhs[2] = (mxArray*) get_sim_state_c3_A5_TwoDOF_PID_torque_task_space
    ((SFc3_A5_TwoDOF_PID_torque_task_spaceInstanceStruct*)
     chartInfo->chartInstance);        /* raw sim ctx */
  prhs[3] = (mxArray*) sf_get_sim_state_info_c3_A5_TwoDOF_PID_torque_task_space();/* state var info */
  mxError = sf_mex_call_matlab(1, plhs, 4, prhs, "sfprivate");
  mxDestroyArray(prhs[0]);
  mxDestroyArray(prhs[1]);
  mxDestroyArray(prhs[2]);
  mxDestroyArray(prhs[3]);
  if (mxError || plhs[0] == NULL) {
    sf_mex_error_message("Stateflow Internal Error: \nError calling 'chart_simctx_raw2high'.\n");
  }

  return plhs[0];
}

extern void sf_internal_set_sim_state_c3_A5_TwoDOF_PID_torque_task_space
  (SimStruct* S, const mxArray *st)
{
  ChartRunTimeInfo * crtInfo = (ChartRunTimeInfo *)(ssGetUserData(S));
  ChartInfoStruct * chartInfo = (ChartInfoStruct *)(crtInfo->instanceInfo);
  mxArray *plhs[1] = { NULL };

  mxArray *prhs[3];
  int mxError = 0;
  prhs[0] = mxCreateString("chart_simctx_high2raw");
  prhs[1] = mxDuplicateArray(st);      /* high level simctx */
  prhs[2] = (mxArray*) sf_get_sim_state_info_c3_A5_TwoDOF_PID_torque_task_space();/* state var info */
  mxError = sf_mex_call_matlab(1, plhs, 3, prhs, "sfprivate");
  mxDestroyArray(prhs[0]);
  mxDestroyArray(prhs[1]);
  mxDestroyArray(prhs[2]);
  if (mxError || plhs[0] == NULL) {
    sf_mex_error_message("Stateflow Internal Error: \nError calling 'chart_simctx_high2raw'.\n");
  }

  set_sim_state_c3_A5_TwoDOF_PID_torque_task_space
    ((SFc3_A5_TwoDOF_PID_torque_task_spaceInstanceStruct*)
     chartInfo->chartInstance, mxDuplicateArray(plhs[0]));
  mxDestroyArray(plhs[0]);
}

static const mxArray* sf_opaque_get_sim_state_c3_A5_TwoDOF_PID_torque_task_space
  (SimStruct* S)
{
  return sf_internal_get_sim_state_c3_A5_TwoDOF_PID_torque_task_space(S);
}

static void sf_opaque_set_sim_state_c3_A5_TwoDOF_PID_torque_task_space(SimStruct*
  S, const mxArray *st)
{
  sf_internal_set_sim_state_c3_A5_TwoDOF_PID_torque_task_space(S, st);
}

static void sf_opaque_terminate_c3_A5_TwoDOF_PID_torque_task_space(void
  *chartInstanceVar)
{
  if (chartInstanceVar!=NULL) {
    SimStruct *S = ((SFc3_A5_TwoDOF_PID_torque_task_spaceInstanceStruct*)
                    chartInstanceVar)->S;
    ChartRunTimeInfo * crtInfo = (ChartRunTimeInfo *)(ssGetUserData(S));
    if (sim_mode_is_rtw_gen(S) || sim_mode_is_external(S)) {
      sf_clear_rtw_identifier(S);
      unload_A5_TwoDOF_PID_torque_task_space_optimization_info();
    }

    finalize_c3_A5_TwoDOF_PID_torque_task_space
      ((SFc3_A5_TwoDOF_PID_torque_task_spaceInstanceStruct*) chartInstanceVar);
    utFree((void *)chartInstanceVar);
    if (crtInfo != NULL) {
      utFree((void *)crtInfo);
    }

    ssSetUserData(S,NULL);
  }
}

static void sf_opaque_init_subchart_simstructs(void *chartInstanceVar)
{
  initSimStructsc3_A5_TwoDOF_PID_torque_task_space
    ((SFc3_A5_TwoDOF_PID_torque_task_spaceInstanceStruct*) chartInstanceVar);
}

extern unsigned int sf_machine_global_initializer_called(void);
static void mdlProcessParameters_c3_A5_TwoDOF_PID_torque_task_space(SimStruct *S)
{
  int i;
  for (i=0;i<ssGetNumRunTimeParams(S);i++) {
    if (ssGetSFcnParamTunable(S,i)) {
      ssUpdateDlgParamAsRunTimeParam(S,i);
    }
  }

  if (sf_machine_global_initializer_called()) {
    ChartRunTimeInfo * crtInfo = (ChartRunTimeInfo *)(ssGetUserData(S));
    ChartInfoStruct * chartInfo = (ChartInfoStruct *)(crtInfo->instanceInfo);
    initialize_params_c3_A5_TwoDOF_PID_torque_task_space
      ((SFc3_A5_TwoDOF_PID_torque_task_spaceInstanceStruct*)
       (chartInfo->chartInstance));
  }
}

static void mdlSetWorkWidths_c3_A5_TwoDOF_PID_torque_task_space(SimStruct *S)
{
  if (sim_mode_is_rtw_gen(S) || sim_mode_is_external(S)) {
    mxArray *infoStruct = load_A5_TwoDOF_PID_torque_task_space_optimization_info
      ();
    int_T chartIsInlinable =
      (int_T)sf_is_chart_inlinable(sf_get_instance_specialization(),infoStruct,3);
    ssSetStateflowIsInlinable(S,chartIsInlinable);
    ssSetRTWCG(S,sf_rtw_info_uint_prop(sf_get_instance_specialization(),
                infoStruct,3,"RTWCG"));
    ssSetEnableFcnIsTrivial(S,1);
    ssSetDisableFcnIsTrivial(S,1);
    ssSetNotMultipleInlinable(S,sf_rtw_info_uint_prop
      (sf_get_instance_specialization(),infoStruct,3,
       "gatewayCannotBeInlinedMultipleTimes"));
    sf_update_buildInfo(sf_get_instance_specialization(),infoStruct,3);
    if (chartIsInlinable) {
      ssSetInputPortOptimOpts(S, 0, SS_REUSABLE_AND_LOCAL);
      sf_mark_chart_expressionable_inputs(S,sf_get_instance_specialization(),
        infoStruct,3,1);
      sf_mark_chart_reusable_outputs(S,sf_get_instance_specialization(),
        infoStruct,3,1);
    }

    {
      unsigned int outPortIdx;
      for (outPortIdx=1; outPortIdx<=1; ++outPortIdx) {
        ssSetOutputPortOptimizeInIR(S, outPortIdx, 1U);
      }
    }

    {
      unsigned int inPortIdx;
      for (inPortIdx=0; inPortIdx < 1; ++inPortIdx) {
        ssSetInputPortOptimizeInIR(S, inPortIdx, 1U);
      }
    }

    sf_set_rtw_dwork_info(S,sf_get_instance_specialization(),infoStruct,3);
    ssSetHasSubFunctions(S,!(chartIsInlinable));
  } else {
  }

  ssSetOptions(S,ssGetOptions(S)|SS_OPTION_WORKS_WITH_CODE_REUSE);
  ssSetChecksum0(S,(2506427820U));
  ssSetChecksum1(S,(2254834971U));
  ssSetChecksum2(S,(3206823980U));
  ssSetChecksum3(S,(1328450076U));
  ssSetmdlDerivatives(S, NULL);
  ssSetExplicitFCSSCtrl(S,1);
  ssSupportsMultipleExecInstances(S,1);
}

static void mdlRTW_c3_A5_TwoDOF_PID_torque_task_space(SimStruct *S)
{
  if (sim_mode_is_rtw_gen(S)) {
    ssWriteRTWStrParam(S, "StateflowChartType", "Embedded MATLAB");
  }
}

static void mdlStart_c3_A5_TwoDOF_PID_torque_task_space(SimStruct *S)
{
  SFc3_A5_TwoDOF_PID_torque_task_spaceInstanceStruct *chartInstance;
  ChartRunTimeInfo * crtInfo = (ChartRunTimeInfo *)utMalloc(sizeof
    (ChartRunTimeInfo));
  chartInstance = (SFc3_A5_TwoDOF_PID_torque_task_spaceInstanceStruct *)utMalloc
    (sizeof(SFc3_A5_TwoDOF_PID_torque_task_spaceInstanceStruct));
  memset(chartInstance, 0, sizeof
         (SFc3_A5_TwoDOF_PID_torque_task_spaceInstanceStruct));
  if (chartInstance==NULL) {
    sf_mex_error_message("Could not allocate memory for chart instance.");
  }

  chartInstance->chartInfo.chartInstance = chartInstance;
  chartInstance->chartInfo.isEMLChart = 1;
  chartInstance->chartInfo.chartInitialized = 0;
  chartInstance->chartInfo.sFunctionGateway =
    sf_opaque_gateway_c3_A5_TwoDOF_PID_torque_task_space;
  chartInstance->chartInfo.initializeChart =
    sf_opaque_initialize_c3_A5_TwoDOF_PID_torque_task_space;
  chartInstance->chartInfo.terminateChart =
    sf_opaque_terminate_c3_A5_TwoDOF_PID_torque_task_space;
  chartInstance->chartInfo.enableChart =
    sf_opaque_enable_c3_A5_TwoDOF_PID_torque_task_space;
  chartInstance->chartInfo.disableChart =
    sf_opaque_disable_c3_A5_TwoDOF_PID_torque_task_space;
  chartInstance->chartInfo.getSimState =
    sf_opaque_get_sim_state_c3_A5_TwoDOF_PID_torque_task_space;
  chartInstance->chartInfo.setSimState =
    sf_opaque_set_sim_state_c3_A5_TwoDOF_PID_torque_task_space;
  chartInstance->chartInfo.getSimStateInfo =
    sf_get_sim_state_info_c3_A5_TwoDOF_PID_torque_task_space;
  chartInstance->chartInfo.zeroCrossings = NULL;
  chartInstance->chartInfo.outputs = NULL;
  chartInstance->chartInfo.derivatives = NULL;
  chartInstance->chartInfo.mdlRTW = mdlRTW_c3_A5_TwoDOF_PID_torque_task_space;
  chartInstance->chartInfo.mdlStart =
    mdlStart_c3_A5_TwoDOF_PID_torque_task_space;
  chartInstance->chartInfo.mdlSetWorkWidths =
    mdlSetWorkWidths_c3_A5_TwoDOF_PID_torque_task_space;
  chartInstance->chartInfo.extModeExec = NULL;
  chartInstance->chartInfo.restoreLastMajorStepConfiguration = NULL;
  chartInstance->chartInfo.restoreBeforeLastMajorStepConfiguration = NULL;
  chartInstance->chartInfo.storeCurrentConfiguration = NULL;
  chartInstance->chartInfo.debugInstance = sfGlobalDebugInstanceStruct;
  chartInstance->S = S;
  crtInfo->instanceInfo = (&(chartInstance->chartInfo));
  crtInfo->isJITEnabled = false;
  ssSetUserData(S,(void *)(crtInfo));  /* register the chart instance with simstruct */
  init_dsm_address_info(chartInstance);
  if (!sim_mode_is_rtw_gen(S)) {
  }

  sf_opaque_init_subchart_simstructs(chartInstance->chartInfo.chartInstance);
  chart_debug_initialization(S,1);
}

void c3_A5_TwoDOF_PID_torque_task_space_method_dispatcher(SimStruct *S, int_T
  method, void *data)
{
  switch (method) {
   case SS_CALL_MDL_START:
    mdlStart_c3_A5_TwoDOF_PID_torque_task_space(S);
    break;

   case SS_CALL_MDL_SET_WORK_WIDTHS:
    mdlSetWorkWidths_c3_A5_TwoDOF_PID_torque_task_space(S);
    break;

   case SS_CALL_MDL_PROCESS_PARAMETERS:
    mdlProcessParameters_c3_A5_TwoDOF_PID_torque_task_space(S);
    break;

   default:
    /* Unhandled method */
    sf_mex_error_message("Stateflow Internal Error:\n"
                         "Error calling c3_A5_TwoDOF_PID_torque_task_space_method_dispatcher.\n"
                         "Can't handle method %d.\n", method);
    break;
  }
}
