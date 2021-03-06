/* Include files */

#include <stddef.h>
#include "blas.h"
#include "TwoDOF_PID_joint_space_sfun.h"
#include "c9_TwoDOF_PID_joint_space.h"
#include "mwmathutil.h"
#define CHARTINSTANCE_CHARTNUMBER      (chartInstance->chartNumber)
#define CHARTINSTANCE_INSTANCENUMBER   (chartInstance->instanceNumber)
#include "TwoDOF_PID_joint_space_sfun_debug_macros.h"
#define _SF_MEX_LISTEN_FOR_CTRL_C(S)   sf_mex_listen_for_ctrl_c(sfGlobalDebugInstanceStruct,S);

/* Type Definitions */

/* Named Constants */
#define CALL_EVENT                     (-1)

/* Variable Declarations */

/* Variable Definitions */
static real_T _sfTime_;
static const char * c9_debug_family_names[12] = { "C1", "S1", "C12", "S12", "L1",
  "L2", "J", "nargin", "nargout", "Q", "X", "G" };

/* Function Declarations */
static void initialize_c9_TwoDOF_PID_joint_space
  (SFc9_TwoDOF_PID_joint_spaceInstanceStruct *chartInstance);
static void initialize_params_c9_TwoDOF_PID_joint_space
  (SFc9_TwoDOF_PID_joint_spaceInstanceStruct *chartInstance);
static void enable_c9_TwoDOF_PID_joint_space
  (SFc9_TwoDOF_PID_joint_spaceInstanceStruct *chartInstance);
static void disable_c9_TwoDOF_PID_joint_space
  (SFc9_TwoDOF_PID_joint_spaceInstanceStruct *chartInstance);
static void c9_update_debugger_state_c9_TwoDOF_PID_joint_space
  (SFc9_TwoDOF_PID_joint_spaceInstanceStruct *chartInstance);
static const mxArray *get_sim_state_c9_TwoDOF_PID_joint_space
  (SFc9_TwoDOF_PID_joint_spaceInstanceStruct *chartInstance);
static void set_sim_state_c9_TwoDOF_PID_joint_space
  (SFc9_TwoDOF_PID_joint_spaceInstanceStruct *chartInstance, const mxArray
   *c9_st);
static void finalize_c9_TwoDOF_PID_joint_space
  (SFc9_TwoDOF_PID_joint_spaceInstanceStruct *chartInstance);
static void sf_gateway_c9_TwoDOF_PID_joint_space
  (SFc9_TwoDOF_PID_joint_spaceInstanceStruct *chartInstance);
static void initSimStructsc9_TwoDOF_PID_joint_space
  (SFc9_TwoDOF_PID_joint_spaceInstanceStruct *chartInstance);
static void init_script_number_translation(uint32_T c9_machineNumber, uint32_T
  c9_chartNumber, uint32_T c9_instanceNumber);
static const mxArray *c9_sf_marshallOut(void *chartInstanceVoid, void *c9_inData);
static void c9_emlrt_marshallIn(SFc9_TwoDOF_PID_joint_spaceInstanceStruct
  *chartInstance, const mxArray *c9_G, const char_T *c9_identifier, real_T c9_y
  [2]);
static void c9_b_emlrt_marshallIn(SFc9_TwoDOF_PID_joint_spaceInstanceStruct
  *chartInstance, const mxArray *c9_u, const emlrtMsgIdentifier *c9_parentId,
  real_T c9_y[2]);
static void c9_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c9_mxArrayInData, const char_T *c9_varName, void *c9_outData);
static const mxArray *c9_b_sf_marshallOut(void *chartInstanceVoid, void
  *c9_inData);
static real_T c9_c_emlrt_marshallIn(SFc9_TwoDOF_PID_joint_spaceInstanceStruct
  *chartInstance, const mxArray *c9_u, const emlrtMsgIdentifier *c9_parentId);
static void c9_b_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c9_mxArrayInData, const char_T *c9_varName, void *c9_outData);
static const mxArray *c9_c_sf_marshallOut(void *chartInstanceVoid, void
  *c9_inData);
static void c9_d_emlrt_marshallIn(SFc9_TwoDOF_PID_joint_spaceInstanceStruct
  *chartInstance, const mxArray *c9_u, const emlrtMsgIdentifier *c9_parentId,
  real_T c9_y[4]);
static void c9_c_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c9_mxArrayInData, const char_T *c9_varName, void *c9_outData);
static void c9_info_helper(const mxArray **c9_info);
static const mxArray *c9_emlrt_marshallOut(const char * c9_u);
static const mxArray *c9_b_emlrt_marshallOut(const uint32_T c9_u);
static void c9_eml_scalar_eg(SFc9_TwoDOF_PID_joint_spaceInstanceStruct
  *chartInstance);
static void c9_eml_xgemm(SFc9_TwoDOF_PID_joint_spaceInstanceStruct
  *chartInstance, real_T c9_A[4], real_T c9_B[2], real_T c9_C[2], real_T c9_b_C
  [2]);
static const mxArray *c9_d_sf_marshallOut(void *chartInstanceVoid, void
  *c9_inData);
static int32_T c9_e_emlrt_marshallIn(SFc9_TwoDOF_PID_joint_spaceInstanceStruct
  *chartInstance, const mxArray *c9_u, const emlrtMsgIdentifier *c9_parentId);
static void c9_d_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c9_mxArrayInData, const char_T *c9_varName, void *c9_outData);
static uint8_T c9_f_emlrt_marshallIn(SFc9_TwoDOF_PID_joint_spaceInstanceStruct
  *chartInstance, const mxArray *c9_b_is_active_c9_TwoDOF_PID_joint_space, const
  char_T *c9_identifier);
static uint8_T c9_g_emlrt_marshallIn(SFc9_TwoDOF_PID_joint_spaceInstanceStruct
  *chartInstance, const mxArray *c9_u, const emlrtMsgIdentifier *c9_parentId);
static void c9_b_eml_xgemm(SFc9_TwoDOF_PID_joint_spaceInstanceStruct
  *chartInstance, real_T c9_A[4], real_T c9_B[2], real_T c9_C[2]);
static void init_dsm_address_info(SFc9_TwoDOF_PID_joint_spaceInstanceStruct
  *chartInstance);

/* Function Definitions */
static void initialize_c9_TwoDOF_PID_joint_space
  (SFc9_TwoDOF_PID_joint_spaceInstanceStruct *chartInstance)
{
  chartInstance->c9_sfEvent = CALL_EVENT;
  _sfTime_ = sf_get_time(chartInstance->S);
  chartInstance->c9_is_active_c9_TwoDOF_PID_joint_space = 0U;
}

static void initialize_params_c9_TwoDOF_PID_joint_space
  (SFc9_TwoDOF_PID_joint_spaceInstanceStruct *chartInstance)
{
  (void)chartInstance;
}

static void enable_c9_TwoDOF_PID_joint_space
  (SFc9_TwoDOF_PID_joint_spaceInstanceStruct *chartInstance)
{
  _sfTime_ = sf_get_time(chartInstance->S);
}

static void disable_c9_TwoDOF_PID_joint_space
  (SFc9_TwoDOF_PID_joint_spaceInstanceStruct *chartInstance)
{
  _sfTime_ = sf_get_time(chartInstance->S);
}

static void c9_update_debugger_state_c9_TwoDOF_PID_joint_space
  (SFc9_TwoDOF_PID_joint_spaceInstanceStruct *chartInstance)
{
  (void)chartInstance;
}

static const mxArray *get_sim_state_c9_TwoDOF_PID_joint_space
  (SFc9_TwoDOF_PID_joint_spaceInstanceStruct *chartInstance)
{
  const mxArray *c9_st;
  const mxArray *c9_y = NULL;
  int32_T c9_i0;
  real_T c9_u[2];
  const mxArray *c9_b_y = NULL;
  uint8_T c9_hoistedGlobal;
  uint8_T c9_b_u;
  const mxArray *c9_c_y = NULL;
  real_T (*c9_G)[2];
  c9_G = (real_T (*)[2])ssGetOutputPortSignal(chartInstance->S, 1);
  c9_st = NULL;
  c9_st = NULL;
  c9_y = NULL;
  sf_mex_assign(&c9_y, sf_mex_createcellmatrix(2, 1), false);
  for (c9_i0 = 0; c9_i0 < 2; c9_i0++) {
    c9_u[c9_i0] = (*c9_G)[c9_i0];
  }

  c9_b_y = NULL;
  sf_mex_assign(&c9_b_y, sf_mex_create("y", c9_u, 0, 0U, 1U, 0U, 1, 2), false);
  sf_mex_setcell(c9_y, 0, c9_b_y);
  c9_hoistedGlobal = chartInstance->c9_is_active_c9_TwoDOF_PID_joint_space;
  c9_b_u = c9_hoistedGlobal;
  c9_c_y = NULL;
  sf_mex_assign(&c9_c_y, sf_mex_create("y", &c9_b_u, 3, 0U, 0U, 0U, 0), false);
  sf_mex_setcell(c9_y, 1, c9_c_y);
  sf_mex_assign(&c9_st, c9_y, false);
  return c9_st;
}

static void set_sim_state_c9_TwoDOF_PID_joint_space
  (SFc9_TwoDOF_PID_joint_spaceInstanceStruct *chartInstance, const mxArray
   *c9_st)
{
  const mxArray *c9_u;
  real_T c9_dv0[2];
  int32_T c9_i1;
  real_T (*c9_G)[2];
  c9_G = (real_T (*)[2])ssGetOutputPortSignal(chartInstance->S, 1);
  chartInstance->c9_doneDoubleBufferReInit = true;
  c9_u = sf_mex_dup(c9_st);
  c9_emlrt_marshallIn(chartInstance, sf_mex_dup(sf_mex_getcell(c9_u, 0)), "G",
                      c9_dv0);
  for (c9_i1 = 0; c9_i1 < 2; c9_i1++) {
    (*c9_G)[c9_i1] = c9_dv0[c9_i1];
  }

  chartInstance->c9_is_active_c9_TwoDOF_PID_joint_space = c9_f_emlrt_marshallIn
    (chartInstance, sf_mex_dup(sf_mex_getcell(c9_u, 1)),
     "is_active_c9_TwoDOF_PID_joint_space");
  sf_mex_destroy(&c9_u);
  c9_update_debugger_state_c9_TwoDOF_PID_joint_space(chartInstance);
  sf_mex_destroy(&c9_st);
}

static void finalize_c9_TwoDOF_PID_joint_space
  (SFc9_TwoDOF_PID_joint_spaceInstanceStruct *chartInstance)
{
  (void)chartInstance;
}

static void sf_gateway_c9_TwoDOF_PID_joint_space
  (SFc9_TwoDOF_PID_joint_spaceInstanceStruct *chartInstance)
{
  int32_T c9_i2;
  int32_T c9_i3;
  real_T c9_Q[2];
  int32_T c9_i4;
  real_T c9_X[2];
  uint32_T c9_debug_family_var_map[12];
  real_T c9_C1;
  real_T c9_S1;
  real_T c9_C12;
  real_T c9_S12;
  real_T c9_L1;
  real_T c9_L2;
  real_T c9_J[4];
  real_T c9_nargin = 2.0;
  real_T c9_nargout = 1.0;
  real_T c9_G[2];
  real_T c9_x;
  real_T c9_b_x;
  real_T c9_c_x;
  real_T c9_d_x;
  real_T c9_e_x;
  real_T c9_f_x;
  real_T c9_g_x;
  real_T c9_h_x;
  int32_T c9_i5;
  int32_T c9_i6;
  int32_T c9_i7;
  int32_T c9_i8;
  real_T c9_a[4];
  int32_T c9_i9;
  real_T c9_b[2];
  int32_T c9_i10;
  int32_T c9_i11;
  int32_T c9_i12;
  real_T c9_dv1[4];
  int32_T c9_i13;
  real_T c9_dv2[2];
  int32_T c9_i14;
  real_T c9_dv3[4];
  int32_T c9_i15;
  real_T c9_dv4[2];
  int32_T c9_i16;
  int32_T c9_i17;
  int32_T c9_i18;
  real_T (*c9_b_G)[2];
  real_T (*c9_b_X)[2];
  real_T (*c9_b_Q)[2];
  c9_b_X = (real_T (*)[2])ssGetInputPortSignal(chartInstance->S, 1);
  c9_b_G = (real_T (*)[2])ssGetOutputPortSignal(chartInstance->S, 1);
  c9_b_Q = (real_T (*)[2])ssGetInputPortSignal(chartInstance->S, 0);
  _SFD_SYMBOL_SCOPE_PUSH(0U, 0U);
  _sfTime_ = sf_get_time(chartInstance->S);
  _SFD_CC_CALL(CHART_ENTER_SFUNCTION_TAG, 1U, chartInstance->c9_sfEvent);
  for (c9_i2 = 0; c9_i2 < 2; c9_i2++) {
    _SFD_DATA_RANGE_CHECK((*c9_b_Q)[c9_i2], 0U);
  }

  chartInstance->c9_sfEvent = CALL_EVENT;
  _SFD_CC_CALL(CHART_ENTER_DURING_FUNCTION_TAG, 1U, chartInstance->c9_sfEvent);
  for (c9_i3 = 0; c9_i3 < 2; c9_i3++) {
    c9_Q[c9_i3] = (*c9_b_Q)[c9_i3];
  }

  for (c9_i4 = 0; c9_i4 < 2; c9_i4++) {
    c9_X[c9_i4] = (*c9_b_X)[c9_i4];
  }

  _SFD_SYMBOL_SCOPE_PUSH_EML(0U, 12U, 12U, c9_debug_family_names,
    c9_debug_family_var_map);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c9_C1, 0U, c9_b_sf_marshallOut,
    c9_b_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c9_S1, 1U, c9_b_sf_marshallOut,
    c9_b_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c9_C12, 2U, c9_b_sf_marshallOut,
    c9_b_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c9_S12, 3U, c9_b_sf_marshallOut,
    c9_b_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML(&c9_L1, 4U, c9_b_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML(&c9_L2, 5U, c9_b_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c9_J, 6U, c9_c_sf_marshallOut,
    c9_c_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c9_nargin, 7U, c9_b_sf_marshallOut,
    c9_b_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c9_nargout, 8U, c9_b_sf_marshallOut,
    c9_b_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML(c9_Q, 9U, c9_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML(c9_X, 10U, c9_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c9_G, 11U, c9_sf_marshallOut,
    c9_sf_marshallIn);
  CV_EML_FCN(0, 0);
  _SFD_EML_CALL(0U, chartInstance->c9_sfEvent, 3);
  c9_x = c9_Q[0];
  c9_C1 = c9_x;
  c9_b_x = c9_C1;
  c9_C1 = c9_b_x;
  c9_C1 = muDoubleScalarCos(c9_C1);
  _SFD_EML_CALL(0U, chartInstance->c9_sfEvent, 4);
  c9_c_x = c9_Q[0];
  c9_S1 = c9_c_x;
  c9_d_x = c9_S1;
  c9_S1 = c9_d_x;
  c9_S1 = muDoubleScalarSin(c9_S1);
  _SFD_EML_CALL(0U, chartInstance->c9_sfEvent, 5);
  c9_e_x = c9_Q[0] + c9_Q[1];
  c9_C12 = c9_e_x;
  c9_f_x = c9_C12;
  c9_C12 = c9_f_x;
  c9_C12 = muDoubleScalarCos(c9_C12);
  _SFD_EML_CALL(0U, chartInstance->c9_sfEvent, 6);
  c9_g_x = c9_Q[0] + c9_Q[1];
  c9_S12 = c9_g_x;
  c9_h_x = c9_S12;
  c9_S12 = c9_h_x;
  c9_S12 = muDoubleScalarSin(c9_S12);
  _SFD_EML_CALL(0U, chartInstance->c9_sfEvent, 8);
  c9_L1 = 0.2;
  _SFD_EML_CALL(0U, chartInstance->c9_sfEvent, 9);
  c9_L2 = 0.1442;
  _SFD_EML_CALL(0U, chartInstance->c9_sfEvent, 11);
  c9_J[0] = -0.2 * c9_S1 - 0.1442 * c9_S12;
  c9_J[2] = -0.1442 * c9_S12;
  c9_J[1] = 0.2 * c9_C1 + 0.1442 * c9_C12;
  c9_J[3] = 0.1442 * c9_C12;
  _SFD_EML_CALL(0U, chartInstance->c9_sfEvent, 13);
  c9_i5 = 0;
  for (c9_i6 = 0; c9_i6 < 2; c9_i6++) {
    c9_i7 = 0;
    for (c9_i8 = 0; c9_i8 < 2; c9_i8++) {
      c9_a[c9_i8 + c9_i5] = c9_J[c9_i7 + c9_i6];
      c9_i7 += 2;
    }

    c9_i5 += 2;
  }

  for (c9_i9 = 0; c9_i9 < 2; c9_i9++) {
    c9_b[c9_i9] = c9_X[c9_i9];
  }

  c9_eml_scalar_eg(chartInstance);
  c9_eml_scalar_eg(chartInstance);
  for (c9_i10 = 0; c9_i10 < 2; c9_i10++) {
    c9_G[c9_i10] = 0.0;
  }

  for (c9_i11 = 0; c9_i11 < 2; c9_i11++) {
    c9_G[c9_i11] = 0.0;
  }

  for (c9_i12 = 0; c9_i12 < 4; c9_i12++) {
    c9_dv1[c9_i12] = c9_a[c9_i12];
  }

  for (c9_i13 = 0; c9_i13 < 2; c9_i13++) {
    c9_dv2[c9_i13] = c9_b[c9_i13];
  }

  for (c9_i14 = 0; c9_i14 < 4; c9_i14++) {
    c9_dv3[c9_i14] = c9_dv1[c9_i14];
  }

  for (c9_i15 = 0; c9_i15 < 2; c9_i15++) {
    c9_dv4[c9_i15] = c9_dv2[c9_i15];
  }

  c9_b_eml_xgemm(chartInstance, c9_dv3, c9_dv4, c9_G);
  _SFD_EML_CALL(0U, chartInstance->c9_sfEvent, -13);
  _SFD_SYMBOL_SCOPE_POP();
  for (c9_i16 = 0; c9_i16 < 2; c9_i16++) {
    (*c9_b_G)[c9_i16] = c9_G[c9_i16];
  }

  _SFD_CC_CALL(EXIT_OUT_OF_FUNCTION_TAG, 1U, chartInstance->c9_sfEvent);
  _SFD_SYMBOL_SCOPE_POP();
  _SFD_CHECK_FOR_STATE_INCONSISTENCY(_TwoDOF_PID_joint_spaceMachineNumber_,
    chartInstance->chartNumber, chartInstance->instanceNumber);
  for (c9_i17 = 0; c9_i17 < 2; c9_i17++) {
    _SFD_DATA_RANGE_CHECK((*c9_b_G)[c9_i17], 1U);
  }

  for (c9_i18 = 0; c9_i18 < 2; c9_i18++) {
    _SFD_DATA_RANGE_CHECK((*c9_b_X)[c9_i18], 2U);
  }
}

static void initSimStructsc9_TwoDOF_PID_joint_space
  (SFc9_TwoDOF_PID_joint_spaceInstanceStruct *chartInstance)
{
  (void)chartInstance;
}

static void init_script_number_translation(uint32_T c9_machineNumber, uint32_T
  c9_chartNumber, uint32_T c9_instanceNumber)
{
  (void)c9_machineNumber;
  (void)c9_chartNumber;
  (void)c9_instanceNumber;
}

static const mxArray *c9_sf_marshallOut(void *chartInstanceVoid, void *c9_inData)
{
  const mxArray *c9_mxArrayOutData = NULL;
  int32_T c9_i19;
  real_T c9_b_inData[2];
  int32_T c9_i20;
  real_T c9_u[2];
  const mxArray *c9_y = NULL;
  SFc9_TwoDOF_PID_joint_spaceInstanceStruct *chartInstance;
  chartInstance = (SFc9_TwoDOF_PID_joint_spaceInstanceStruct *)chartInstanceVoid;
  c9_mxArrayOutData = NULL;
  for (c9_i19 = 0; c9_i19 < 2; c9_i19++) {
    c9_b_inData[c9_i19] = (*(real_T (*)[2])c9_inData)[c9_i19];
  }

  for (c9_i20 = 0; c9_i20 < 2; c9_i20++) {
    c9_u[c9_i20] = c9_b_inData[c9_i20];
  }

  c9_y = NULL;
  sf_mex_assign(&c9_y, sf_mex_create("y", c9_u, 0, 0U, 1U, 0U, 1, 2), false);
  sf_mex_assign(&c9_mxArrayOutData, c9_y, false);
  return c9_mxArrayOutData;
}

static void c9_emlrt_marshallIn(SFc9_TwoDOF_PID_joint_spaceInstanceStruct
  *chartInstance, const mxArray *c9_G, const char_T *c9_identifier, real_T c9_y
  [2])
{
  emlrtMsgIdentifier c9_thisId;
  c9_thisId.fIdentifier = c9_identifier;
  c9_thisId.fParent = NULL;
  c9_b_emlrt_marshallIn(chartInstance, sf_mex_dup(c9_G), &c9_thisId, c9_y);
  sf_mex_destroy(&c9_G);
}

static void c9_b_emlrt_marshallIn(SFc9_TwoDOF_PID_joint_spaceInstanceStruct
  *chartInstance, const mxArray *c9_u, const emlrtMsgIdentifier *c9_parentId,
  real_T c9_y[2])
{
  real_T c9_dv5[2];
  int32_T c9_i21;
  (void)chartInstance;
  sf_mex_import(c9_parentId, sf_mex_dup(c9_u), c9_dv5, 1, 0, 0U, 1, 0U, 1, 2);
  for (c9_i21 = 0; c9_i21 < 2; c9_i21++) {
    c9_y[c9_i21] = c9_dv5[c9_i21];
  }

  sf_mex_destroy(&c9_u);
}

static void c9_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c9_mxArrayInData, const char_T *c9_varName, void *c9_outData)
{
  const mxArray *c9_G;
  const char_T *c9_identifier;
  emlrtMsgIdentifier c9_thisId;
  real_T c9_y[2];
  int32_T c9_i22;
  SFc9_TwoDOF_PID_joint_spaceInstanceStruct *chartInstance;
  chartInstance = (SFc9_TwoDOF_PID_joint_spaceInstanceStruct *)chartInstanceVoid;
  c9_G = sf_mex_dup(c9_mxArrayInData);
  c9_identifier = c9_varName;
  c9_thisId.fIdentifier = c9_identifier;
  c9_thisId.fParent = NULL;
  c9_b_emlrt_marshallIn(chartInstance, sf_mex_dup(c9_G), &c9_thisId, c9_y);
  sf_mex_destroy(&c9_G);
  for (c9_i22 = 0; c9_i22 < 2; c9_i22++) {
    (*(real_T (*)[2])c9_outData)[c9_i22] = c9_y[c9_i22];
  }

  sf_mex_destroy(&c9_mxArrayInData);
}

static const mxArray *c9_b_sf_marshallOut(void *chartInstanceVoid, void
  *c9_inData)
{
  const mxArray *c9_mxArrayOutData = NULL;
  real_T c9_u;
  const mxArray *c9_y = NULL;
  SFc9_TwoDOF_PID_joint_spaceInstanceStruct *chartInstance;
  chartInstance = (SFc9_TwoDOF_PID_joint_spaceInstanceStruct *)chartInstanceVoid;
  c9_mxArrayOutData = NULL;
  c9_u = *(real_T *)c9_inData;
  c9_y = NULL;
  sf_mex_assign(&c9_y, sf_mex_create("y", &c9_u, 0, 0U, 0U, 0U, 0), false);
  sf_mex_assign(&c9_mxArrayOutData, c9_y, false);
  return c9_mxArrayOutData;
}

static real_T c9_c_emlrt_marshallIn(SFc9_TwoDOF_PID_joint_spaceInstanceStruct
  *chartInstance, const mxArray *c9_u, const emlrtMsgIdentifier *c9_parentId)
{
  real_T c9_y;
  real_T c9_d0;
  (void)chartInstance;
  sf_mex_import(c9_parentId, sf_mex_dup(c9_u), &c9_d0, 1, 0, 0U, 0, 0U, 0);
  c9_y = c9_d0;
  sf_mex_destroy(&c9_u);
  return c9_y;
}

static void c9_b_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c9_mxArrayInData, const char_T *c9_varName, void *c9_outData)
{
  const mxArray *c9_nargout;
  const char_T *c9_identifier;
  emlrtMsgIdentifier c9_thisId;
  real_T c9_y;
  SFc9_TwoDOF_PID_joint_spaceInstanceStruct *chartInstance;
  chartInstance = (SFc9_TwoDOF_PID_joint_spaceInstanceStruct *)chartInstanceVoid;
  c9_nargout = sf_mex_dup(c9_mxArrayInData);
  c9_identifier = c9_varName;
  c9_thisId.fIdentifier = c9_identifier;
  c9_thisId.fParent = NULL;
  c9_y = c9_c_emlrt_marshallIn(chartInstance, sf_mex_dup(c9_nargout), &c9_thisId);
  sf_mex_destroy(&c9_nargout);
  *(real_T *)c9_outData = c9_y;
  sf_mex_destroy(&c9_mxArrayInData);
}

static const mxArray *c9_c_sf_marshallOut(void *chartInstanceVoid, void
  *c9_inData)
{
  const mxArray *c9_mxArrayOutData = NULL;
  int32_T c9_i23;
  int32_T c9_i24;
  int32_T c9_i25;
  real_T c9_b_inData[4];
  int32_T c9_i26;
  int32_T c9_i27;
  int32_T c9_i28;
  real_T c9_u[4];
  const mxArray *c9_y = NULL;
  SFc9_TwoDOF_PID_joint_spaceInstanceStruct *chartInstance;
  chartInstance = (SFc9_TwoDOF_PID_joint_spaceInstanceStruct *)chartInstanceVoid;
  c9_mxArrayOutData = NULL;
  c9_i23 = 0;
  for (c9_i24 = 0; c9_i24 < 2; c9_i24++) {
    for (c9_i25 = 0; c9_i25 < 2; c9_i25++) {
      c9_b_inData[c9_i25 + c9_i23] = (*(real_T (*)[4])c9_inData)[c9_i25 + c9_i23];
    }

    c9_i23 += 2;
  }

  c9_i26 = 0;
  for (c9_i27 = 0; c9_i27 < 2; c9_i27++) {
    for (c9_i28 = 0; c9_i28 < 2; c9_i28++) {
      c9_u[c9_i28 + c9_i26] = c9_b_inData[c9_i28 + c9_i26];
    }

    c9_i26 += 2;
  }

  c9_y = NULL;
  sf_mex_assign(&c9_y, sf_mex_create("y", c9_u, 0, 0U, 1U, 0U, 2, 2, 2), false);
  sf_mex_assign(&c9_mxArrayOutData, c9_y, false);
  return c9_mxArrayOutData;
}

static void c9_d_emlrt_marshallIn(SFc9_TwoDOF_PID_joint_spaceInstanceStruct
  *chartInstance, const mxArray *c9_u, const emlrtMsgIdentifier *c9_parentId,
  real_T c9_y[4])
{
  real_T c9_dv6[4];
  int32_T c9_i29;
  (void)chartInstance;
  sf_mex_import(c9_parentId, sf_mex_dup(c9_u), c9_dv6, 1, 0, 0U, 1, 0U, 2, 2, 2);
  for (c9_i29 = 0; c9_i29 < 4; c9_i29++) {
    c9_y[c9_i29] = c9_dv6[c9_i29];
  }

  sf_mex_destroy(&c9_u);
}

static void c9_c_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c9_mxArrayInData, const char_T *c9_varName, void *c9_outData)
{
  const mxArray *c9_J;
  const char_T *c9_identifier;
  emlrtMsgIdentifier c9_thisId;
  real_T c9_y[4];
  int32_T c9_i30;
  int32_T c9_i31;
  int32_T c9_i32;
  SFc9_TwoDOF_PID_joint_spaceInstanceStruct *chartInstance;
  chartInstance = (SFc9_TwoDOF_PID_joint_spaceInstanceStruct *)chartInstanceVoid;
  c9_J = sf_mex_dup(c9_mxArrayInData);
  c9_identifier = c9_varName;
  c9_thisId.fIdentifier = c9_identifier;
  c9_thisId.fParent = NULL;
  c9_d_emlrt_marshallIn(chartInstance, sf_mex_dup(c9_J), &c9_thisId, c9_y);
  sf_mex_destroy(&c9_J);
  c9_i30 = 0;
  for (c9_i31 = 0; c9_i31 < 2; c9_i31++) {
    for (c9_i32 = 0; c9_i32 < 2; c9_i32++) {
      (*(real_T (*)[4])c9_outData)[c9_i32 + c9_i30] = c9_y[c9_i32 + c9_i30];
    }

    c9_i30 += 2;
  }

  sf_mex_destroy(&c9_mxArrayInData);
}

const mxArray *sf_c9_TwoDOF_PID_joint_space_get_eml_resolved_functions_info(void)
{
  const mxArray *c9_nameCaptureInfo = NULL;
  c9_nameCaptureInfo = NULL;
  sf_mex_assign(&c9_nameCaptureInfo, sf_mex_createstruct("structure", 2, 17, 1),
                false);
  c9_info_helper(&c9_nameCaptureInfo);
  sf_mex_emlrtNameCapturePostProcessR2012a(&c9_nameCaptureInfo);
  return c9_nameCaptureInfo;
}

static void c9_info_helper(const mxArray **c9_info)
{
  const mxArray *c9_rhs0 = NULL;
  const mxArray *c9_lhs0 = NULL;
  const mxArray *c9_rhs1 = NULL;
  const mxArray *c9_lhs1 = NULL;
  const mxArray *c9_rhs2 = NULL;
  const mxArray *c9_lhs2 = NULL;
  const mxArray *c9_rhs3 = NULL;
  const mxArray *c9_lhs3 = NULL;
  const mxArray *c9_rhs4 = NULL;
  const mxArray *c9_lhs4 = NULL;
  const mxArray *c9_rhs5 = NULL;
  const mxArray *c9_lhs5 = NULL;
  const mxArray *c9_rhs6 = NULL;
  const mxArray *c9_lhs6 = NULL;
  const mxArray *c9_rhs7 = NULL;
  const mxArray *c9_lhs7 = NULL;
  const mxArray *c9_rhs8 = NULL;
  const mxArray *c9_lhs8 = NULL;
  const mxArray *c9_rhs9 = NULL;
  const mxArray *c9_lhs9 = NULL;
  const mxArray *c9_rhs10 = NULL;
  const mxArray *c9_lhs10 = NULL;
  const mxArray *c9_rhs11 = NULL;
  const mxArray *c9_lhs11 = NULL;
  const mxArray *c9_rhs12 = NULL;
  const mxArray *c9_lhs12 = NULL;
  const mxArray *c9_rhs13 = NULL;
  const mxArray *c9_lhs13 = NULL;
  const mxArray *c9_rhs14 = NULL;
  const mxArray *c9_lhs14 = NULL;
  const mxArray *c9_rhs15 = NULL;
  const mxArray *c9_lhs15 = NULL;
  const mxArray *c9_rhs16 = NULL;
  const mxArray *c9_lhs16 = NULL;
  sf_mex_addfield(*c9_info, c9_emlrt_marshallOut(""), "context", "context", 0);
  sf_mex_addfield(*c9_info, c9_emlrt_marshallOut("cos"), "name", "name", 0);
  sf_mex_addfield(*c9_info, c9_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 0);
  sf_mex_addfield(*c9_info, c9_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/cos.m"), "resolved",
                  "resolved", 0);
  sf_mex_addfield(*c9_info, c9_b_emlrt_marshallOut(1343837572U), "fileTimeLo",
                  "fileTimeLo", 0);
  sf_mex_addfield(*c9_info, c9_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 0);
  sf_mex_addfield(*c9_info, c9_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 0);
  sf_mex_addfield(*c9_info, c9_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 0);
  sf_mex_assign(&c9_rhs0, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c9_lhs0, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c9_info, sf_mex_duplicatearraysafe(&c9_rhs0), "rhs", "rhs", 0);
  sf_mex_addfield(*c9_info, sf_mex_duplicatearraysafe(&c9_lhs0), "lhs", "lhs", 0);
  sf_mex_addfield(*c9_info, c9_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/cos.m"), "context",
                  "context", 1);
  sf_mex_addfield(*c9_info, c9_emlrt_marshallOut("eml_scalar_cos"), "name",
                  "name", 1);
  sf_mex_addfield(*c9_info, c9_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 1);
  sf_mex_addfield(*c9_info, c9_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/eml_scalar_cos.m"),
                  "resolved", "resolved", 1);
  sf_mex_addfield(*c9_info, c9_b_emlrt_marshallOut(1286825922U), "fileTimeLo",
                  "fileTimeLo", 1);
  sf_mex_addfield(*c9_info, c9_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 1);
  sf_mex_addfield(*c9_info, c9_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 1);
  sf_mex_addfield(*c9_info, c9_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 1);
  sf_mex_assign(&c9_rhs1, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c9_lhs1, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c9_info, sf_mex_duplicatearraysafe(&c9_rhs1), "rhs", "rhs", 1);
  sf_mex_addfield(*c9_info, sf_mex_duplicatearraysafe(&c9_lhs1), "lhs", "lhs", 1);
  sf_mex_addfield(*c9_info, c9_emlrt_marshallOut(""), "context", "context", 2);
  sf_mex_addfield(*c9_info, c9_emlrt_marshallOut("sin"), "name", "name", 2);
  sf_mex_addfield(*c9_info, c9_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 2);
  sf_mex_addfield(*c9_info, c9_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/sin.m"), "resolved",
                  "resolved", 2);
  sf_mex_addfield(*c9_info, c9_b_emlrt_marshallOut(1343837586U), "fileTimeLo",
                  "fileTimeLo", 2);
  sf_mex_addfield(*c9_info, c9_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 2);
  sf_mex_addfield(*c9_info, c9_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 2);
  sf_mex_addfield(*c9_info, c9_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 2);
  sf_mex_assign(&c9_rhs2, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c9_lhs2, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c9_info, sf_mex_duplicatearraysafe(&c9_rhs2), "rhs", "rhs", 2);
  sf_mex_addfield(*c9_info, sf_mex_duplicatearraysafe(&c9_lhs2), "lhs", "lhs", 2);
  sf_mex_addfield(*c9_info, c9_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/sin.m"), "context",
                  "context", 3);
  sf_mex_addfield(*c9_info, c9_emlrt_marshallOut("eml_scalar_sin"), "name",
                  "name", 3);
  sf_mex_addfield(*c9_info, c9_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 3);
  sf_mex_addfield(*c9_info, c9_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/eml_scalar_sin.m"),
                  "resolved", "resolved", 3);
  sf_mex_addfield(*c9_info, c9_b_emlrt_marshallOut(1286825936U), "fileTimeLo",
                  "fileTimeLo", 3);
  sf_mex_addfield(*c9_info, c9_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 3);
  sf_mex_addfield(*c9_info, c9_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 3);
  sf_mex_addfield(*c9_info, c9_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 3);
  sf_mex_assign(&c9_rhs3, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c9_lhs3, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c9_info, sf_mex_duplicatearraysafe(&c9_rhs3), "rhs", "rhs", 3);
  sf_mex_addfield(*c9_info, sf_mex_duplicatearraysafe(&c9_lhs3), "lhs", "lhs", 3);
  sf_mex_addfield(*c9_info, c9_emlrt_marshallOut(""), "context", "context", 4);
  sf_mex_addfield(*c9_info, c9_emlrt_marshallOut("eml_mtimes_helper"), "name",
                  "name", 4);
  sf_mex_addfield(*c9_info, c9_emlrt_marshallOut(""), "dominantType",
                  "dominantType", 4);
  sf_mex_addfield(*c9_info, c9_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/eml_mtimes_helper.m"),
                  "resolved", "resolved", 4);
  sf_mex_addfield(*c9_info, c9_b_emlrt_marshallOut(1383880894U), "fileTimeLo",
                  "fileTimeLo", 4);
  sf_mex_addfield(*c9_info, c9_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 4);
  sf_mex_addfield(*c9_info, c9_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 4);
  sf_mex_addfield(*c9_info, c9_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 4);
  sf_mex_assign(&c9_rhs4, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c9_lhs4, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c9_info, sf_mex_duplicatearraysafe(&c9_rhs4), "rhs", "rhs", 4);
  sf_mex_addfield(*c9_info, sf_mex_duplicatearraysafe(&c9_lhs4), "lhs", "lhs", 4);
  sf_mex_addfield(*c9_info, c9_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/eml_mtimes_helper.m!common_checks"),
                  "context", "context", 5);
  sf_mex_addfield(*c9_info, c9_emlrt_marshallOut(
    "coder.internal.isBuiltInNumeric"), "name", "name", 5);
  sf_mex_addfield(*c9_info, c9_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 5);
  sf_mex_addfield(*c9_info, c9_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/shared/coder/coder/+coder/+internal/isBuiltInNumeric.m"),
                  "resolved", "resolved", 5);
  sf_mex_addfield(*c9_info, c9_b_emlrt_marshallOut(1363718156U), "fileTimeLo",
                  "fileTimeLo", 5);
  sf_mex_addfield(*c9_info, c9_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 5);
  sf_mex_addfield(*c9_info, c9_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 5);
  sf_mex_addfield(*c9_info, c9_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 5);
  sf_mex_assign(&c9_rhs5, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c9_lhs5, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c9_info, sf_mex_duplicatearraysafe(&c9_rhs5), "rhs", "rhs", 5);
  sf_mex_addfield(*c9_info, sf_mex_duplicatearraysafe(&c9_lhs5), "lhs", "lhs", 5);
  sf_mex_addfield(*c9_info, c9_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/eml_mtimes_helper.m"),
                  "context", "context", 6);
  sf_mex_addfield(*c9_info, c9_emlrt_marshallOut("eml_index_class"), "name",
                  "name", 6);
  sf_mex_addfield(*c9_info, c9_emlrt_marshallOut(""), "dominantType",
                  "dominantType", 6);
  sf_mex_addfield(*c9_info, c9_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_class.m"),
                  "resolved", "resolved", 6);
  sf_mex_addfield(*c9_info, c9_b_emlrt_marshallOut(1323174178U), "fileTimeLo",
                  "fileTimeLo", 6);
  sf_mex_addfield(*c9_info, c9_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 6);
  sf_mex_addfield(*c9_info, c9_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 6);
  sf_mex_addfield(*c9_info, c9_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 6);
  sf_mex_assign(&c9_rhs6, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c9_lhs6, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c9_info, sf_mex_duplicatearraysafe(&c9_rhs6), "rhs", "rhs", 6);
  sf_mex_addfield(*c9_info, sf_mex_duplicatearraysafe(&c9_lhs6), "lhs", "lhs", 6);
  sf_mex_addfield(*c9_info, c9_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/eml_mtimes_helper.m"),
                  "context", "context", 7);
  sf_mex_addfield(*c9_info, c9_emlrt_marshallOut("eml_scalar_eg"), "name",
                  "name", 7);
  sf_mex_addfield(*c9_info, c9_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 7);
  sf_mex_addfield(*c9_info, c9_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalar_eg.m"), "resolved",
                  "resolved", 7);
  sf_mex_addfield(*c9_info, c9_b_emlrt_marshallOut(1375987888U), "fileTimeLo",
                  "fileTimeLo", 7);
  sf_mex_addfield(*c9_info, c9_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 7);
  sf_mex_addfield(*c9_info, c9_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 7);
  sf_mex_addfield(*c9_info, c9_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 7);
  sf_mex_assign(&c9_rhs7, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c9_lhs7, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c9_info, sf_mex_duplicatearraysafe(&c9_rhs7), "rhs", "rhs", 7);
  sf_mex_addfield(*c9_info, sf_mex_duplicatearraysafe(&c9_lhs7), "lhs", "lhs", 7);
  sf_mex_addfield(*c9_info, c9_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalar_eg.m"), "context",
                  "context", 8);
  sf_mex_addfield(*c9_info, c9_emlrt_marshallOut("coder.internal.scalarEg"),
                  "name", "name", 8);
  sf_mex_addfield(*c9_info, c9_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 8);
  sf_mex_addfield(*c9_info, c9_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/scalarEg.p"),
                  "resolved", "resolved", 8);
  sf_mex_addfield(*c9_info, c9_b_emlrt_marshallOut(1389311520U), "fileTimeLo",
                  "fileTimeLo", 8);
  sf_mex_addfield(*c9_info, c9_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 8);
  sf_mex_addfield(*c9_info, c9_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 8);
  sf_mex_addfield(*c9_info, c9_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 8);
  sf_mex_assign(&c9_rhs8, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c9_lhs8, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c9_info, sf_mex_duplicatearraysafe(&c9_rhs8), "rhs", "rhs", 8);
  sf_mex_addfield(*c9_info, sf_mex_duplicatearraysafe(&c9_lhs8), "lhs", "lhs", 8);
  sf_mex_addfield(*c9_info, c9_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/eml_mtimes_helper.m"),
                  "context", "context", 9);
  sf_mex_addfield(*c9_info, c9_emlrt_marshallOut("eml_xgemm"), "name", "name", 9);
  sf_mex_addfield(*c9_info, c9_emlrt_marshallOut("char"), "dominantType",
                  "dominantType", 9);
  sf_mex_addfield(*c9_info, c9_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/eml_xgemm.m"),
                  "resolved", "resolved", 9);
  sf_mex_addfield(*c9_info, c9_b_emlrt_marshallOut(1375987890U), "fileTimeLo",
                  "fileTimeLo", 9);
  sf_mex_addfield(*c9_info, c9_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 9);
  sf_mex_addfield(*c9_info, c9_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 9);
  sf_mex_addfield(*c9_info, c9_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 9);
  sf_mex_assign(&c9_rhs9, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c9_lhs9, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c9_info, sf_mex_duplicatearraysafe(&c9_rhs9), "rhs", "rhs", 9);
  sf_mex_addfield(*c9_info, sf_mex_duplicatearraysafe(&c9_lhs9), "lhs", "lhs", 9);
  sf_mex_addfield(*c9_info, c9_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/eml_xgemm.m"), "context",
                  "context", 10);
  sf_mex_addfield(*c9_info, c9_emlrt_marshallOut("coder.internal.blas.inline"),
                  "name", "name", 10);
  sf_mex_addfield(*c9_info, c9_emlrt_marshallOut(""), "dominantType",
                  "dominantType", 10);
  sf_mex_addfield(*c9_info, c9_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+blas/inline.p"),
                  "resolved", "resolved", 10);
  sf_mex_addfield(*c9_info, c9_b_emlrt_marshallOut(1389311522U), "fileTimeLo",
                  "fileTimeLo", 10);
  sf_mex_addfield(*c9_info, c9_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 10);
  sf_mex_addfield(*c9_info, c9_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 10);
  sf_mex_addfield(*c9_info, c9_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 10);
  sf_mex_assign(&c9_rhs10, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c9_lhs10, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c9_info, sf_mex_duplicatearraysafe(&c9_rhs10), "rhs", "rhs",
                  10);
  sf_mex_addfield(*c9_info, sf_mex_duplicatearraysafe(&c9_lhs10), "lhs", "lhs",
                  10);
  sf_mex_addfield(*c9_info, c9_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/eml_xgemm.m"), "context",
                  "context", 11);
  sf_mex_addfield(*c9_info, c9_emlrt_marshallOut("coder.internal.blas.xgemm"),
                  "name", "name", 11);
  sf_mex_addfield(*c9_info, c9_emlrt_marshallOut("char"), "dominantType",
                  "dominantType", 11);
  sf_mex_addfield(*c9_info, c9_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+blas/xgemm.p"),
                  "resolved", "resolved", 11);
  sf_mex_addfield(*c9_info, c9_b_emlrt_marshallOut(1389311522U), "fileTimeLo",
                  "fileTimeLo", 11);
  sf_mex_addfield(*c9_info, c9_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 11);
  sf_mex_addfield(*c9_info, c9_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 11);
  sf_mex_addfield(*c9_info, c9_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 11);
  sf_mex_assign(&c9_rhs11, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c9_lhs11, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c9_info, sf_mex_duplicatearraysafe(&c9_rhs11), "rhs", "rhs",
                  11);
  sf_mex_addfield(*c9_info, sf_mex_duplicatearraysafe(&c9_lhs11), "lhs", "lhs",
                  11);
  sf_mex_addfield(*c9_info, c9_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+blas/xgemm.p"),
                  "context", "context", 12);
  sf_mex_addfield(*c9_info, c9_emlrt_marshallOut(
    "coder.internal.blas.use_refblas"), "name", "name", 12);
  sf_mex_addfield(*c9_info, c9_emlrt_marshallOut(""), "dominantType",
                  "dominantType", 12);
  sf_mex_addfield(*c9_info, c9_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+blas/use_refblas.p"),
                  "resolved", "resolved", 12);
  sf_mex_addfield(*c9_info, c9_b_emlrt_marshallOut(1389311522U), "fileTimeLo",
                  "fileTimeLo", 12);
  sf_mex_addfield(*c9_info, c9_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 12);
  sf_mex_addfield(*c9_info, c9_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 12);
  sf_mex_addfield(*c9_info, c9_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 12);
  sf_mex_assign(&c9_rhs12, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c9_lhs12, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c9_info, sf_mex_duplicatearraysafe(&c9_rhs12), "rhs", "rhs",
                  12);
  sf_mex_addfield(*c9_info, sf_mex_duplicatearraysafe(&c9_lhs12), "lhs", "lhs",
                  12);
  sf_mex_addfield(*c9_info, c9_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+blas/xgemm.p!below_threshold"),
                  "context", "context", 13);
  sf_mex_addfield(*c9_info, c9_emlrt_marshallOut("coder.internal.blas.threshold"),
                  "name", "name", 13);
  sf_mex_addfield(*c9_info, c9_emlrt_marshallOut("char"), "dominantType",
                  "dominantType", 13);
  sf_mex_addfield(*c9_info, c9_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+blas/threshold.p"),
                  "resolved", "resolved", 13);
  sf_mex_addfield(*c9_info, c9_b_emlrt_marshallOut(1389311522U), "fileTimeLo",
                  "fileTimeLo", 13);
  sf_mex_addfield(*c9_info, c9_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 13);
  sf_mex_addfield(*c9_info, c9_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 13);
  sf_mex_addfield(*c9_info, c9_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 13);
  sf_mex_assign(&c9_rhs13, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c9_lhs13, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c9_info, sf_mex_duplicatearraysafe(&c9_rhs13), "rhs", "rhs",
                  13);
  sf_mex_addfield(*c9_info, sf_mex_duplicatearraysafe(&c9_lhs13), "lhs", "lhs",
                  13);
  sf_mex_addfield(*c9_info, c9_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+blas/threshold.p"),
                  "context", "context", 14);
  sf_mex_addfield(*c9_info, c9_emlrt_marshallOut("eml_switch_helper"), "name",
                  "name", 14);
  sf_mex_addfield(*c9_info, c9_emlrt_marshallOut(""), "dominantType",
                  "dominantType", 14);
  sf_mex_addfield(*c9_info, c9_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_switch_helper.m"),
                  "resolved", "resolved", 14);
  sf_mex_addfield(*c9_info, c9_b_emlrt_marshallOut(1381857500U), "fileTimeLo",
                  "fileTimeLo", 14);
  sf_mex_addfield(*c9_info, c9_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 14);
  sf_mex_addfield(*c9_info, c9_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 14);
  sf_mex_addfield(*c9_info, c9_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 14);
  sf_mex_assign(&c9_rhs14, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c9_lhs14, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c9_info, sf_mex_duplicatearraysafe(&c9_rhs14), "rhs", "rhs",
                  14);
  sf_mex_addfield(*c9_info, sf_mex_duplicatearraysafe(&c9_lhs14), "lhs", "lhs",
                  14);
  sf_mex_addfield(*c9_info, c9_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+blas/xgemm.p"),
                  "context", "context", 15);
  sf_mex_addfield(*c9_info, c9_emlrt_marshallOut("coder.internal.scalarEg"),
                  "name", "name", 15);
  sf_mex_addfield(*c9_info, c9_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 15);
  sf_mex_addfield(*c9_info, c9_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/scalarEg.p"),
                  "resolved", "resolved", 15);
  sf_mex_addfield(*c9_info, c9_b_emlrt_marshallOut(1389311520U), "fileTimeLo",
                  "fileTimeLo", 15);
  sf_mex_addfield(*c9_info, c9_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 15);
  sf_mex_addfield(*c9_info, c9_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 15);
  sf_mex_addfield(*c9_info, c9_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 15);
  sf_mex_assign(&c9_rhs15, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c9_lhs15, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c9_info, sf_mex_duplicatearraysafe(&c9_rhs15), "rhs", "rhs",
                  15);
  sf_mex_addfield(*c9_info, sf_mex_duplicatearraysafe(&c9_lhs15), "lhs", "lhs",
                  15);
  sf_mex_addfield(*c9_info, c9_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+blas/xgemm.p"),
                  "context", "context", 16);
  sf_mex_addfield(*c9_info, c9_emlrt_marshallOut("coder.internal.refblas.xgemm"),
                  "name", "name", 16);
  sf_mex_addfield(*c9_info, c9_emlrt_marshallOut("char"), "dominantType",
                  "dominantType", 16);
  sf_mex_addfield(*c9_info, c9_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+refblas/xgemm.p"),
                  "resolved", "resolved", 16);
  sf_mex_addfield(*c9_info, c9_b_emlrt_marshallOut(1389311522U), "fileTimeLo",
                  "fileTimeLo", 16);
  sf_mex_addfield(*c9_info, c9_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 16);
  sf_mex_addfield(*c9_info, c9_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 16);
  sf_mex_addfield(*c9_info, c9_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 16);
  sf_mex_assign(&c9_rhs16, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c9_lhs16, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c9_info, sf_mex_duplicatearraysafe(&c9_rhs16), "rhs", "rhs",
                  16);
  sf_mex_addfield(*c9_info, sf_mex_duplicatearraysafe(&c9_lhs16), "lhs", "lhs",
                  16);
  sf_mex_destroy(&c9_rhs0);
  sf_mex_destroy(&c9_lhs0);
  sf_mex_destroy(&c9_rhs1);
  sf_mex_destroy(&c9_lhs1);
  sf_mex_destroy(&c9_rhs2);
  sf_mex_destroy(&c9_lhs2);
  sf_mex_destroy(&c9_rhs3);
  sf_mex_destroy(&c9_lhs3);
  sf_mex_destroy(&c9_rhs4);
  sf_mex_destroy(&c9_lhs4);
  sf_mex_destroy(&c9_rhs5);
  sf_mex_destroy(&c9_lhs5);
  sf_mex_destroy(&c9_rhs6);
  sf_mex_destroy(&c9_lhs6);
  sf_mex_destroy(&c9_rhs7);
  sf_mex_destroy(&c9_lhs7);
  sf_mex_destroy(&c9_rhs8);
  sf_mex_destroy(&c9_lhs8);
  sf_mex_destroy(&c9_rhs9);
  sf_mex_destroy(&c9_lhs9);
  sf_mex_destroy(&c9_rhs10);
  sf_mex_destroy(&c9_lhs10);
  sf_mex_destroy(&c9_rhs11);
  sf_mex_destroy(&c9_lhs11);
  sf_mex_destroy(&c9_rhs12);
  sf_mex_destroy(&c9_lhs12);
  sf_mex_destroy(&c9_rhs13);
  sf_mex_destroy(&c9_lhs13);
  sf_mex_destroy(&c9_rhs14);
  sf_mex_destroy(&c9_lhs14);
  sf_mex_destroy(&c9_rhs15);
  sf_mex_destroy(&c9_lhs15);
  sf_mex_destroy(&c9_rhs16);
  sf_mex_destroy(&c9_lhs16);
}

static const mxArray *c9_emlrt_marshallOut(const char * c9_u)
{
  const mxArray *c9_y = NULL;
  c9_y = NULL;
  sf_mex_assign(&c9_y, sf_mex_create("y", c9_u, 15, 0U, 0U, 0U, 2, 1, strlen
    (c9_u)), false);
  return c9_y;
}

static const mxArray *c9_b_emlrt_marshallOut(const uint32_T c9_u)
{
  const mxArray *c9_y = NULL;
  c9_y = NULL;
  sf_mex_assign(&c9_y, sf_mex_create("y", &c9_u, 7, 0U, 0U, 0U, 0), false);
  return c9_y;
}

static void c9_eml_scalar_eg(SFc9_TwoDOF_PID_joint_spaceInstanceStruct
  *chartInstance)
{
  (void)chartInstance;
}

static void c9_eml_xgemm(SFc9_TwoDOF_PID_joint_spaceInstanceStruct
  *chartInstance, real_T c9_A[4], real_T c9_B[2], real_T c9_C[2], real_T c9_b_C
  [2])
{
  int32_T c9_i33;
  int32_T c9_i34;
  real_T c9_b_A[4];
  int32_T c9_i35;
  real_T c9_b_B[2];
  for (c9_i33 = 0; c9_i33 < 2; c9_i33++) {
    c9_b_C[c9_i33] = c9_C[c9_i33];
  }

  for (c9_i34 = 0; c9_i34 < 4; c9_i34++) {
    c9_b_A[c9_i34] = c9_A[c9_i34];
  }

  for (c9_i35 = 0; c9_i35 < 2; c9_i35++) {
    c9_b_B[c9_i35] = c9_B[c9_i35];
  }

  c9_b_eml_xgemm(chartInstance, c9_b_A, c9_b_B, c9_b_C);
}

static const mxArray *c9_d_sf_marshallOut(void *chartInstanceVoid, void
  *c9_inData)
{
  const mxArray *c9_mxArrayOutData = NULL;
  int32_T c9_u;
  const mxArray *c9_y = NULL;
  SFc9_TwoDOF_PID_joint_spaceInstanceStruct *chartInstance;
  chartInstance = (SFc9_TwoDOF_PID_joint_spaceInstanceStruct *)chartInstanceVoid;
  c9_mxArrayOutData = NULL;
  c9_u = *(int32_T *)c9_inData;
  c9_y = NULL;
  sf_mex_assign(&c9_y, sf_mex_create("y", &c9_u, 6, 0U, 0U, 0U, 0), false);
  sf_mex_assign(&c9_mxArrayOutData, c9_y, false);
  return c9_mxArrayOutData;
}

static int32_T c9_e_emlrt_marshallIn(SFc9_TwoDOF_PID_joint_spaceInstanceStruct
  *chartInstance, const mxArray *c9_u, const emlrtMsgIdentifier *c9_parentId)
{
  int32_T c9_y;
  int32_T c9_i36;
  (void)chartInstance;
  sf_mex_import(c9_parentId, sf_mex_dup(c9_u), &c9_i36, 1, 6, 0U, 0, 0U, 0);
  c9_y = c9_i36;
  sf_mex_destroy(&c9_u);
  return c9_y;
}

static void c9_d_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c9_mxArrayInData, const char_T *c9_varName, void *c9_outData)
{
  const mxArray *c9_b_sfEvent;
  const char_T *c9_identifier;
  emlrtMsgIdentifier c9_thisId;
  int32_T c9_y;
  SFc9_TwoDOF_PID_joint_spaceInstanceStruct *chartInstance;
  chartInstance = (SFc9_TwoDOF_PID_joint_spaceInstanceStruct *)chartInstanceVoid;
  c9_b_sfEvent = sf_mex_dup(c9_mxArrayInData);
  c9_identifier = c9_varName;
  c9_thisId.fIdentifier = c9_identifier;
  c9_thisId.fParent = NULL;
  c9_y = c9_e_emlrt_marshallIn(chartInstance, sf_mex_dup(c9_b_sfEvent),
    &c9_thisId);
  sf_mex_destroy(&c9_b_sfEvent);
  *(int32_T *)c9_outData = c9_y;
  sf_mex_destroy(&c9_mxArrayInData);
}

static uint8_T c9_f_emlrt_marshallIn(SFc9_TwoDOF_PID_joint_spaceInstanceStruct
  *chartInstance, const mxArray *c9_b_is_active_c9_TwoDOF_PID_joint_space, const
  char_T *c9_identifier)
{
  uint8_T c9_y;
  emlrtMsgIdentifier c9_thisId;
  c9_thisId.fIdentifier = c9_identifier;
  c9_thisId.fParent = NULL;
  c9_y = c9_g_emlrt_marshallIn(chartInstance, sf_mex_dup
    (c9_b_is_active_c9_TwoDOF_PID_joint_space), &c9_thisId);
  sf_mex_destroy(&c9_b_is_active_c9_TwoDOF_PID_joint_space);
  return c9_y;
}

static uint8_T c9_g_emlrt_marshallIn(SFc9_TwoDOF_PID_joint_spaceInstanceStruct
  *chartInstance, const mxArray *c9_u, const emlrtMsgIdentifier *c9_parentId)
{
  uint8_T c9_y;
  uint8_T c9_u0;
  (void)chartInstance;
  sf_mex_import(c9_parentId, sf_mex_dup(c9_u), &c9_u0, 1, 3, 0U, 0, 0U, 0);
  c9_y = c9_u0;
  sf_mex_destroy(&c9_u);
  return c9_y;
}

static void c9_b_eml_xgemm(SFc9_TwoDOF_PID_joint_spaceInstanceStruct
  *chartInstance, real_T c9_A[4], real_T c9_B[2], real_T c9_C[2])
{
  int32_T c9_i37;
  int32_T c9_i38;
  int32_T c9_i39;
  (void)chartInstance;
  for (c9_i37 = 0; c9_i37 < 2; c9_i37++) {
    c9_C[c9_i37] = 0.0;
    c9_i38 = 0;
    for (c9_i39 = 0; c9_i39 < 2; c9_i39++) {
      c9_C[c9_i37] += c9_A[c9_i38 + c9_i37] * c9_B[c9_i39];
      c9_i38 += 2;
    }
  }
}

static void init_dsm_address_info(SFc9_TwoDOF_PID_joint_spaceInstanceStruct
  *chartInstance)
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

void sf_c9_TwoDOF_PID_joint_space_get_check_sum(mxArray *plhs[])
{
  ((real_T *)mxGetPr((plhs[0])))[0] = (real_T)(2985556425U);
  ((real_T *)mxGetPr((plhs[0])))[1] = (real_T)(2565674376U);
  ((real_T *)mxGetPr((plhs[0])))[2] = (real_T)(2158323446U);
  ((real_T *)mxGetPr((plhs[0])))[3] = (real_T)(4061285166U);
}

mxArray *sf_c9_TwoDOF_PID_joint_space_get_autoinheritance_info(void)
{
  const char *autoinheritanceFields[] = { "checksum", "inputs", "parameters",
    "outputs", "locals" };

  mxArray *mxAutoinheritanceInfo = mxCreateStructMatrix(1,1,5,
    autoinheritanceFields);

  {
    mxArray *mxChecksum = mxCreateString("iMbzw5BimdeIoShrbYywdG");
    mxSetField(mxAutoinheritanceInfo,0,"checksum",mxChecksum);
  }

  {
    const char *dataFields[] = { "size", "type", "complexity" };

    mxArray *mxData = mxCreateStructMatrix(1,2,3,dataFields);

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

    {
      mxArray *mxSize = mxCreateDoubleMatrix(1,2,mxREAL);
      double *pr = mxGetPr(mxSize);
      pr[0] = (double)(2);
      pr[1] = (double)(1);
      mxSetField(mxData,1,"size",mxSize);
    }

    {
      const char *typeFields[] = { "base", "fixpt" };

      mxArray *mxType = mxCreateStructMatrix(1,1,2,typeFields);
      mxSetField(mxType,0,"base",mxCreateDoubleScalar(10));
      mxSetField(mxType,0,"fixpt",mxCreateDoubleMatrix(0,0,mxREAL));
      mxSetField(mxData,1,"type",mxType);
    }

    mxSetField(mxData,1,"complexity",mxCreateDoubleScalar(0));
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

mxArray *sf_c9_TwoDOF_PID_joint_space_third_party_uses_info(void)
{
  mxArray * mxcell3p = mxCreateCellMatrix(1,0);
  return(mxcell3p);
}

mxArray *sf_c9_TwoDOF_PID_joint_space_updateBuildInfo_args_info(void)
{
  mxArray *mxBIArgs = mxCreateCellMatrix(1,0);
  return mxBIArgs;
}

static const mxArray *sf_get_sim_state_info_c9_TwoDOF_PID_joint_space(void)
{
  const char *infoFields[] = { "chartChecksum", "varInfo" };

  mxArray *mxInfo = mxCreateStructMatrix(1, 1, 2, infoFields);
  const char *infoEncStr[] = {
    "100 S1x2'type','srcId','name','auxInfo'{{M[1],M[5],T\"G\",},{M[8],M[0],T\"is_active_c9_TwoDOF_PID_joint_space\",}}"
  };

  mxArray *mxVarInfo = sf_mex_decode_encoded_mx_struct_array(infoEncStr, 2, 10);
  mxArray *mxChecksum = mxCreateDoubleMatrix(1, 4, mxREAL);
  sf_c9_TwoDOF_PID_joint_space_get_check_sum(&mxChecksum);
  mxSetField(mxInfo, 0, infoFields[0], mxChecksum);
  mxSetField(mxInfo, 0, infoFields[1], mxVarInfo);
  return mxInfo;
}

static void chart_debug_initialization(SimStruct *S, unsigned int
  fullDebuggerInitialization)
{
  if (!sim_mode_is_rtw_gen(S)) {
    SFc9_TwoDOF_PID_joint_spaceInstanceStruct *chartInstance;
    ChartRunTimeInfo * crtInfo = (ChartRunTimeInfo *)(ssGetUserData(S));
    ChartInfoStruct * chartInfo = (ChartInfoStruct *)(crtInfo->instanceInfo);
    chartInstance = (SFc9_TwoDOF_PID_joint_spaceInstanceStruct *)
      chartInfo->chartInstance;
    if (ssIsFirstInitCond(S) && fullDebuggerInitialization==1) {
      /* do this only if simulation is starting */
      {
        unsigned int chartAlreadyPresent;
        chartAlreadyPresent = sf_debug_initialize_chart
          (sfGlobalDebugInstanceStruct,
           _TwoDOF_PID_joint_spaceMachineNumber_,
           9,
           1,
           1,
           0,
           3,
           0,
           0,
           0,
           0,
           0,
           &(chartInstance->chartNumber),
           &(chartInstance->instanceNumber),
           (void *)S);

        /* Each instance must initialize ist own list of scripts */
        init_script_number_translation(_TwoDOF_PID_joint_spaceMachineNumber_,
          chartInstance->chartNumber,chartInstance->instanceNumber);
        if (chartAlreadyPresent==0) {
          /* this is the first instance */
          sf_debug_set_chart_disable_implicit_casting
            (sfGlobalDebugInstanceStruct,_TwoDOF_PID_joint_spaceMachineNumber_,
             chartInstance->chartNumber,1);
          sf_debug_set_chart_event_thresholds(sfGlobalDebugInstanceStruct,
            _TwoDOF_PID_joint_spaceMachineNumber_,
            chartInstance->chartNumber,
            0,
            0,
            0);
          _SFD_SET_DATA_PROPS(0,1,1,0,"Q");
          _SFD_SET_DATA_PROPS(1,2,0,1,"G");
          _SFD_SET_DATA_PROPS(2,1,1,0,"X");
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
        _SFD_CV_INIT_EML_FCN(0,0,"eML_blk_kernel",0,-1,200);

        {
          unsigned int dimVector[1];
          dimVector[0]= 2;
          _SFD_SET_DATA_COMPILED_PROPS(0,SF_DOUBLE,1,&(dimVector[0]),0,0,0,0.0,
            1.0,0,0,(MexFcnForType)c9_sf_marshallOut,(MexInFcnForType)NULL);
        }

        {
          unsigned int dimVector[1];
          dimVector[0]= 2;
          _SFD_SET_DATA_COMPILED_PROPS(1,SF_DOUBLE,1,&(dimVector[0]),0,0,0,0.0,
            1.0,0,0,(MexFcnForType)c9_sf_marshallOut,(MexInFcnForType)
            c9_sf_marshallIn);
        }

        {
          unsigned int dimVector[1];
          dimVector[0]= 2;
          _SFD_SET_DATA_COMPILED_PROPS(2,SF_DOUBLE,1,&(dimVector[0]),0,0,0,0.0,
            1.0,0,0,(MexFcnForType)c9_sf_marshallOut,(MexInFcnForType)NULL);
        }

        {
          real_T (*c9_Q)[2];
          real_T (*c9_G)[2];
          real_T (*c9_X)[2];
          c9_X = (real_T (*)[2])ssGetInputPortSignal(chartInstance->S, 1);
          c9_G = (real_T (*)[2])ssGetOutputPortSignal(chartInstance->S, 1);
          c9_Q = (real_T (*)[2])ssGetInputPortSignal(chartInstance->S, 0);
          _SFD_SET_DATA_VALUE_PTR(0U, *c9_Q);
          _SFD_SET_DATA_VALUE_PTR(1U, *c9_G);
          _SFD_SET_DATA_VALUE_PTR(2U, *c9_X);
        }
      }
    } else {
      sf_debug_reset_current_state_configuration(sfGlobalDebugInstanceStruct,
        _TwoDOF_PID_joint_spaceMachineNumber_,chartInstance->chartNumber,
        chartInstance->instanceNumber);
    }
  }
}

static const char* sf_get_instance_specialization(void)
{
  return "7jmHyeqnFcN4ekjywyImE";
}

static void sf_opaque_initialize_c9_TwoDOF_PID_joint_space(void
  *chartInstanceVar)
{
  chart_debug_initialization(((SFc9_TwoDOF_PID_joint_spaceInstanceStruct*)
    chartInstanceVar)->S,0);
  initialize_params_c9_TwoDOF_PID_joint_space
    ((SFc9_TwoDOF_PID_joint_spaceInstanceStruct*) chartInstanceVar);
  initialize_c9_TwoDOF_PID_joint_space
    ((SFc9_TwoDOF_PID_joint_spaceInstanceStruct*) chartInstanceVar);
}

static void sf_opaque_enable_c9_TwoDOF_PID_joint_space(void *chartInstanceVar)
{
  enable_c9_TwoDOF_PID_joint_space((SFc9_TwoDOF_PID_joint_spaceInstanceStruct*)
    chartInstanceVar);
}

static void sf_opaque_disable_c9_TwoDOF_PID_joint_space(void *chartInstanceVar)
{
  disable_c9_TwoDOF_PID_joint_space((SFc9_TwoDOF_PID_joint_spaceInstanceStruct*)
    chartInstanceVar);
}

static void sf_opaque_gateway_c9_TwoDOF_PID_joint_space(void *chartInstanceVar)
{
  sf_gateway_c9_TwoDOF_PID_joint_space
    ((SFc9_TwoDOF_PID_joint_spaceInstanceStruct*) chartInstanceVar);
}

extern const mxArray* sf_internal_get_sim_state_c9_TwoDOF_PID_joint_space
  (SimStruct* S)
{
  ChartRunTimeInfo * crtInfo = (ChartRunTimeInfo *)(ssGetUserData(S));
  ChartInfoStruct * chartInfo = (ChartInfoStruct *)(crtInfo->instanceInfo);
  mxArray *plhs[1] = { NULL };

  mxArray *prhs[4];
  int mxError = 0;
  prhs[0] = mxCreateString("chart_simctx_raw2high");
  prhs[1] = mxCreateDoubleScalar(ssGetSFuncBlockHandle(S));
  prhs[2] = (mxArray*) get_sim_state_c9_TwoDOF_PID_joint_space
    ((SFc9_TwoDOF_PID_joint_spaceInstanceStruct*)chartInfo->chartInstance);/* raw sim ctx */
  prhs[3] = (mxArray*) sf_get_sim_state_info_c9_TwoDOF_PID_joint_space();/* state var info */
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

extern void sf_internal_set_sim_state_c9_TwoDOF_PID_joint_space(SimStruct* S,
  const mxArray *st)
{
  ChartRunTimeInfo * crtInfo = (ChartRunTimeInfo *)(ssGetUserData(S));
  ChartInfoStruct * chartInfo = (ChartInfoStruct *)(crtInfo->instanceInfo);
  mxArray *plhs[1] = { NULL };

  mxArray *prhs[3];
  int mxError = 0;
  prhs[0] = mxCreateString("chart_simctx_high2raw");
  prhs[1] = mxDuplicateArray(st);      /* high level simctx */
  prhs[2] = (mxArray*) sf_get_sim_state_info_c9_TwoDOF_PID_joint_space();/* state var info */
  mxError = sf_mex_call_matlab(1, plhs, 3, prhs, "sfprivate");
  mxDestroyArray(prhs[0]);
  mxDestroyArray(prhs[1]);
  mxDestroyArray(prhs[2]);
  if (mxError || plhs[0] == NULL) {
    sf_mex_error_message("Stateflow Internal Error: \nError calling 'chart_simctx_high2raw'.\n");
  }

  set_sim_state_c9_TwoDOF_PID_joint_space
    ((SFc9_TwoDOF_PID_joint_spaceInstanceStruct*)chartInfo->chartInstance,
     mxDuplicateArray(plhs[0]));
  mxDestroyArray(plhs[0]);
}

static const mxArray* sf_opaque_get_sim_state_c9_TwoDOF_PID_joint_space
  (SimStruct* S)
{
  return sf_internal_get_sim_state_c9_TwoDOF_PID_joint_space(S);
}

static void sf_opaque_set_sim_state_c9_TwoDOF_PID_joint_space(SimStruct* S,
  const mxArray *st)
{
  sf_internal_set_sim_state_c9_TwoDOF_PID_joint_space(S, st);
}

static void sf_opaque_terminate_c9_TwoDOF_PID_joint_space(void *chartInstanceVar)
{
  if (chartInstanceVar!=NULL) {
    SimStruct *S = ((SFc9_TwoDOF_PID_joint_spaceInstanceStruct*)
                    chartInstanceVar)->S;
    ChartRunTimeInfo * crtInfo = (ChartRunTimeInfo *)(ssGetUserData(S));
    if (sim_mode_is_rtw_gen(S) || sim_mode_is_external(S)) {
      sf_clear_rtw_identifier(S);
      unload_TwoDOF_PID_joint_space_optimization_info();
    }

    finalize_c9_TwoDOF_PID_joint_space
      ((SFc9_TwoDOF_PID_joint_spaceInstanceStruct*) chartInstanceVar);
    utFree((void *)chartInstanceVar);
    if (crtInfo != NULL) {
      utFree((void *)crtInfo);
    }

    ssSetUserData(S,NULL);
  }
}

static void sf_opaque_init_subchart_simstructs(void *chartInstanceVar)
{
  initSimStructsc9_TwoDOF_PID_joint_space
    ((SFc9_TwoDOF_PID_joint_spaceInstanceStruct*) chartInstanceVar);
}

extern unsigned int sf_machine_global_initializer_called(void);
static void mdlProcessParameters_c9_TwoDOF_PID_joint_space(SimStruct *S)
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
    initialize_params_c9_TwoDOF_PID_joint_space
      ((SFc9_TwoDOF_PID_joint_spaceInstanceStruct*)(chartInfo->chartInstance));
  }
}

static void mdlSetWorkWidths_c9_TwoDOF_PID_joint_space(SimStruct *S)
{
  if (sim_mode_is_rtw_gen(S) || sim_mode_is_external(S)) {
    mxArray *infoStruct = load_TwoDOF_PID_joint_space_optimization_info();
    int_T chartIsInlinable =
      (int_T)sf_is_chart_inlinable(sf_get_instance_specialization(),infoStruct,9);
    ssSetStateflowIsInlinable(S,chartIsInlinable);
    ssSetRTWCG(S,sf_rtw_info_uint_prop(sf_get_instance_specialization(),
                infoStruct,9,"RTWCG"));
    ssSetEnableFcnIsTrivial(S,1);
    ssSetDisableFcnIsTrivial(S,1);
    ssSetNotMultipleInlinable(S,sf_rtw_info_uint_prop
      (sf_get_instance_specialization(),infoStruct,9,
       "gatewayCannotBeInlinedMultipleTimes"));
    sf_update_buildInfo(sf_get_instance_specialization(),infoStruct,9);
    if (chartIsInlinable) {
      ssSetInputPortOptimOpts(S, 0, SS_REUSABLE_AND_LOCAL);
      ssSetInputPortOptimOpts(S, 1, SS_REUSABLE_AND_LOCAL);
      sf_mark_chart_expressionable_inputs(S,sf_get_instance_specialization(),
        infoStruct,9,2);
      sf_mark_chart_reusable_outputs(S,sf_get_instance_specialization(),
        infoStruct,9,1);
    }

    {
      unsigned int outPortIdx;
      for (outPortIdx=1; outPortIdx<=1; ++outPortIdx) {
        ssSetOutputPortOptimizeInIR(S, outPortIdx, 1U);
      }
    }

    {
      unsigned int inPortIdx;
      for (inPortIdx=0; inPortIdx < 2; ++inPortIdx) {
        ssSetInputPortOptimizeInIR(S, inPortIdx, 1U);
      }
    }

    sf_set_rtw_dwork_info(S,sf_get_instance_specialization(),infoStruct,9);
    ssSetHasSubFunctions(S,!(chartIsInlinable));
  } else {
  }

  ssSetOptions(S,ssGetOptions(S)|SS_OPTION_WORKS_WITH_CODE_REUSE);
  ssSetChecksum0(S,(2741939827U));
  ssSetChecksum1(S,(4286473807U));
  ssSetChecksum2(S,(2877217746U));
  ssSetChecksum3(S,(3413817737U));
  ssSetmdlDerivatives(S, NULL);
  ssSetExplicitFCSSCtrl(S,1);
  ssSupportsMultipleExecInstances(S,1);
}

static void mdlRTW_c9_TwoDOF_PID_joint_space(SimStruct *S)
{
  if (sim_mode_is_rtw_gen(S)) {
    ssWriteRTWStrParam(S, "StateflowChartType", "Embedded MATLAB");
  }
}

static void mdlStart_c9_TwoDOF_PID_joint_space(SimStruct *S)
{
  SFc9_TwoDOF_PID_joint_spaceInstanceStruct *chartInstance;
  ChartRunTimeInfo * crtInfo = (ChartRunTimeInfo *)utMalloc(sizeof
    (ChartRunTimeInfo));
  chartInstance = (SFc9_TwoDOF_PID_joint_spaceInstanceStruct *)utMalloc(sizeof
    (SFc9_TwoDOF_PID_joint_spaceInstanceStruct));
  memset(chartInstance, 0, sizeof(SFc9_TwoDOF_PID_joint_spaceInstanceStruct));
  if (chartInstance==NULL) {
    sf_mex_error_message("Could not allocate memory for chart instance.");
  }

  chartInstance->chartInfo.chartInstance = chartInstance;
  chartInstance->chartInfo.isEMLChart = 1;
  chartInstance->chartInfo.chartInitialized = 0;
  chartInstance->chartInfo.sFunctionGateway =
    sf_opaque_gateway_c9_TwoDOF_PID_joint_space;
  chartInstance->chartInfo.initializeChart =
    sf_opaque_initialize_c9_TwoDOF_PID_joint_space;
  chartInstance->chartInfo.terminateChart =
    sf_opaque_terminate_c9_TwoDOF_PID_joint_space;
  chartInstance->chartInfo.enableChart =
    sf_opaque_enable_c9_TwoDOF_PID_joint_space;
  chartInstance->chartInfo.disableChart =
    sf_opaque_disable_c9_TwoDOF_PID_joint_space;
  chartInstance->chartInfo.getSimState =
    sf_opaque_get_sim_state_c9_TwoDOF_PID_joint_space;
  chartInstance->chartInfo.setSimState =
    sf_opaque_set_sim_state_c9_TwoDOF_PID_joint_space;
  chartInstance->chartInfo.getSimStateInfo =
    sf_get_sim_state_info_c9_TwoDOF_PID_joint_space;
  chartInstance->chartInfo.zeroCrossings = NULL;
  chartInstance->chartInfo.outputs = NULL;
  chartInstance->chartInfo.derivatives = NULL;
  chartInstance->chartInfo.mdlRTW = mdlRTW_c9_TwoDOF_PID_joint_space;
  chartInstance->chartInfo.mdlStart = mdlStart_c9_TwoDOF_PID_joint_space;
  chartInstance->chartInfo.mdlSetWorkWidths =
    mdlSetWorkWidths_c9_TwoDOF_PID_joint_space;
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

void c9_TwoDOF_PID_joint_space_method_dispatcher(SimStruct *S, int_T method,
  void *data)
{
  switch (method) {
   case SS_CALL_MDL_START:
    mdlStart_c9_TwoDOF_PID_joint_space(S);
    break;

   case SS_CALL_MDL_SET_WORK_WIDTHS:
    mdlSetWorkWidths_c9_TwoDOF_PID_joint_space(S);
    break;

   case SS_CALL_MDL_PROCESS_PARAMETERS:
    mdlProcessParameters_c9_TwoDOF_PID_joint_space(S);
    break;

   default:
    /* Unhandled method */
    sf_mex_error_message("Stateflow Internal Error:\n"
                         "Error calling c9_TwoDOF_PID_joint_space_method_dispatcher.\n"
                         "Can't handle method %d.\n", method);
    break;
  }
}
