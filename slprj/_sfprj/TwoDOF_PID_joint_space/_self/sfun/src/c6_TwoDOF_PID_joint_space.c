/* Include files */

#include <stddef.h>
#include "blas.h"
#include "TwoDOF_PID_joint_space_sfun.h"
#include "c6_TwoDOF_PID_joint_space.h"
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
static const char * c6_debug_family_names[13] = { "C1", "S1", "C12", "S12", "L1",
  "L2", "J", "nargin", "nargout", "Q", "V", "invJ", "QP" };

/* Function Declarations */
static void initialize_c6_TwoDOF_PID_joint_space
  (SFc6_TwoDOF_PID_joint_spaceInstanceStruct *chartInstance);
static void initialize_params_c6_TwoDOF_PID_joint_space
  (SFc6_TwoDOF_PID_joint_spaceInstanceStruct *chartInstance);
static void enable_c6_TwoDOF_PID_joint_space
  (SFc6_TwoDOF_PID_joint_spaceInstanceStruct *chartInstance);
static void disable_c6_TwoDOF_PID_joint_space
  (SFc6_TwoDOF_PID_joint_spaceInstanceStruct *chartInstance);
static void c6_update_debugger_state_c6_TwoDOF_PID_joint_space
  (SFc6_TwoDOF_PID_joint_spaceInstanceStruct *chartInstance);
static const mxArray *get_sim_state_c6_TwoDOF_PID_joint_space
  (SFc6_TwoDOF_PID_joint_spaceInstanceStruct *chartInstance);
static void set_sim_state_c6_TwoDOF_PID_joint_space
  (SFc6_TwoDOF_PID_joint_spaceInstanceStruct *chartInstance, const mxArray
   *c6_st);
static void finalize_c6_TwoDOF_PID_joint_space
  (SFc6_TwoDOF_PID_joint_spaceInstanceStruct *chartInstance);
static void sf_gateway_c6_TwoDOF_PID_joint_space
  (SFc6_TwoDOF_PID_joint_spaceInstanceStruct *chartInstance);
static void initSimStructsc6_TwoDOF_PID_joint_space
  (SFc6_TwoDOF_PID_joint_spaceInstanceStruct *chartInstance);
static void init_script_number_translation(uint32_T c6_machineNumber, uint32_T
  c6_chartNumber, uint32_T c6_instanceNumber);
static const mxArray *c6_sf_marshallOut(void *chartInstanceVoid, void *c6_inData);
static void c6_emlrt_marshallIn(SFc6_TwoDOF_PID_joint_spaceInstanceStruct
  *chartInstance, const mxArray *c6_QP, const char_T *c6_identifier, real_T
  c6_y[2]);
static void c6_b_emlrt_marshallIn(SFc6_TwoDOF_PID_joint_spaceInstanceStruct
  *chartInstance, const mxArray *c6_u, const emlrtMsgIdentifier *c6_parentId,
  real_T c6_y[2]);
static void c6_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c6_mxArrayInData, const char_T *c6_varName, void *c6_outData);
static const mxArray *c6_b_sf_marshallOut(void *chartInstanceVoid, void
  *c6_inData);
static void c6_c_emlrt_marshallIn(SFc6_TwoDOF_PID_joint_spaceInstanceStruct
  *chartInstance, const mxArray *c6_invJ, const char_T *c6_identifier, real_T
  c6_y[4]);
static void c6_d_emlrt_marshallIn(SFc6_TwoDOF_PID_joint_spaceInstanceStruct
  *chartInstance, const mxArray *c6_u, const emlrtMsgIdentifier *c6_parentId,
  real_T c6_y[4]);
static void c6_b_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c6_mxArrayInData, const char_T *c6_varName, void *c6_outData);
static const mxArray *c6_c_sf_marshallOut(void *chartInstanceVoid, void
  *c6_inData);
static real_T c6_e_emlrt_marshallIn(SFc6_TwoDOF_PID_joint_spaceInstanceStruct
  *chartInstance, const mxArray *c6_u, const emlrtMsgIdentifier *c6_parentId);
static void c6_c_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c6_mxArrayInData, const char_T *c6_varName, void *c6_outData);
static void c6_info_helper(const mxArray **c6_info);
static const mxArray *c6_emlrt_marshallOut(const char * c6_u);
static const mxArray *c6_b_emlrt_marshallOut(const uint32_T c6_u);
static void c6_mpower(SFc6_TwoDOF_PID_joint_spaceInstanceStruct *chartInstance,
                      real_T c6_a[4], real_T c6_c[4]);
static void c6_inv2x2(SFc6_TwoDOF_PID_joint_spaceInstanceStruct *chartInstance,
                      real_T c6_x[4], real_T c6_y[4]);
static real_T c6_norm(SFc6_TwoDOF_PID_joint_spaceInstanceStruct *chartInstance,
                      real_T c6_x[4]);
static void c6_eml_warning(SFc6_TwoDOF_PID_joint_spaceInstanceStruct
  *chartInstance);
static void c6_b_eml_warning(SFc6_TwoDOF_PID_joint_spaceInstanceStruct
  *chartInstance, char_T c6_varargin_2[14]);
static void c6_eml_scalar_eg(SFc6_TwoDOF_PID_joint_spaceInstanceStruct
  *chartInstance);
static void c6_eml_xgemm(SFc6_TwoDOF_PID_joint_spaceInstanceStruct
  *chartInstance, real_T c6_A[4], real_T c6_B[2], real_T c6_C[2], real_T c6_b_C
  [2]);
static void c6_f_emlrt_marshallIn(SFc6_TwoDOF_PID_joint_spaceInstanceStruct
  *chartInstance, const mxArray *c6_sprintf, const char_T *c6_identifier, char_T
  c6_y[14]);
static void c6_g_emlrt_marshallIn(SFc6_TwoDOF_PID_joint_spaceInstanceStruct
  *chartInstance, const mxArray *c6_u, const emlrtMsgIdentifier *c6_parentId,
  char_T c6_y[14]);
static const mxArray *c6_d_sf_marshallOut(void *chartInstanceVoid, void
  *c6_inData);
static int32_T c6_h_emlrt_marshallIn(SFc6_TwoDOF_PID_joint_spaceInstanceStruct
  *chartInstance, const mxArray *c6_u, const emlrtMsgIdentifier *c6_parentId);
static void c6_d_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c6_mxArrayInData, const char_T *c6_varName, void *c6_outData);
static uint8_T c6_i_emlrt_marshallIn(SFc6_TwoDOF_PID_joint_spaceInstanceStruct
  *chartInstance, const mxArray *c6_b_is_active_c6_TwoDOF_PID_joint_space, const
  char_T *c6_identifier);
static uint8_T c6_j_emlrt_marshallIn(SFc6_TwoDOF_PID_joint_spaceInstanceStruct
  *chartInstance, const mxArray *c6_u, const emlrtMsgIdentifier *c6_parentId);
static void c6_b_eml_xgemm(SFc6_TwoDOF_PID_joint_spaceInstanceStruct
  *chartInstance, real_T c6_A[4], real_T c6_B[2], real_T c6_C[2]);
static void init_dsm_address_info(SFc6_TwoDOF_PID_joint_spaceInstanceStruct
  *chartInstance);

/* Function Definitions */
static void initialize_c6_TwoDOF_PID_joint_space
  (SFc6_TwoDOF_PID_joint_spaceInstanceStruct *chartInstance)
{
  chartInstance->c6_sfEvent = CALL_EVENT;
  _sfTime_ = sf_get_time(chartInstance->S);
  chartInstance->c6_is_active_c6_TwoDOF_PID_joint_space = 0U;
}

static void initialize_params_c6_TwoDOF_PID_joint_space
  (SFc6_TwoDOF_PID_joint_spaceInstanceStruct *chartInstance)
{
  (void)chartInstance;
}

static void enable_c6_TwoDOF_PID_joint_space
  (SFc6_TwoDOF_PID_joint_spaceInstanceStruct *chartInstance)
{
  _sfTime_ = sf_get_time(chartInstance->S);
}

static void disable_c6_TwoDOF_PID_joint_space
  (SFc6_TwoDOF_PID_joint_spaceInstanceStruct *chartInstance)
{
  _sfTime_ = sf_get_time(chartInstance->S);
}

static void c6_update_debugger_state_c6_TwoDOF_PID_joint_space
  (SFc6_TwoDOF_PID_joint_spaceInstanceStruct *chartInstance)
{
  (void)chartInstance;
}

static const mxArray *get_sim_state_c6_TwoDOF_PID_joint_space
  (SFc6_TwoDOF_PID_joint_spaceInstanceStruct *chartInstance)
{
  const mxArray *c6_st;
  const mxArray *c6_y = NULL;
  int32_T c6_i0;
  real_T c6_u[2];
  const mxArray *c6_b_y = NULL;
  int32_T c6_i1;
  real_T c6_b_u[4];
  const mxArray *c6_c_y = NULL;
  uint8_T c6_hoistedGlobal;
  uint8_T c6_c_u;
  const mxArray *c6_d_y = NULL;
  real_T (*c6_invJ)[4];
  real_T (*c6_QP)[2];
  c6_QP = (real_T (*)[2])ssGetOutputPortSignal(chartInstance->S, 2);
  c6_invJ = (real_T (*)[4])ssGetOutputPortSignal(chartInstance->S, 1);
  c6_st = NULL;
  c6_st = NULL;
  c6_y = NULL;
  sf_mex_assign(&c6_y, sf_mex_createcellmatrix(3, 1), false);
  for (c6_i0 = 0; c6_i0 < 2; c6_i0++) {
    c6_u[c6_i0] = (*c6_QP)[c6_i0];
  }

  c6_b_y = NULL;
  sf_mex_assign(&c6_b_y, sf_mex_create("y", c6_u, 0, 0U, 1U, 0U, 1, 2), false);
  sf_mex_setcell(c6_y, 0, c6_b_y);
  for (c6_i1 = 0; c6_i1 < 4; c6_i1++) {
    c6_b_u[c6_i1] = (*c6_invJ)[c6_i1];
  }

  c6_c_y = NULL;
  sf_mex_assign(&c6_c_y, sf_mex_create("y", c6_b_u, 0, 0U, 1U, 0U, 2, 2, 2),
                false);
  sf_mex_setcell(c6_y, 1, c6_c_y);
  c6_hoistedGlobal = chartInstance->c6_is_active_c6_TwoDOF_PID_joint_space;
  c6_c_u = c6_hoistedGlobal;
  c6_d_y = NULL;
  sf_mex_assign(&c6_d_y, sf_mex_create("y", &c6_c_u, 3, 0U, 0U, 0U, 0), false);
  sf_mex_setcell(c6_y, 2, c6_d_y);
  sf_mex_assign(&c6_st, c6_y, false);
  return c6_st;
}

static void set_sim_state_c6_TwoDOF_PID_joint_space
  (SFc6_TwoDOF_PID_joint_spaceInstanceStruct *chartInstance, const mxArray
   *c6_st)
{
  const mxArray *c6_u;
  real_T c6_dv0[2];
  int32_T c6_i2;
  real_T c6_dv1[4];
  int32_T c6_i3;
  real_T (*c6_QP)[2];
  real_T (*c6_invJ)[4];
  c6_QP = (real_T (*)[2])ssGetOutputPortSignal(chartInstance->S, 2);
  c6_invJ = (real_T (*)[4])ssGetOutputPortSignal(chartInstance->S, 1);
  chartInstance->c6_doneDoubleBufferReInit = true;
  c6_u = sf_mex_dup(c6_st);
  c6_emlrt_marshallIn(chartInstance, sf_mex_dup(sf_mex_getcell(c6_u, 0)), "QP",
                      c6_dv0);
  for (c6_i2 = 0; c6_i2 < 2; c6_i2++) {
    (*c6_QP)[c6_i2] = c6_dv0[c6_i2];
  }

  c6_c_emlrt_marshallIn(chartInstance, sf_mex_dup(sf_mex_getcell(c6_u, 1)),
                        "invJ", c6_dv1);
  for (c6_i3 = 0; c6_i3 < 4; c6_i3++) {
    (*c6_invJ)[c6_i3] = c6_dv1[c6_i3];
  }

  chartInstance->c6_is_active_c6_TwoDOF_PID_joint_space = c6_i_emlrt_marshallIn
    (chartInstance, sf_mex_dup(sf_mex_getcell(c6_u, 2)),
     "is_active_c6_TwoDOF_PID_joint_space");
  sf_mex_destroy(&c6_u);
  c6_update_debugger_state_c6_TwoDOF_PID_joint_space(chartInstance);
  sf_mex_destroy(&c6_st);
}

static void finalize_c6_TwoDOF_PID_joint_space
  (SFc6_TwoDOF_PID_joint_spaceInstanceStruct *chartInstance)
{
  (void)chartInstance;
}

static void sf_gateway_c6_TwoDOF_PID_joint_space
  (SFc6_TwoDOF_PID_joint_spaceInstanceStruct *chartInstance)
{
  int32_T c6_i4;
  int32_T c6_i5;
  real_T c6_Q[2];
  int32_T c6_i6;
  real_T c6_V[2];
  uint32_T c6_debug_family_var_map[13];
  real_T c6_C1;
  real_T c6_S1;
  real_T c6_C12;
  real_T c6_S12;
  real_T c6_L1;
  real_T c6_L2;
  real_T c6_J[4];
  real_T c6_nargin = 2.0;
  real_T c6_nargout = 2.0;
  real_T c6_invJ[4];
  real_T c6_QP[2];
  real_T c6_x;
  real_T c6_b_x;
  real_T c6_c_x;
  real_T c6_d_x;
  real_T c6_e_x;
  real_T c6_f_x;
  real_T c6_g_x;
  real_T c6_h_x;
  int32_T c6_i7;
  real_T c6_b_J[4];
  real_T c6_dv2[4];
  int32_T c6_i8;
  int32_T c6_i9;
  real_T c6_c_J[4];
  real_T c6_a[4];
  int32_T c6_i10;
  real_T c6_b[2];
  int32_T c6_i11;
  int32_T c6_i12;
  int32_T c6_i13;
  real_T c6_dv3[4];
  int32_T c6_i14;
  real_T c6_dv4[2];
  int32_T c6_i15;
  real_T c6_dv5[4];
  int32_T c6_i16;
  real_T c6_dv6[2];
  int32_T c6_i17;
  int32_T c6_i18;
  int32_T c6_i19;
  int32_T c6_i20;
  int32_T c6_i21;
  real_T (*c6_b_invJ)[4];
  real_T (*c6_b_QP)[2];
  real_T (*c6_b_V)[2];
  real_T (*c6_b_Q)[2];
  c6_b_QP = (real_T (*)[2])ssGetOutputPortSignal(chartInstance->S, 2);
  c6_b_V = (real_T (*)[2])ssGetInputPortSignal(chartInstance->S, 1);
  c6_b_invJ = (real_T (*)[4])ssGetOutputPortSignal(chartInstance->S, 1);
  c6_b_Q = (real_T (*)[2])ssGetInputPortSignal(chartInstance->S, 0);
  _SFD_SYMBOL_SCOPE_PUSH(0U, 0U);
  _sfTime_ = sf_get_time(chartInstance->S);
  _SFD_CC_CALL(CHART_ENTER_SFUNCTION_TAG, 1U, chartInstance->c6_sfEvent);
  for (c6_i4 = 0; c6_i4 < 2; c6_i4++) {
    _SFD_DATA_RANGE_CHECK((*c6_b_Q)[c6_i4], 0U);
  }

  chartInstance->c6_sfEvent = CALL_EVENT;
  _SFD_CC_CALL(CHART_ENTER_DURING_FUNCTION_TAG, 1U, chartInstance->c6_sfEvent);
  for (c6_i5 = 0; c6_i5 < 2; c6_i5++) {
    c6_Q[c6_i5] = (*c6_b_Q)[c6_i5];
  }

  for (c6_i6 = 0; c6_i6 < 2; c6_i6++) {
    c6_V[c6_i6] = (*c6_b_V)[c6_i6];
  }

  _SFD_SYMBOL_SCOPE_PUSH_EML(0U, 13U, 13U, c6_debug_family_names,
    c6_debug_family_var_map);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c6_C1, 0U, c6_c_sf_marshallOut,
    c6_c_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c6_S1, 1U, c6_c_sf_marshallOut,
    c6_c_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c6_C12, 2U, c6_c_sf_marshallOut,
    c6_c_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c6_S12, 3U, c6_c_sf_marshallOut,
    c6_c_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML(&c6_L1, 4U, c6_c_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML(&c6_L2, 5U, c6_c_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c6_J, 6U, c6_b_sf_marshallOut,
    c6_b_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c6_nargin, 7U, c6_c_sf_marshallOut,
    c6_c_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c6_nargout, 8U, c6_c_sf_marshallOut,
    c6_c_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML(c6_Q, 9U, c6_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML(c6_V, 10U, c6_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c6_invJ, 11U, c6_b_sf_marshallOut,
    c6_b_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c6_QP, 12U, c6_sf_marshallOut,
    c6_sf_marshallIn);
  CV_EML_FCN(0, 0);
  _SFD_EML_CALL(0U, chartInstance->c6_sfEvent, 3);
  c6_x = c6_Q[0];
  c6_C1 = c6_x;
  c6_b_x = c6_C1;
  c6_C1 = c6_b_x;
  c6_C1 = muDoubleScalarCos(c6_C1);
  _SFD_EML_CALL(0U, chartInstance->c6_sfEvent, 4);
  c6_c_x = c6_Q[0];
  c6_S1 = c6_c_x;
  c6_d_x = c6_S1;
  c6_S1 = c6_d_x;
  c6_S1 = muDoubleScalarSin(c6_S1);
  _SFD_EML_CALL(0U, chartInstance->c6_sfEvent, 5);
  c6_e_x = c6_Q[0] + c6_Q[1];
  c6_C12 = c6_e_x;
  c6_f_x = c6_C12;
  c6_C12 = c6_f_x;
  c6_C12 = muDoubleScalarCos(c6_C12);
  _SFD_EML_CALL(0U, chartInstance->c6_sfEvent, 6);
  c6_g_x = c6_Q[0] + c6_Q[1];
  c6_S12 = c6_g_x;
  c6_h_x = c6_S12;
  c6_S12 = c6_h_x;
  c6_S12 = muDoubleScalarSin(c6_S12);
  _SFD_EML_CALL(0U, chartInstance->c6_sfEvent, 8);
  c6_L1 = 0.2;
  _SFD_EML_CALL(0U, chartInstance->c6_sfEvent, 9);
  c6_L2 = 0.1442;
  _SFD_EML_CALL(0U, chartInstance->c6_sfEvent, 11);
  c6_J[0] = -0.2 * c6_S1 - 0.1442 * c6_S12;
  c6_J[2] = -0.1442 * c6_S12;
  c6_J[1] = 0.2 * c6_C1 + 0.1442 * c6_C12;
  c6_J[3] = 0.1442 * c6_C12;
  _SFD_EML_CALL(0U, chartInstance->c6_sfEvent, 13);
  for (c6_i7 = 0; c6_i7 < 4; c6_i7++) {
    c6_b_J[c6_i7] = c6_J[c6_i7];
  }

  c6_mpower(chartInstance, c6_b_J, c6_dv2);
  for (c6_i8 = 0; c6_i8 < 4; c6_i8++) {
    c6_invJ[c6_i8] = c6_dv2[c6_i8];
  }

  _SFD_EML_CALL(0U, chartInstance->c6_sfEvent, 14);
  for (c6_i9 = 0; c6_i9 < 4; c6_i9++) {
    c6_c_J[c6_i9] = c6_J[c6_i9];
  }

  c6_mpower(chartInstance, c6_c_J, c6_a);
  for (c6_i10 = 0; c6_i10 < 2; c6_i10++) {
    c6_b[c6_i10] = c6_V[c6_i10];
  }

  c6_eml_scalar_eg(chartInstance);
  c6_eml_scalar_eg(chartInstance);
  for (c6_i11 = 0; c6_i11 < 2; c6_i11++) {
    c6_QP[c6_i11] = 0.0;
  }

  for (c6_i12 = 0; c6_i12 < 2; c6_i12++) {
    c6_QP[c6_i12] = 0.0;
  }

  for (c6_i13 = 0; c6_i13 < 4; c6_i13++) {
    c6_dv3[c6_i13] = c6_a[c6_i13];
  }

  for (c6_i14 = 0; c6_i14 < 2; c6_i14++) {
    c6_dv4[c6_i14] = c6_b[c6_i14];
  }

  for (c6_i15 = 0; c6_i15 < 4; c6_i15++) {
    c6_dv5[c6_i15] = c6_dv3[c6_i15];
  }

  for (c6_i16 = 0; c6_i16 < 2; c6_i16++) {
    c6_dv6[c6_i16] = c6_dv4[c6_i16];
  }

  c6_b_eml_xgemm(chartInstance, c6_dv5, c6_dv6, c6_QP);
  _SFD_EML_CALL(0U, chartInstance->c6_sfEvent, -14);
  _SFD_SYMBOL_SCOPE_POP();
  for (c6_i17 = 0; c6_i17 < 4; c6_i17++) {
    (*c6_b_invJ)[c6_i17] = c6_invJ[c6_i17];
  }

  for (c6_i18 = 0; c6_i18 < 2; c6_i18++) {
    (*c6_b_QP)[c6_i18] = c6_QP[c6_i18];
  }

  _SFD_CC_CALL(EXIT_OUT_OF_FUNCTION_TAG, 1U, chartInstance->c6_sfEvent);
  _SFD_SYMBOL_SCOPE_POP();
  _SFD_CHECK_FOR_STATE_INCONSISTENCY(_TwoDOF_PID_joint_spaceMachineNumber_,
    chartInstance->chartNumber, chartInstance->instanceNumber);
  for (c6_i19 = 0; c6_i19 < 4; c6_i19++) {
    _SFD_DATA_RANGE_CHECK((*c6_b_invJ)[c6_i19], 1U);
  }

  for (c6_i20 = 0; c6_i20 < 2; c6_i20++) {
    _SFD_DATA_RANGE_CHECK((*c6_b_V)[c6_i20], 2U);
  }

  for (c6_i21 = 0; c6_i21 < 2; c6_i21++) {
    _SFD_DATA_RANGE_CHECK((*c6_b_QP)[c6_i21], 3U);
  }
}

static void initSimStructsc6_TwoDOF_PID_joint_space
  (SFc6_TwoDOF_PID_joint_spaceInstanceStruct *chartInstance)
{
  (void)chartInstance;
}

static void init_script_number_translation(uint32_T c6_machineNumber, uint32_T
  c6_chartNumber, uint32_T c6_instanceNumber)
{
  (void)c6_machineNumber;
  (void)c6_chartNumber;
  (void)c6_instanceNumber;
}

static const mxArray *c6_sf_marshallOut(void *chartInstanceVoid, void *c6_inData)
{
  const mxArray *c6_mxArrayOutData = NULL;
  int32_T c6_i22;
  real_T c6_b_inData[2];
  int32_T c6_i23;
  real_T c6_u[2];
  const mxArray *c6_y = NULL;
  SFc6_TwoDOF_PID_joint_spaceInstanceStruct *chartInstance;
  chartInstance = (SFc6_TwoDOF_PID_joint_spaceInstanceStruct *)chartInstanceVoid;
  c6_mxArrayOutData = NULL;
  for (c6_i22 = 0; c6_i22 < 2; c6_i22++) {
    c6_b_inData[c6_i22] = (*(real_T (*)[2])c6_inData)[c6_i22];
  }

  for (c6_i23 = 0; c6_i23 < 2; c6_i23++) {
    c6_u[c6_i23] = c6_b_inData[c6_i23];
  }

  c6_y = NULL;
  sf_mex_assign(&c6_y, sf_mex_create("y", c6_u, 0, 0U, 1U, 0U, 1, 2), false);
  sf_mex_assign(&c6_mxArrayOutData, c6_y, false);
  return c6_mxArrayOutData;
}

static void c6_emlrt_marshallIn(SFc6_TwoDOF_PID_joint_spaceInstanceStruct
  *chartInstance, const mxArray *c6_QP, const char_T *c6_identifier, real_T
  c6_y[2])
{
  emlrtMsgIdentifier c6_thisId;
  c6_thisId.fIdentifier = c6_identifier;
  c6_thisId.fParent = NULL;
  c6_b_emlrt_marshallIn(chartInstance, sf_mex_dup(c6_QP), &c6_thisId, c6_y);
  sf_mex_destroy(&c6_QP);
}

static void c6_b_emlrt_marshallIn(SFc6_TwoDOF_PID_joint_spaceInstanceStruct
  *chartInstance, const mxArray *c6_u, const emlrtMsgIdentifier *c6_parentId,
  real_T c6_y[2])
{
  real_T c6_dv7[2];
  int32_T c6_i24;
  (void)chartInstance;
  sf_mex_import(c6_parentId, sf_mex_dup(c6_u), c6_dv7, 1, 0, 0U, 1, 0U, 1, 2);
  for (c6_i24 = 0; c6_i24 < 2; c6_i24++) {
    c6_y[c6_i24] = c6_dv7[c6_i24];
  }

  sf_mex_destroy(&c6_u);
}

static void c6_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c6_mxArrayInData, const char_T *c6_varName, void *c6_outData)
{
  const mxArray *c6_QP;
  const char_T *c6_identifier;
  emlrtMsgIdentifier c6_thisId;
  real_T c6_y[2];
  int32_T c6_i25;
  SFc6_TwoDOF_PID_joint_spaceInstanceStruct *chartInstance;
  chartInstance = (SFc6_TwoDOF_PID_joint_spaceInstanceStruct *)chartInstanceVoid;
  c6_QP = sf_mex_dup(c6_mxArrayInData);
  c6_identifier = c6_varName;
  c6_thisId.fIdentifier = c6_identifier;
  c6_thisId.fParent = NULL;
  c6_b_emlrt_marshallIn(chartInstance, sf_mex_dup(c6_QP), &c6_thisId, c6_y);
  sf_mex_destroy(&c6_QP);
  for (c6_i25 = 0; c6_i25 < 2; c6_i25++) {
    (*(real_T (*)[2])c6_outData)[c6_i25] = c6_y[c6_i25];
  }

  sf_mex_destroy(&c6_mxArrayInData);
}

static const mxArray *c6_b_sf_marshallOut(void *chartInstanceVoid, void
  *c6_inData)
{
  const mxArray *c6_mxArrayOutData = NULL;
  int32_T c6_i26;
  int32_T c6_i27;
  int32_T c6_i28;
  real_T c6_b_inData[4];
  int32_T c6_i29;
  int32_T c6_i30;
  int32_T c6_i31;
  real_T c6_u[4];
  const mxArray *c6_y = NULL;
  SFc6_TwoDOF_PID_joint_spaceInstanceStruct *chartInstance;
  chartInstance = (SFc6_TwoDOF_PID_joint_spaceInstanceStruct *)chartInstanceVoid;
  c6_mxArrayOutData = NULL;
  c6_i26 = 0;
  for (c6_i27 = 0; c6_i27 < 2; c6_i27++) {
    for (c6_i28 = 0; c6_i28 < 2; c6_i28++) {
      c6_b_inData[c6_i28 + c6_i26] = (*(real_T (*)[4])c6_inData)[c6_i28 + c6_i26];
    }

    c6_i26 += 2;
  }

  c6_i29 = 0;
  for (c6_i30 = 0; c6_i30 < 2; c6_i30++) {
    for (c6_i31 = 0; c6_i31 < 2; c6_i31++) {
      c6_u[c6_i31 + c6_i29] = c6_b_inData[c6_i31 + c6_i29];
    }

    c6_i29 += 2;
  }

  c6_y = NULL;
  sf_mex_assign(&c6_y, sf_mex_create("y", c6_u, 0, 0U, 1U, 0U, 2, 2, 2), false);
  sf_mex_assign(&c6_mxArrayOutData, c6_y, false);
  return c6_mxArrayOutData;
}

static void c6_c_emlrt_marshallIn(SFc6_TwoDOF_PID_joint_spaceInstanceStruct
  *chartInstance, const mxArray *c6_invJ, const char_T *c6_identifier, real_T
  c6_y[4])
{
  emlrtMsgIdentifier c6_thisId;
  c6_thisId.fIdentifier = c6_identifier;
  c6_thisId.fParent = NULL;
  c6_d_emlrt_marshallIn(chartInstance, sf_mex_dup(c6_invJ), &c6_thisId, c6_y);
  sf_mex_destroy(&c6_invJ);
}

static void c6_d_emlrt_marshallIn(SFc6_TwoDOF_PID_joint_spaceInstanceStruct
  *chartInstance, const mxArray *c6_u, const emlrtMsgIdentifier *c6_parentId,
  real_T c6_y[4])
{
  real_T c6_dv8[4];
  int32_T c6_i32;
  (void)chartInstance;
  sf_mex_import(c6_parentId, sf_mex_dup(c6_u), c6_dv8, 1, 0, 0U, 1, 0U, 2, 2, 2);
  for (c6_i32 = 0; c6_i32 < 4; c6_i32++) {
    c6_y[c6_i32] = c6_dv8[c6_i32];
  }

  sf_mex_destroy(&c6_u);
}

static void c6_b_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c6_mxArrayInData, const char_T *c6_varName, void *c6_outData)
{
  const mxArray *c6_invJ;
  const char_T *c6_identifier;
  emlrtMsgIdentifier c6_thisId;
  real_T c6_y[4];
  int32_T c6_i33;
  int32_T c6_i34;
  int32_T c6_i35;
  SFc6_TwoDOF_PID_joint_spaceInstanceStruct *chartInstance;
  chartInstance = (SFc6_TwoDOF_PID_joint_spaceInstanceStruct *)chartInstanceVoid;
  c6_invJ = sf_mex_dup(c6_mxArrayInData);
  c6_identifier = c6_varName;
  c6_thisId.fIdentifier = c6_identifier;
  c6_thisId.fParent = NULL;
  c6_d_emlrt_marshallIn(chartInstance, sf_mex_dup(c6_invJ), &c6_thisId, c6_y);
  sf_mex_destroy(&c6_invJ);
  c6_i33 = 0;
  for (c6_i34 = 0; c6_i34 < 2; c6_i34++) {
    for (c6_i35 = 0; c6_i35 < 2; c6_i35++) {
      (*(real_T (*)[4])c6_outData)[c6_i35 + c6_i33] = c6_y[c6_i35 + c6_i33];
    }

    c6_i33 += 2;
  }

  sf_mex_destroy(&c6_mxArrayInData);
}

static const mxArray *c6_c_sf_marshallOut(void *chartInstanceVoid, void
  *c6_inData)
{
  const mxArray *c6_mxArrayOutData = NULL;
  real_T c6_u;
  const mxArray *c6_y = NULL;
  SFc6_TwoDOF_PID_joint_spaceInstanceStruct *chartInstance;
  chartInstance = (SFc6_TwoDOF_PID_joint_spaceInstanceStruct *)chartInstanceVoid;
  c6_mxArrayOutData = NULL;
  c6_u = *(real_T *)c6_inData;
  c6_y = NULL;
  sf_mex_assign(&c6_y, sf_mex_create("y", &c6_u, 0, 0U, 0U, 0U, 0), false);
  sf_mex_assign(&c6_mxArrayOutData, c6_y, false);
  return c6_mxArrayOutData;
}

static real_T c6_e_emlrt_marshallIn(SFc6_TwoDOF_PID_joint_spaceInstanceStruct
  *chartInstance, const mxArray *c6_u, const emlrtMsgIdentifier *c6_parentId)
{
  real_T c6_y;
  real_T c6_d0;
  (void)chartInstance;
  sf_mex_import(c6_parentId, sf_mex_dup(c6_u), &c6_d0, 1, 0, 0U, 0, 0U, 0);
  c6_y = c6_d0;
  sf_mex_destroy(&c6_u);
  return c6_y;
}

static void c6_c_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c6_mxArrayInData, const char_T *c6_varName, void *c6_outData)
{
  const mxArray *c6_nargout;
  const char_T *c6_identifier;
  emlrtMsgIdentifier c6_thisId;
  real_T c6_y;
  SFc6_TwoDOF_PID_joint_spaceInstanceStruct *chartInstance;
  chartInstance = (SFc6_TwoDOF_PID_joint_spaceInstanceStruct *)chartInstanceVoid;
  c6_nargout = sf_mex_dup(c6_mxArrayInData);
  c6_identifier = c6_varName;
  c6_thisId.fIdentifier = c6_identifier;
  c6_thisId.fParent = NULL;
  c6_y = c6_e_emlrt_marshallIn(chartInstance, sf_mex_dup(c6_nargout), &c6_thisId);
  sf_mex_destroy(&c6_nargout);
  *(real_T *)c6_outData = c6_y;
  sf_mex_destroy(&c6_mxArrayInData);
}

const mxArray *sf_c6_TwoDOF_PID_joint_space_get_eml_resolved_functions_info(void)
{
  const mxArray *c6_nameCaptureInfo = NULL;
  c6_nameCaptureInfo = NULL;
  sf_mex_assign(&c6_nameCaptureInfo, sf_mex_createstruct("structure", 2, 50, 1),
                false);
  c6_info_helper(&c6_nameCaptureInfo);
  sf_mex_emlrtNameCapturePostProcessR2012a(&c6_nameCaptureInfo);
  return c6_nameCaptureInfo;
}

static void c6_info_helper(const mxArray **c6_info)
{
  const mxArray *c6_rhs0 = NULL;
  const mxArray *c6_lhs0 = NULL;
  const mxArray *c6_rhs1 = NULL;
  const mxArray *c6_lhs1 = NULL;
  const mxArray *c6_rhs2 = NULL;
  const mxArray *c6_lhs2 = NULL;
  const mxArray *c6_rhs3 = NULL;
  const mxArray *c6_lhs3 = NULL;
  const mxArray *c6_rhs4 = NULL;
  const mxArray *c6_lhs4 = NULL;
  const mxArray *c6_rhs5 = NULL;
  const mxArray *c6_lhs5 = NULL;
  const mxArray *c6_rhs6 = NULL;
  const mxArray *c6_lhs6 = NULL;
  const mxArray *c6_rhs7 = NULL;
  const mxArray *c6_lhs7 = NULL;
  const mxArray *c6_rhs8 = NULL;
  const mxArray *c6_lhs8 = NULL;
  const mxArray *c6_rhs9 = NULL;
  const mxArray *c6_lhs9 = NULL;
  const mxArray *c6_rhs10 = NULL;
  const mxArray *c6_lhs10 = NULL;
  const mxArray *c6_rhs11 = NULL;
  const mxArray *c6_lhs11 = NULL;
  const mxArray *c6_rhs12 = NULL;
  const mxArray *c6_lhs12 = NULL;
  const mxArray *c6_rhs13 = NULL;
  const mxArray *c6_lhs13 = NULL;
  const mxArray *c6_rhs14 = NULL;
  const mxArray *c6_lhs14 = NULL;
  const mxArray *c6_rhs15 = NULL;
  const mxArray *c6_lhs15 = NULL;
  const mxArray *c6_rhs16 = NULL;
  const mxArray *c6_lhs16 = NULL;
  const mxArray *c6_rhs17 = NULL;
  const mxArray *c6_lhs17 = NULL;
  const mxArray *c6_rhs18 = NULL;
  const mxArray *c6_lhs18 = NULL;
  const mxArray *c6_rhs19 = NULL;
  const mxArray *c6_lhs19 = NULL;
  const mxArray *c6_rhs20 = NULL;
  const mxArray *c6_lhs20 = NULL;
  const mxArray *c6_rhs21 = NULL;
  const mxArray *c6_lhs21 = NULL;
  const mxArray *c6_rhs22 = NULL;
  const mxArray *c6_lhs22 = NULL;
  const mxArray *c6_rhs23 = NULL;
  const mxArray *c6_lhs23 = NULL;
  const mxArray *c6_rhs24 = NULL;
  const mxArray *c6_lhs24 = NULL;
  const mxArray *c6_rhs25 = NULL;
  const mxArray *c6_lhs25 = NULL;
  const mxArray *c6_rhs26 = NULL;
  const mxArray *c6_lhs26 = NULL;
  const mxArray *c6_rhs27 = NULL;
  const mxArray *c6_lhs27 = NULL;
  const mxArray *c6_rhs28 = NULL;
  const mxArray *c6_lhs28 = NULL;
  const mxArray *c6_rhs29 = NULL;
  const mxArray *c6_lhs29 = NULL;
  const mxArray *c6_rhs30 = NULL;
  const mxArray *c6_lhs30 = NULL;
  const mxArray *c6_rhs31 = NULL;
  const mxArray *c6_lhs31 = NULL;
  const mxArray *c6_rhs32 = NULL;
  const mxArray *c6_lhs32 = NULL;
  const mxArray *c6_rhs33 = NULL;
  const mxArray *c6_lhs33 = NULL;
  const mxArray *c6_rhs34 = NULL;
  const mxArray *c6_lhs34 = NULL;
  const mxArray *c6_rhs35 = NULL;
  const mxArray *c6_lhs35 = NULL;
  const mxArray *c6_rhs36 = NULL;
  const mxArray *c6_lhs36 = NULL;
  const mxArray *c6_rhs37 = NULL;
  const mxArray *c6_lhs37 = NULL;
  const mxArray *c6_rhs38 = NULL;
  const mxArray *c6_lhs38 = NULL;
  const mxArray *c6_rhs39 = NULL;
  const mxArray *c6_lhs39 = NULL;
  const mxArray *c6_rhs40 = NULL;
  const mxArray *c6_lhs40 = NULL;
  const mxArray *c6_rhs41 = NULL;
  const mxArray *c6_lhs41 = NULL;
  const mxArray *c6_rhs42 = NULL;
  const mxArray *c6_lhs42 = NULL;
  const mxArray *c6_rhs43 = NULL;
  const mxArray *c6_lhs43 = NULL;
  const mxArray *c6_rhs44 = NULL;
  const mxArray *c6_lhs44 = NULL;
  const mxArray *c6_rhs45 = NULL;
  const mxArray *c6_lhs45 = NULL;
  const mxArray *c6_rhs46 = NULL;
  const mxArray *c6_lhs46 = NULL;
  const mxArray *c6_rhs47 = NULL;
  const mxArray *c6_lhs47 = NULL;
  const mxArray *c6_rhs48 = NULL;
  const mxArray *c6_lhs48 = NULL;
  const mxArray *c6_rhs49 = NULL;
  const mxArray *c6_lhs49 = NULL;
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(""), "context", "context", 0);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut("cos"), "name", "name", 0);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 0);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/cos.m"), "resolved",
                  "resolved", 0);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(1343837572U), "fileTimeLo",
                  "fileTimeLo", 0);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 0);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 0);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 0);
  sf_mex_assign(&c6_rhs0, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c6_lhs0, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c6_info, sf_mex_duplicatearraysafe(&c6_rhs0), "rhs", "rhs", 0);
  sf_mex_addfield(*c6_info, sf_mex_duplicatearraysafe(&c6_lhs0), "lhs", "lhs", 0);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/cos.m"), "context",
                  "context", 1);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut("eml_scalar_cos"), "name",
                  "name", 1);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 1);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/eml_scalar_cos.m"),
                  "resolved", "resolved", 1);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(1286825922U), "fileTimeLo",
                  "fileTimeLo", 1);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 1);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 1);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 1);
  sf_mex_assign(&c6_rhs1, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c6_lhs1, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c6_info, sf_mex_duplicatearraysafe(&c6_rhs1), "rhs", "rhs", 1);
  sf_mex_addfield(*c6_info, sf_mex_duplicatearraysafe(&c6_lhs1), "lhs", "lhs", 1);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(""), "context", "context", 2);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut("sin"), "name", "name", 2);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 2);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/sin.m"), "resolved",
                  "resolved", 2);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(1343837586U), "fileTimeLo",
                  "fileTimeLo", 2);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 2);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 2);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 2);
  sf_mex_assign(&c6_rhs2, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c6_lhs2, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c6_info, sf_mex_duplicatearraysafe(&c6_rhs2), "rhs", "rhs", 2);
  sf_mex_addfield(*c6_info, sf_mex_duplicatearraysafe(&c6_lhs2), "lhs", "lhs", 2);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/sin.m"), "context",
                  "context", 3);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut("eml_scalar_sin"), "name",
                  "name", 3);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 3);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/eml_scalar_sin.m"),
                  "resolved", "resolved", 3);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(1286825936U), "fileTimeLo",
                  "fileTimeLo", 3);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 3);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 3);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 3);
  sf_mex_assign(&c6_rhs3, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c6_lhs3, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c6_info, sf_mex_duplicatearraysafe(&c6_rhs3), "rhs", "rhs", 3);
  sf_mex_addfield(*c6_info, sf_mex_duplicatearraysafe(&c6_lhs3), "lhs", "lhs", 3);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(""), "context", "context", 4);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut("mpower"), "name", "name", 4);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 4);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/mpower.m"), "resolved",
                  "resolved", 4);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(1363717478U), "fileTimeLo",
                  "fileTimeLo", 4);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 4);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 4);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 4);
  sf_mex_assign(&c6_rhs4, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c6_lhs4, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c6_info, sf_mex_duplicatearraysafe(&c6_rhs4), "rhs", "rhs", 4);
  sf_mex_addfield(*c6_info, sf_mex_duplicatearraysafe(&c6_lhs4), "lhs", "lhs", 4);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/mpower.m"), "context",
                  "context", 5);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(
    "coder.internal.isBuiltInNumeric"), "name", "name", 5);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 5);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/shared/coder/coder/+coder/+internal/isBuiltInNumeric.m"),
                  "resolved", "resolved", 5);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(1363718156U), "fileTimeLo",
                  "fileTimeLo", 5);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 5);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 5);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 5);
  sf_mex_assign(&c6_rhs5, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c6_lhs5, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c6_info, sf_mex_duplicatearraysafe(&c6_rhs5), "rhs", "rhs", 5);
  sf_mex_addfield(*c6_info, sf_mex_duplicatearraysafe(&c6_lhs5), "lhs", "lhs", 5);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/mpower.m"), "context",
                  "context", 6);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut("ismatrix"), "name", "name", 6);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 6);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/ismatrix.m"), "resolved",
                  "resolved", 6);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(1331308458U), "fileTimeLo",
                  "fileTimeLo", 6);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 6);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 6);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 6);
  sf_mex_assign(&c6_rhs6, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c6_lhs6, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c6_info, sf_mex_duplicatearraysafe(&c6_rhs6), "rhs", "rhs", 6);
  sf_mex_addfield(*c6_info, sf_mex_duplicatearraysafe(&c6_lhs6), "lhs", "lhs", 6);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/mpower.m"), "context",
                  "context", 7);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut("eml_scalar_floor"), "name",
                  "name", 7);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 7);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/eml_scalar_floor.m"),
                  "resolved", "resolved", 7);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(1286825926U), "fileTimeLo",
                  "fileTimeLo", 7);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 7);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 7);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 7);
  sf_mex_assign(&c6_rhs7, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c6_lhs7, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c6_info, sf_mex_duplicatearraysafe(&c6_rhs7), "rhs", "rhs", 7);
  sf_mex_addfield(*c6_info, sf_mex_duplicatearraysafe(&c6_lhs7), "lhs", "lhs", 7);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/mpower.m!matrix_to_integer_power"),
                  "context", "context", 8);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut("eml_scalar_eg"), "name",
                  "name", 8);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 8);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalar_eg.m"), "resolved",
                  "resolved", 8);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(1375987888U), "fileTimeLo",
                  "fileTimeLo", 8);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 8);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 8);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 8);
  sf_mex_assign(&c6_rhs8, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c6_lhs8, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c6_info, sf_mex_duplicatearraysafe(&c6_rhs8), "rhs", "rhs", 8);
  sf_mex_addfield(*c6_info, sf_mex_duplicatearraysafe(&c6_lhs8), "lhs", "lhs", 8);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalar_eg.m"), "context",
                  "context", 9);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut("coder.internal.scalarEg"),
                  "name", "name", 9);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 9);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/scalarEg.p"),
                  "resolved", "resolved", 9);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(1389311520U), "fileTimeLo",
                  "fileTimeLo", 9);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 9);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 9);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 9);
  sf_mex_assign(&c6_rhs9, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c6_lhs9, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c6_info, sf_mex_duplicatearraysafe(&c6_rhs9), "rhs", "rhs", 9);
  sf_mex_addfield(*c6_info, sf_mex_duplicatearraysafe(&c6_lhs9), "lhs", "lhs", 9);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/mpower.m!matrix_to_integer_power"),
                  "context", "context", 10);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut("inv"), "name", "name", 10);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 10);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/matfun/inv.m"), "resolved",
                  "resolved", 10);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(1305325200U), "fileTimeLo",
                  "fileTimeLo", 10);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 10);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 10);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 10);
  sf_mex_assign(&c6_rhs10, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c6_lhs10, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c6_info, sf_mex_duplicatearraysafe(&c6_rhs10), "rhs", "rhs",
                  10);
  sf_mex_addfield(*c6_info, sf_mex_duplicatearraysafe(&c6_lhs10), "lhs", "lhs",
                  10);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/matfun/inv.m!inv2x2"), "context",
                  "context", 11);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut("eml_xcabs1"), "name", "name",
                  11);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 11);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/eml_xcabs1.m"),
                  "resolved", "resolved", 11);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(1375987888U), "fileTimeLo",
                  "fileTimeLo", 11);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 11);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 11);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 11);
  sf_mex_assign(&c6_rhs11, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c6_lhs11, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c6_info, sf_mex_duplicatearraysafe(&c6_rhs11), "rhs", "rhs",
                  11);
  sf_mex_addfield(*c6_info, sf_mex_duplicatearraysafe(&c6_lhs11), "lhs", "lhs",
                  11);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/eml_xcabs1.m"),
                  "context", "context", 12);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut("coder.internal.refblas.xcabs1"),
                  "name", "name", 12);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 12);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+refblas/xcabs1.p"),
                  "resolved", "resolved", 12);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(1389311522U), "fileTimeLo",
                  "fileTimeLo", 12);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 12);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 12);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 12);
  sf_mex_assign(&c6_rhs12, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c6_lhs12, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c6_info, sf_mex_duplicatearraysafe(&c6_rhs12), "rhs", "rhs",
                  12);
  sf_mex_addfield(*c6_info, sf_mex_duplicatearraysafe(&c6_lhs12), "lhs", "lhs",
                  12);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+refblas/xcabs1.p"),
                  "context", "context", 13);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut("abs"), "name", "name", 13);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 13);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/abs.m"), "resolved",
                  "resolved", 13);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(1363717452U), "fileTimeLo",
                  "fileTimeLo", 13);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 13);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 13);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 13);
  sf_mex_assign(&c6_rhs13, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c6_lhs13, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c6_info, sf_mex_duplicatearraysafe(&c6_rhs13), "rhs", "rhs",
                  13);
  sf_mex_addfield(*c6_info, sf_mex_duplicatearraysafe(&c6_lhs13), "lhs", "lhs",
                  13);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/abs.m"), "context",
                  "context", 14);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(
    "coder.internal.isBuiltInNumeric"), "name", "name", 14);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 14);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/shared/coder/coder/+coder/+internal/isBuiltInNumeric.m"),
                  "resolved", "resolved", 14);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(1363718156U), "fileTimeLo",
                  "fileTimeLo", 14);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 14);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 14);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 14);
  sf_mex_assign(&c6_rhs14, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c6_lhs14, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c6_info, sf_mex_duplicatearraysafe(&c6_rhs14), "rhs", "rhs",
                  14);
  sf_mex_addfield(*c6_info, sf_mex_duplicatearraysafe(&c6_lhs14), "lhs", "lhs",
                  14);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/abs.m"), "context",
                  "context", 15);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut("eml_scalar_abs"), "name",
                  "name", 15);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 15);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/eml_scalar_abs.m"),
                  "resolved", "resolved", 15);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(1286825912U), "fileTimeLo",
                  "fileTimeLo", 15);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 15);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 15);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 15);
  sf_mex_assign(&c6_rhs15, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c6_lhs15, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c6_info, sf_mex_duplicatearraysafe(&c6_rhs15), "rhs", "rhs",
                  15);
  sf_mex_addfield(*c6_info, sf_mex_duplicatearraysafe(&c6_lhs15), "lhs", "lhs",
                  15);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/matfun/inv.m!inv2x2"), "context",
                  "context", 16);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut("mrdivide"), "name", "name", 16);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 16);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/mrdivide.p"), "resolved",
                  "resolved", 16);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(1388463696U), "fileTimeLo",
                  "fileTimeLo", 16);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 16);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(1370017086U), "mFileTimeLo",
                  "mFileTimeLo", 16);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 16);
  sf_mex_assign(&c6_rhs16, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c6_lhs16, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c6_info, sf_mex_duplicatearraysafe(&c6_rhs16), "rhs", "rhs",
                  16);
  sf_mex_addfield(*c6_info, sf_mex_duplicatearraysafe(&c6_lhs16), "lhs", "lhs",
                  16);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/mrdivide.p"), "context",
                  "context", 17);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut("coder.internal.assert"),
                  "name", "name", 17);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut("char"), "dominantType",
                  "dominantType", 17);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/shared/coder/coder/+coder/+internal/assert.m"),
                  "resolved", "resolved", 17);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(1363718156U), "fileTimeLo",
                  "fileTimeLo", 17);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 17);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 17);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 17);
  sf_mex_assign(&c6_rhs17, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c6_lhs17, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c6_info, sf_mex_duplicatearraysafe(&c6_rhs17), "rhs", "rhs",
                  17);
  sf_mex_addfield(*c6_info, sf_mex_duplicatearraysafe(&c6_lhs17), "lhs", "lhs",
                  17);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/mrdivide.p"), "context",
                  "context", 18);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut("rdivide"), "name", "name", 18);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 18);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/rdivide.m"), "resolved",
                  "resolved", 18);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(1363717480U), "fileTimeLo",
                  "fileTimeLo", 18);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 18);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 18);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 18);
  sf_mex_assign(&c6_rhs18, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c6_lhs18, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c6_info, sf_mex_duplicatearraysafe(&c6_rhs18), "rhs", "rhs",
                  18);
  sf_mex_addfield(*c6_info, sf_mex_duplicatearraysafe(&c6_lhs18), "lhs", "lhs",
                  18);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/rdivide.m"), "context",
                  "context", 19);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(
    "coder.internal.isBuiltInNumeric"), "name", "name", 19);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 19);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/shared/coder/coder/+coder/+internal/isBuiltInNumeric.m"),
                  "resolved", "resolved", 19);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(1363718156U), "fileTimeLo",
                  "fileTimeLo", 19);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 19);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 19);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 19);
  sf_mex_assign(&c6_rhs19, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c6_lhs19, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c6_info, sf_mex_duplicatearraysafe(&c6_rhs19), "rhs", "rhs",
                  19);
  sf_mex_addfield(*c6_info, sf_mex_duplicatearraysafe(&c6_lhs19), "lhs", "lhs",
                  19);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/rdivide.m"), "context",
                  "context", 20);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut("eml_scalexp_compatible"),
                  "name", "name", 20);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 20);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalexp_compatible.m"),
                  "resolved", "resolved", 20);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(1286825996U), "fileTimeLo",
                  "fileTimeLo", 20);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 20);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 20);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 20);
  sf_mex_assign(&c6_rhs20, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c6_lhs20, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c6_info, sf_mex_duplicatearraysafe(&c6_rhs20), "rhs", "rhs",
                  20);
  sf_mex_addfield(*c6_info, sf_mex_duplicatearraysafe(&c6_lhs20), "lhs", "lhs",
                  20);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/rdivide.m"), "context",
                  "context", 21);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut("eml_div"), "name", "name", 21);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 21);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_div.m"), "resolved",
                  "resolved", 21);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(1375987888U), "fileTimeLo",
                  "fileTimeLo", 21);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 21);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 21);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 21);
  sf_mex_assign(&c6_rhs21, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c6_lhs21, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c6_info, sf_mex_duplicatearraysafe(&c6_rhs21), "rhs", "rhs",
                  21);
  sf_mex_addfield(*c6_info, sf_mex_duplicatearraysafe(&c6_lhs21), "lhs", "lhs",
                  21);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_div.m"), "context",
                  "context", 22);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut("coder.internal.div"), "name",
                  "name", 22);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 22);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/div.p"), "resolved",
                  "resolved", 22);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(1389311520U), "fileTimeLo",
                  "fileTimeLo", 22);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 22);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 22);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 22);
  sf_mex_assign(&c6_rhs22, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c6_lhs22, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c6_info, sf_mex_duplicatearraysafe(&c6_rhs22), "rhs", "rhs",
                  22);
  sf_mex_addfield(*c6_info, sf_mex_duplicatearraysafe(&c6_lhs22), "lhs", "lhs",
                  22);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/matfun/inv.m!checkcond"),
                  "context", "context", 23);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut("norm"), "name", "name", 23);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 23);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/matfun/norm.m"), "resolved",
                  "resolved", 23);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(1363717468U), "fileTimeLo",
                  "fileTimeLo", 23);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 23);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 23);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 23);
  sf_mex_assign(&c6_rhs23, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c6_lhs23, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c6_info, sf_mex_duplicatearraysafe(&c6_rhs23), "rhs", "rhs",
                  23);
  sf_mex_addfield(*c6_info, sf_mex_duplicatearraysafe(&c6_lhs23), "lhs", "lhs",
                  23);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/matfun/norm.m"), "context",
                  "context", 24);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(
    "coder.internal.isBuiltInNumeric"), "name", "name", 24);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 24);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/shared/coder/coder/+coder/+internal/isBuiltInNumeric.m"),
                  "resolved", "resolved", 24);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(1363718156U), "fileTimeLo",
                  "fileTimeLo", 24);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 24);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 24);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 24);
  sf_mex_assign(&c6_rhs24, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c6_lhs24, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c6_info, sf_mex_duplicatearraysafe(&c6_rhs24), "rhs", "rhs",
                  24);
  sf_mex_addfield(*c6_info, sf_mex_duplicatearraysafe(&c6_lhs24), "lhs", "lhs",
                  24);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/matfun/norm.m!mat1norm"),
                  "context", "context", 25);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut("abs"), "name", "name", 25);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 25);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/abs.m"), "resolved",
                  "resolved", 25);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(1363717452U), "fileTimeLo",
                  "fileTimeLo", 25);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 25);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 25);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 25);
  sf_mex_assign(&c6_rhs25, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c6_lhs25, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c6_info, sf_mex_duplicatearraysafe(&c6_rhs25), "rhs", "rhs",
                  25);
  sf_mex_addfield(*c6_info, sf_mex_duplicatearraysafe(&c6_lhs25), "lhs", "lhs",
                  25);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/matfun/norm.m!mat1norm"),
                  "context", "context", 26);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut("isnan"), "name", "name", 26);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 26);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/isnan.m"), "resolved",
                  "resolved", 26);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(1363717458U), "fileTimeLo",
                  "fileTimeLo", 26);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 26);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 26);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 26);
  sf_mex_assign(&c6_rhs26, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c6_lhs26, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c6_info, sf_mex_duplicatearraysafe(&c6_rhs26), "rhs", "rhs",
                  26);
  sf_mex_addfield(*c6_info, sf_mex_duplicatearraysafe(&c6_lhs26), "lhs", "lhs",
                  26);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/isnan.m"), "context",
                  "context", 27);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(
    "coder.internal.isBuiltInNumeric"), "name", "name", 27);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 27);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/shared/coder/coder/+coder/+internal/isBuiltInNumeric.m"),
                  "resolved", "resolved", 27);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(1363718156U), "fileTimeLo",
                  "fileTimeLo", 27);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 27);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 27);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 27);
  sf_mex_assign(&c6_rhs27, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c6_lhs27, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c6_info, sf_mex_duplicatearraysafe(&c6_rhs27), "rhs", "rhs",
                  27);
  sf_mex_addfield(*c6_info, sf_mex_duplicatearraysafe(&c6_lhs27), "lhs", "lhs",
                  27);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/matfun/norm.m!mat1norm"),
                  "context", "context", 28);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut("eml_guarded_nan"), "name",
                  "name", 28);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut("char"), "dominantType",
                  "dominantType", 28);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_guarded_nan.m"),
                  "resolved", "resolved", 28);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(1286825976U), "fileTimeLo",
                  "fileTimeLo", 28);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 28);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 28);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 28);
  sf_mex_assign(&c6_rhs28, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c6_lhs28, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c6_info, sf_mex_duplicatearraysafe(&c6_rhs28), "rhs", "rhs",
                  28);
  sf_mex_addfield(*c6_info, sf_mex_duplicatearraysafe(&c6_lhs28), "lhs", "lhs",
                  28);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_guarded_nan.m"),
                  "context", "context", 29);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut("eml_is_float_class"), "name",
                  "name", 29);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut("char"), "dominantType",
                  "dominantType", 29);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_is_float_class.m"),
                  "resolved", "resolved", 29);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(1286825982U), "fileTimeLo",
                  "fileTimeLo", 29);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 29);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 29);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 29);
  sf_mex_assign(&c6_rhs29, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c6_lhs29, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c6_info, sf_mex_duplicatearraysafe(&c6_rhs29), "rhs", "rhs",
                  29);
  sf_mex_addfield(*c6_info, sf_mex_duplicatearraysafe(&c6_lhs29), "lhs", "lhs",
                  29);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/matfun/inv.m!checkcond"),
                  "context", "context", 30);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut("eml_warning"), "name", "name",
                  30);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut("char"), "dominantType",
                  "dominantType", 30);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_warning.m"), "resolved",
                  "resolved", 30);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(1286826002U), "fileTimeLo",
                  "fileTimeLo", 30);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 30);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 30);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 30);
  sf_mex_assign(&c6_rhs30, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c6_lhs30, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c6_info, sf_mex_duplicatearraysafe(&c6_rhs30), "rhs", "rhs",
                  30);
  sf_mex_addfield(*c6_info, sf_mex_duplicatearraysafe(&c6_lhs30), "lhs", "lhs",
                  30);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/matfun/inv.m!checkcond"),
                  "context", "context", 31);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut("isnan"), "name", "name", 31);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 31);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/isnan.m"), "resolved",
                  "resolved", 31);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(1363717458U), "fileTimeLo",
                  "fileTimeLo", 31);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 31);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 31);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 31);
  sf_mex_assign(&c6_rhs31, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c6_lhs31, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c6_info, sf_mex_duplicatearraysafe(&c6_rhs31), "rhs", "rhs",
                  31);
  sf_mex_addfield(*c6_info, sf_mex_duplicatearraysafe(&c6_lhs31), "lhs", "lhs",
                  31);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/matfun/inv.m!checkcond"),
                  "context", "context", 32);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut("eps"), "name", "name", 32);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut("char"), "dominantType",
                  "dominantType", 32);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/eps.m"), "resolved",
                  "resolved", 32);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(1326731596U), "fileTimeLo",
                  "fileTimeLo", 32);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 32);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 32);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 32);
  sf_mex_assign(&c6_rhs32, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c6_lhs32, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c6_info, sf_mex_duplicatearraysafe(&c6_rhs32), "rhs", "rhs",
                  32);
  sf_mex_addfield(*c6_info, sf_mex_duplicatearraysafe(&c6_lhs32), "lhs", "lhs",
                  32);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/eps.m"), "context",
                  "context", 33);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut("eml_is_float_class"), "name",
                  "name", 33);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut("char"), "dominantType",
                  "dominantType", 33);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_is_float_class.m"),
                  "resolved", "resolved", 33);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(1286825982U), "fileTimeLo",
                  "fileTimeLo", 33);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 33);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 33);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 33);
  sf_mex_assign(&c6_rhs33, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c6_lhs33, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c6_info, sf_mex_duplicatearraysafe(&c6_rhs33), "rhs", "rhs",
                  33);
  sf_mex_addfield(*c6_info, sf_mex_duplicatearraysafe(&c6_lhs33), "lhs", "lhs",
                  33);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/eps.m"), "context",
                  "context", 34);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut("eml_eps"), "name", "name", 34);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut("char"), "dominantType",
                  "dominantType", 34);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_eps.m"), "resolved",
                  "resolved", 34);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(1326731596U), "fileTimeLo",
                  "fileTimeLo", 34);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 34);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 34);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 34);
  sf_mex_assign(&c6_rhs34, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c6_lhs34, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c6_info, sf_mex_duplicatearraysafe(&c6_rhs34), "rhs", "rhs",
                  34);
  sf_mex_addfield(*c6_info, sf_mex_duplicatearraysafe(&c6_lhs34), "lhs", "lhs",
                  34);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_eps.m"), "context",
                  "context", 35);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut("eml_float_model"), "name",
                  "name", 35);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut("char"), "dominantType",
                  "dominantType", 35);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_float_model.m"),
                  "resolved", "resolved", 35);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(1326731596U), "fileTimeLo",
                  "fileTimeLo", 35);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 35);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 35);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 35);
  sf_mex_assign(&c6_rhs35, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c6_lhs35, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c6_info, sf_mex_duplicatearraysafe(&c6_rhs35), "rhs", "rhs",
                  35);
  sf_mex_addfield(*c6_info, sf_mex_duplicatearraysafe(&c6_lhs35), "lhs", "lhs",
                  35);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/matfun/inv.m!checkcond"),
                  "context", "context", 36);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut("eml_flt2str"), "name", "name",
                  36);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 36);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_flt2str.m"), "resolved",
                  "resolved", 36);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(1360285950U), "fileTimeLo",
                  "fileTimeLo", 36);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 36);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 36);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 36);
  sf_mex_assign(&c6_rhs36, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c6_lhs36, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c6_info, sf_mex_duplicatearraysafe(&c6_rhs36), "rhs", "rhs",
                  36);
  sf_mex_addfield(*c6_info, sf_mex_duplicatearraysafe(&c6_lhs36), "lhs", "lhs",
                  36);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_flt2str.m"), "context",
                  "context", 37);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut("char"), "name", "name", 37);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 37);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/strfun/char.m"), "resolved",
                  "resolved", 37);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(1319737168U), "fileTimeLo",
                  "fileTimeLo", 37);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 37);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 37);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 37);
  sf_mex_assign(&c6_rhs37, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c6_lhs37, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c6_info, sf_mex_duplicatearraysafe(&c6_rhs37), "rhs", "rhs",
                  37);
  sf_mex_addfield(*c6_info, sf_mex_duplicatearraysafe(&c6_lhs37), "lhs", "lhs",
                  37);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(""), "context", "context", 38);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut("eml_mtimes_helper"), "name",
                  "name", 38);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(""), "dominantType",
                  "dominantType", 38);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/eml_mtimes_helper.m"),
                  "resolved", "resolved", 38);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(1383880894U), "fileTimeLo",
                  "fileTimeLo", 38);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 38);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 38);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 38);
  sf_mex_assign(&c6_rhs38, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c6_lhs38, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c6_info, sf_mex_duplicatearraysafe(&c6_rhs38), "rhs", "rhs",
                  38);
  sf_mex_addfield(*c6_info, sf_mex_duplicatearraysafe(&c6_lhs38), "lhs", "lhs",
                  38);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/eml_mtimes_helper.m!common_checks"),
                  "context", "context", 39);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(
    "coder.internal.isBuiltInNumeric"), "name", "name", 39);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 39);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/shared/coder/coder/+coder/+internal/isBuiltInNumeric.m"),
                  "resolved", "resolved", 39);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(1363718156U), "fileTimeLo",
                  "fileTimeLo", 39);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 39);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 39);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 39);
  sf_mex_assign(&c6_rhs39, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c6_lhs39, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c6_info, sf_mex_duplicatearraysafe(&c6_rhs39), "rhs", "rhs",
                  39);
  sf_mex_addfield(*c6_info, sf_mex_duplicatearraysafe(&c6_lhs39), "lhs", "lhs",
                  39);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/eml_mtimes_helper.m"),
                  "context", "context", 40);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut("eml_index_class"), "name",
                  "name", 40);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(""), "dominantType",
                  "dominantType", 40);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_class.m"),
                  "resolved", "resolved", 40);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(1323174178U), "fileTimeLo",
                  "fileTimeLo", 40);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 40);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 40);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 40);
  sf_mex_assign(&c6_rhs40, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c6_lhs40, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c6_info, sf_mex_duplicatearraysafe(&c6_rhs40), "rhs", "rhs",
                  40);
  sf_mex_addfield(*c6_info, sf_mex_duplicatearraysafe(&c6_lhs40), "lhs", "lhs",
                  40);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/eml_mtimes_helper.m"),
                  "context", "context", 41);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut("eml_scalar_eg"), "name",
                  "name", 41);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 41);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalar_eg.m"), "resolved",
                  "resolved", 41);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(1375987888U), "fileTimeLo",
                  "fileTimeLo", 41);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 41);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 41);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 41);
  sf_mex_assign(&c6_rhs41, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c6_lhs41, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c6_info, sf_mex_duplicatearraysafe(&c6_rhs41), "rhs", "rhs",
                  41);
  sf_mex_addfield(*c6_info, sf_mex_duplicatearraysafe(&c6_lhs41), "lhs", "lhs",
                  41);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/eml_mtimes_helper.m"),
                  "context", "context", 42);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut("eml_xgemm"), "name", "name",
                  42);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut("char"), "dominantType",
                  "dominantType", 42);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/eml_xgemm.m"),
                  "resolved", "resolved", 42);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(1375987890U), "fileTimeLo",
                  "fileTimeLo", 42);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 42);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 42);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 42);
  sf_mex_assign(&c6_rhs42, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c6_lhs42, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c6_info, sf_mex_duplicatearraysafe(&c6_rhs42), "rhs", "rhs",
                  42);
  sf_mex_addfield(*c6_info, sf_mex_duplicatearraysafe(&c6_lhs42), "lhs", "lhs",
                  42);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/eml_xgemm.m"), "context",
                  "context", 43);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut("coder.internal.blas.inline"),
                  "name", "name", 43);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(""), "dominantType",
                  "dominantType", 43);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+blas/inline.p"),
                  "resolved", "resolved", 43);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(1389311522U), "fileTimeLo",
                  "fileTimeLo", 43);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 43);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 43);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 43);
  sf_mex_assign(&c6_rhs43, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c6_lhs43, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c6_info, sf_mex_duplicatearraysafe(&c6_rhs43), "rhs", "rhs",
                  43);
  sf_mex_addfield(*c6_info, sf_mex_duplicatearraysafe(&c6_lhs43), "lhs", "lhs",
                  43);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/eml_xgemm.m"), "context",
                  "context", 44);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut("coder.internal.blas.xgemm"),
                  "name", "name", 44);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut("char"), "dominantType",
                  "dominantType", 44);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+blas/xgemm.p"),
                  "resolved", "resolved", 44);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(1389311522U), "fileTimeLo",
                  "fileTimeLo", 44);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 44);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 44);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 44);
  sf_mex_assign(&c6_rhs44, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c6_lhs44, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c6_info, sf_mex_duplicatearraysafe(&c6_rhs44), "rhs", "rhs",
                  44);
  sf_mex_addfield(*c6_info, sf_mex_duplicatearraysafe(&c6_lhs44), "lhs", "lhs",
                  44);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+blas/xgemm.p"),
                  "context", "context", 45);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(
    "coder.internal.blas.use_refblas"), "name", "name", 45);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(""), "dominantType",
                  "dominantType", 45);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+blas/use_refblas.p"),
                  "resolved", "resolved", 45);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(1389311522U), "fileTimeLo",
                  "fileTimeLo", 45);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 45);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 45);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 45);
  sf_mex_assign(&c6_rhs45, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c6_lhs45, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c6_info, sf_mex_duplicatearraysafe(&c6_rhs45), "rhs", "rhs",
                  45);
  sf_mex_addfield(*c6_info, sf_mex_duplicatearraysafe(&c6_lhs45), "lhs", "lhs",
                  45);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+blas/xgemm.p!below_threshold"),
                  "context", "context", 46);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut("coder.internal.blas.threshold"),
                  "name", "name", 46);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut("char"), "dominantType",
                  "dominantType", 46);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+blas/threshold.p"),
                  "resolved", "resolved", 46);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(1389311522U), "fileTimeLo",
                  "fileTimeLo", 46);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 46);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 46);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 46);
  sf_mex_assign(&c6_rhs46, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c6_lhs46, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c6_info, sf_mex_duplicatearraysafe(&c6_rhs46), "rhs", "rhs",
                  46);
  sf_mex_addfield(*c6_info, sf_mex_duplicatearraysafe(&c6_lhs46), "lhs", "lhs",
                  46);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+blas/threshold.p"),
                  "context", "context", 47);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut("eml_switch_helper"), "name",
                  "name", 47);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(""), "dominantType",
                  "dominantType", 47);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_switch_helper.m"),
                  "resolved", "resolved", 47);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(1381857500U), "fileTimeLo",
                  "fileTimeLo", 47);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 47);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 47);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 47);
  sf_mex_assign(&c6_rhs47, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c6_lhs47, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c6_info, sf_mex_duplicatearraysafe(&c6_rhs47), "rhs", "rhs",
                  47);
  sf_mex_addfield(*c6_info, sf_mex_duplicatearraysafe(&c6_lhs47), "lhs", "lhs",
                  47);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+blas/xgemm.p"),
                  "context", "context", 48);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut("coder.internal.scalarEg"),
                  "name", "name", 48);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 48);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/scalarEg.p"),
                  "resolved", "resolved", 48);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(1389311520U), "fileTimeLo",
                  "fileTimeLo", 48);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 48);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 48);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 48);
  sf_mex_assign(&c6_rhs48, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c6_lhs48, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c6_info, sf_mex_duplicatearraysafe(&c6_rhs48), "rhs", "rhs",
                  48);
  sf_mex_addfield(*c6_info, sf_mex_duplicatearraysafe(&c6_lhs48), "lhs", "lhs",
                  48);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+blas/xgemm.p"),
                  "context", "context", 49);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut("coder.internal.refblas.xgemm"),
                  "name", "name", 49);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut("char"), "dominantType",
                  "dominantType", 49);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+refblas/xgemm.p"),
                  "resolved", "resolved", 49);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(1389311522U), "fileTimeLo",
                  "fileTimeLo", 49);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 49);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 49);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 49);
  sf_mex_assign(&c6_rhs49, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c6_lhs49, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c6_info, sf_mex_duplicatearraysafe(&c6_rhs49), "rhs", "rhs",
                  49);
  sf_mex_addfield(*c6_info, sf_mex_duplicatearraysafe(&c6_lhs49), "lhs", "lhs",
                  49);
  sf_mex_destroy(&c6_rhs0);
  sf_mex_destroy(&c6_lhs0);
  sf_mex_destroy(&c6_rhs1);
  sf_mex_destroy(&c6_lhs1);
  sf_mex_destroy(&c6_rhs2);
  sf_mex_destroy(&c6_lhs2);
  sf_mex_destroy(&c6_rhs3);
  sf_mex_destroy(&c6_lhs3);
  sf_mex_destroy(&c6_rhs4);
  sf_mex_destroy(&c6_lhs4);
  sf_mex_destroy(&c6_rhs5);
  sf_mex_destroy(&c6_lhs5);
  sf_mex_destroy(&c6_rhs6);
  sf_mex_destroy(&c6_lhs6);
  sf_mex_destroy(&c6_rhs7);
  sf_mex_destroy(&c6_lhs7);
  sf_mex_destroy(&c6_rhs8);
  sf_mex_destroy(&c6_lhs8);
  sf_mex_destroy(&c6_rhs9);
  sf_mex_destroy(&c6_lhs9);
  sf_mex_destroy(&c6_rhs10);
  sf_mex_destroy(&c6_lhs10);
  sf_mex_destroy(&c6_rhs11);
  sf_mex_destroy(&c6_lhs11);
  sf_mex_destroy(&c6_rhs12);
  sf_mex_destroy(&c6_lhs12);
  sf_mex_destroy(&c6_rhs13);
  sf_mex_destroy(&c6_lhs13);
  sf_mex_destroy(&c6_rhs14);
  sf_mex_destroy(&c6_lhs14);
  sf_mex_destroy(&c6_rhs15);
  sf_mex_destroy(&c6_lhs15);
  sf_mex_destroy(&c6_rhs16);
  sf_mex_destroy(&c6_lhs16);
  sf_mex_destroy(&c6_rhs17);
  sf_mex_destroy(&c6_lhs17);
  sf_mex_destroy(&c6_rhs18);
  sf_mex_destroy(&c6_lhs18);
  sf_mex_destroy(&c6_rhs19);
  sf_mex_destroy(&c6_lhs19);
  sf_mex_destroy(&c6_rhs20);
  sf_mex_destroy(&c6_lhs20);
  sf_mex_destroy(&c6_rhs21);
  sf_mex_destroy(&c6_lhs21);
  sf_mex_destroy(&c6_rhs22);
  sf_mex_destroy(&c6_lhs22);
  sf_mex_destroy(&c6_rhs23);
  sf_mex_destroy(&c6_lhs23);
  sf_mex_destroy(&c6_rhs24);
  sf_mex_destroy(&c6_lhs24);
  sf_mex_destroy(&c6_rhs25);
  sf_mex_destroy(&c6_lhs25);
  sf_mex_destroy(&c6_rhs26);
  sf_mex_destroy(&c6_lhs26);
  sf_mex_destroy(&c6_rhs27);
  sf_mex_destroy(&c6_lhs27);
  sf_mex_destroy(&c6_rhs28);
  sf_mex_destroy(&c6_lhs28);
  sf_mex_destroy(&c6_rhs29);
  sf_mex_destroy(&c6_lhs29);
  sf_mex_destroy(&c6_rhs30);
  sf_mex_destroy(&c6_lhs30);
  sf_mex_destroy(&c6_rhs31);
  sf_mex_destroy(&c6_lhs31);
  sf_mex_destroy(&c6_rhs32);
  sf_mex_destroy(&c6_lhs32);
  sf_mex_destroy(&c6_rhs33);
  sf_mex_destroy(&c6_lhs33);
  sf_mex_destroy(&c6_rhs34);
  sf_mex_destroy(&c6_lhs34);
  sf_mex_destroy(&c6_rhs35);
  sf_mex_destroy(&c6_lhs35);
  sf_mex_destroy(&c6_rhs36);
  sf_mex_destroy(&c6_lhs36);
  sf_mex_destroy(&c6_rhs37);
  sf_mex_destroy(&c6_lhs37);
  sf_mex_destroy(&c6_rhs38);
  sf_mex_destroy(&c6_lhs38);
  sf_mex_destroy(&c6_rhs39);
  sf_mex_destroy(&c6_lhs39);
  sf_mex_destroy(&c6_rhs40);
  sf_mex_destroy(&c6_lhs40);
  sf_mex_destroy(&c6_rhs41);
  sf_mex_destroy(&c6_lhs41);
  sf_mex_destroy(&c6_rhs42);
  sf_mex_destroy(&c6_lhs42);
  sf_mex_destroy(&c6_rhs43);
  sf_mex_destroy(&c6_lhs43);
  sf_mex_destroy(&c6_rhs44);
  sf_mex_destroy(&c6_lhs44);
  sf_mex_destroy(&c6_rhs45);
  sf_mex_destroy(&c6_lhs45);
  sf_mex_destroy(&c6_rhs46);
  sf_mex_destroy(&c6_lhs46);
  sf_mex_destroy(&c6_rhs47);
  sf_mex_destroy(&c6_lhs47);
  sf_mex_destroy(&c6_rhs48);
  sf_mex_destroy(&c6_lhs48);
  sf_mex_destroy(&c6_rhs49);
  sf_mex_destroy(&c6_lhs49);
}

static const mxArray *c6_emlrt_marshallOut(const char * c6_u)
{
  const mxArray *c6_y = NULL;
  c6_y = NULL;
  sf_mex_assign(&c6_y, sf_mex_create("y", c6_u, 15, 0U, 0U, 0U, 2, 1, strlen
    (c6_u)), false);
  return c6_y;
}

static const mxArray *c6_b_emlrt_marshallOut(const uint32_T c6_u)
{
  const mxArray *c6_y = NULL;
  c6_y = NULL;
  sf_mex_assign(&c6_y, sf_mex_create("y", &c6_u, 7, 0U, 0U, 0U, 0), false);
  return c6_y;
}

static void c6_mpower(SFc6_TwoDOF_PID_joint_spaceInstanceStruct *chartInstance,
                      real_T c6_a[4], real_T c6_c[4])
{
  int32_T c6_i36;
  real_T c6_b_a[4];
  int32_T c6_i37;
  real_T c6_c_a[4];
  real_T c6_n1x;
  int32_T c6_i38;
  real_T c6_b_c[4];
  real_T c6_n1xinv;
  real_T c6_rc;
  real_T c6_x;
  boolean_T c6_b;
  real_T c6_b_x;
  int32_T c6_i39;
  static char_T c6_cv0[8] = { '%', '%', '%', 'd', '.', '%', 'd', 'e' };

  char_T c6_u[8];
  const mxArray *c6_y = NULL;
  real_T c6_b_u;
  const mxArray *c6_b_y = NULL;
  real_T c6_c_u;
  const mxArray *c6_c_y = NULL;
  real_T c6_d_u;
  const mxArray *c6_d_y = NULL;
  char_T c6_str[14];
  int32_T c6_i40;
  char_T c6_b_str[14];
  boolean_T guard1 = false;
  boolean_T guard2 = false;
  boolean_T guard3 = false;
  for (c6_i36 = 0; c6_i36 < 4; c6_i36++) {
    c6_b_a[c6_i36] = c6_a[c6_i36];
  }

  c6_inv2x2(chartInstance, c6_b_a, c6_c);
  for (c6_i37 = 0; c6_i37 < 4; c6_i37++) {
    c6_c_a[c6_i37] = c6_a[c6_i37];
  }

  c6_n1x = c6_norm(chartInstance, c6_c_a);
  for (c6_i38 = 0; c6_i38 < 4; c6_i38++) {
    c6_b_c[c6_i38] = c6_c[c6_i38];
  }

  c6_n1xinv = c6_norm(chartInstance, c6_b_c);
  c6_rc = 1.0 / (c6_n1x * c6_n1xinv);
  guard1 = false;
  guard2 = false;
  if (c6_n1x == 0.0) {
    guard2 = true;
  } else if (c6_n1xinv == 0.0) {
    guard2 = true;
  } else if (c6_rc == 0.0) {
    guard1 = true;
  } else {
    c6_x = c6_rc;
    c6_b = muDoubleScalarIsNaN(c6_x);
    guard3 = false;
    if (c6_b) {
      guard3 = true;
    } else {
      if (c6_rc < 2.2204460492503131E-16) {
        guard3 = true;
      }
    }

    if (guard3 == true) {
      c6_b_x = c6_rc;
      for (c6_i39 = 0; c6_i39 < 8; c6_i39++) {
        c6_u[c6_i39] = c6_cv0[c6_i39];
      }

      c6_y = NULL;
      sf_mex_assign(&c6_y, sf_mex_create("y", c6_u, 10, 0U, 1U, 0U, 2, 1, 8),
                    false);
      c6_b_u = 14.0;
      c6_b_y = NULL;
      sf_mex_assign(&c6_b_y, sf_mex_create("y", &c6_b_u, 0, 0U, 0U, 0U, 0),
                    false);
      c6_c_u = 6.0;
      c6_c_y = NULL;
      sf_mex_assign(&c6_c_y, sf_mex_create("y", &c6_c_u, 0, 0U, 0U, 0U, 0),
                    false);
      c6_d_u = c6_b_x;
      c6_d_y = NULL;
      sf_mex_assign(&c6_d_y, sf_mex_create("y", &c6_d_u, 0, 0U, 0U, 0U, 0),
                    false);
      c6_f_emlrt_marshallIn(chartInstance, sf_mex_call_debug
                            (sfGlobalDebugInstanceStruct, "sprintf", 1U, 2U, 14,
        sf_mex_call_debug(sfGlobalDebugInstanceStruct, "sprintf", 1U, 3U, 14,
                          c6_y, 14, c6_b_y, 14, c6_c_y), 14, c6_d_y), "sprintf",
                            c6_str);
      for (c6_i40 = 0; c6_i40 < 14; c6_i40++) {
        c6_b_str[c6_i40] = c6_str[c6_i40];
      }

      c6_b_eml_warning(chartInstance, c6_b_str);
    }
  }

  if (guard2 == true) {
    guard1 = true;
  }

  if (guard1 == true) {
    c6_eml_warning(chartInstance);
  }
}

static void c6_inv2x2(SFc6_TwoDOF_PID_joint_spaceInstanceStruct *chartInstance,
                      real_T c6_x[4], real_T c6_y[4])
{
  real_T c6_b_x;
  real_T c6_c_x;
  real_T c6_d_x;
  real_T c6_e_x;
  real_T c6_b_y;
  real_T c6_f_x;
  real_T c6_g_x;
  real_T c6_c_y;
  real_T c6_d;
  real_T c6_h_x;
  real_T c6_i_x;
  real_T c6_j_x;
  real_T c6_k_x;
  real_T c6_d_y;
  real_T c6_l_x;
  real_T c6_m_x;
  real_T c6_e_y;
  real_T c6_b_d;
  real_T c6_A;
  real_T c6_B;
  real_T c6_n_x;
  real_T c6_f_y;
  real_T c6_o_x;
  real_T c6_g_y;
  real_T c6_p_x;
  real_T c6_h_y;
  real_T c6_r;
  real_T c6_b_B;
  real_T c6_i_y;
  real_T c6_j_y;
  real_T c6_k_y;
  real_T c6_t;
  real_T c6_b_A;
  real_T c6_c_B;
  real_T c6_q_x;
  real_T c6_l_y;
  real_T c6_r_x;
  real_T c6_m_y;
  real_T c6_s_x;
  real_T c6_n_y;
  real_T c6_o_y;
  real_T c6_c_A;
  real_T c6_d_B;
  real_T c6_t_x;
  real_T c6_p_y;
  real_T c6_u_x;
  real_T c6_q_y;
  real_T c6_v_x;
  real_T c6_r_y;
  real_T c6_s_y;
  real_T c6_d_A;
  real_T c6_e_B;
  real_T c6_w_x;
  real_T c6_t_y;
  real_T c6_x_x;
  real_T c6_u_y;
  real_T c6_y_x;
  real_T c6_v_y;
  real_T c6_f_B;
  real_T c6_w_y;
  real_T c6_x_y;
  real_T c6_y_y;
  real_T c6_e_A;
  real_T c6_g_B;
  real_T c6_ab_x;
  real_T c6_ab_y;
  real_T c6_bb_x;
  real_T c6_bb_y;
  real_T c6_cb_x;
  real_T c6_cb_y;
  real_T c6_db_y;
  real_T c6_f_A;
  real_T c6_h_B;
  real_T c6_db_x;
  real_T c6_eb_y;
  real_T c6_eb_x;
  real_T c6_fb_y;
  real_T c6_fb_x;
  real_T c6_gb_y;
  real_T c6_hb_y;
  (void)chartInstance;
  c6_b_x = c6_x[1];
  c6_c_x = c6_b_x;
  c6_d_x = c6_c_x;
  c6_e_x = c6_d_x;
  c6_b_y = muDoubleScalarAbs(c6_e_x);
  c6_f_x = 0.0;
  c6_g_x = c6_f_x;
  c6_c_y = muDoubleScalarAbs(c6_g_x);
  c6_d = c6_b_y + c6_c_y;
  c6_h_x = c6_x[0];
  c6_i_x = c6_h_x;
  c6_j_x = c6_i_x;
  c6_k_x = c6_j_x;
  c6_d_y = muDoubleScalarAbs(c6_k_x);
  c6_l_x = 0.0;
  c6_m_x = c6_l_x;
  c6_e_y = muDoubleScalarAbs(c6_m_x);
  c6_b_d = c6_d_y + c6_e_y;
  if (c6_d > c6_b_d) {
    c6_A = c6_x[0];
    c6_B = c6_x[1];
    c6_n_x = c6_A;
    c6_f_y = c6_B;
    c6_o_x = c6_n_x;
    c6_g_y = c6_f_y;
    c6_p_x = c6_o_x;
    c6_h_y = c6_g_y;
    c6_r = c6_p_x / c6_h_y;
    c6_b_B = c6_r * c6_x[3] - c6_x[2];
    c6_i_y = c6_b_B;
    c6_j_y = c6_i_y;
    c6_k_y = c6_j_y;
    c6_t = 1.0 / c6_k_y;
    c6_b_A = c6_x[3];
    c6_c_B = c6_x[1];
    c6_q_x = c6_b_A;
    c6_l_y = c6_c_B;
    c6_r_x = c6_q_x;
    c6_m_y = c6_l_y;
    c6_s_x = c6_r_x;
    c6_n_y = c6_m_y;
    c6_o_y = c6_s_x / c6_n_y;
    c6_y[0] = c6_o_y * c6_t;
    c6_y[1] = -c6_t;
    c6_c_A = -c6_x[2];
    c6_d_B = c6_x[1];
    c6_t_x = c6_c_A;
    c6_p_y = c6_d_B;
    c6_u_x = c6_t_x;
    c6_q_y = c6_p_y;
    c6_v_x = c6_u_x;
    c6_r_y = c6_q_y;
    c6_s_y = c6_v_x / c6_r_y;
    c6_y[2] = c6_s_y * c6_t;
    c6_y[3] = c6_r * c6_t;
  } else {
    c6_d_A = c6_x[1];
    c6_e_B = c6_x[0];
    c6_w_x = c6_d_A;
    c6_t_y = c6_e_B;
    c6_x_x = c6_w_x;
    c6_u_y = c6_t_y;
    c6_y_x = c6_x_x;
    c6_v_y = c6_u_y;
    c6_r = c6_y_x / c6_v_y;
    c6_f_B = c6_x[3] - c6_r * c6_x[2];
    c6_w_y = c6_f_B;
    c6_x_y = c6_w_y;
    c6_y_y = c6_x_y;
    c6_t = 1.0 / c6_y_y;
    c6_e_A = c6_x[3];
    c6_g_B = c6_x[0];
    c6_ab_x = c6_e_A;
    c6_ab_y = c6_g_B;
    c6_bb_x = c6_ab_x;
    c6_bb_y = c6_ab_y;
    c6_cb_x = c6_bb_x;
    c6_cb_y = c6_bb_y;
    c6_db_y = c6_cb_x / c6_cb_y;
    c6_y[0] = c6_db_y * c6_t;
    c6_y[1] = -c6_r * c6_t;
    c6_f_A = -c6_x[2];
    c6_h_B = c6_x[0];
    c6_db_x = c6_f_A;
    c6_eb_y = c6_h_B;
    c6_eb_x = c6_db_x;
    c6_fb_y = c6_eb_y;
    c6_fb_x = c6_eb_x;
    c6_gb_y = c6_fb_y;
    c6_hb_y = c6_fb_x / c6_gb_y;
    c6_y[2] = c6_hb_y * c6_t;
    c6_y[3] = c6_t;
  }
}

static real_T c6_norm(SFc6_TwoDOF_PID_joint_spaceInstanceStruct *chartInstance,
                      real_T c6_x[4])
{
  real_T c6_y;
  int32_T c6_j;
  real_T c6_b_j;
  real_T c6_s;
  int32_T c6_i;
  real_T c6_b_i;
  real_T c6_b_x;
  real_T c6_c_x;
  real_T c6_b_y;
  real_T c6_d_x;
  boolean_T c6_b;
  boolean_T exitg1;
  (void)chartInstance;
  c6_y = 0.0;
  c6_j = 0;
  exitg1 = false;
  while ((exitg1 == false) && (c6_j < 2)) {
    c6_b_j = 1.0 + (real_T)c6_j;
    c6_s = 0.0;
    for (c6_i = 0; c6_i < 2; c6_i++) {
      c6_b_i = 1.0 + (real_T)c6_i;
      c6_b_x = c6_x[(_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK
                      ("", c6_b_i), 1, 2, 1, 0) + ((_SFD_EML_ARRAY_BOUNDS_CHECK(
        "", (int32_T)_SFD_INTEGER_CHECK("", c6_b_j), 1, 2, 2, 0) - 1) << 1)) - 1];
      c6_c_x = c6_b_x;
      c6_b_y = muDoubleScalarAbs(c6_c_x);
      c6_s += c6_b_y;
    }

    c6_d_x = c6_s;
    c6_b = muDoubleScalarIsNaN(c6_d_x);
    if (c6_b) {
      c6_y = rtNaN;
      exitg1 = true;
    } else {
      if (c6_s > c6_y) {
        c6_y = c6_s;
      }

      c6_j++;
    }
  }

  return c6_y;
}

static void c6_eml_warning(SFc6_TwoDOF_PID_joint_spaceInstanceStruct
  *chartInstance)
{
  int32_T c6_i41;
  static char_T c6_varargin_1[27] = { 'C', 'o', 'd', 'e', 'r', ':', 'M', 'A',
    'T', 'L', 'A', 'B', ':', 's', 'i', 'n', 'g', 'u', 'l', 'a', 'r', 'M', 'a',
    't', 'r', 'i', 'x' };

  char_T c6_u[27];
  const mxArray *c6_y = NULL;
  (void)chartInstance;
  for (c6_i41 = 0; c6_i41 < 27; c6_i41++) {
    c6_u[c6_i41] = c6_varargin_1[c6_i41];
  }

  c6_y = NULL;
  sf_mex_assign(&c6_y, sf_mex_create("y", c6_u, 10, 0U, 1U, 0U, 2, 1, 27), false);
  sf_mex_call_debug(sfGlobalDebugInstanceStruct, "warning", 0U, 1U, 14,
                    sf_mex_call_debug(sfGlobalDebugInstanceStruct, "message", 1U,
    1U, 14, c6_y));
}

static void c6_b_eml_warning(SFc6_TwoDOF_PID_joint_spaceInstanceStruct
  *chartInstance, char_T c6_varargin_2[14])
{
  int32_T c6_i42;
  static char_T c6_varargin_1[33] = { 'C', 'o', 'd', 'e', 'r', ':', 'M', 'A',
    'T', 'L', 'A', 'B', ':', 'i', 'l', 'l', 'C', 'o', 'n', 'd', 'i', 't', 'i',
    'o', 'n', 'e', 'd', 'M', 'a', 't', 'r', 'i', 'x' };

  char_T c6_u[33];
  const mxArray *c6_y = NULL;
  int32_T c6_i43;
  char_T c6_b_u[14];
  const mxArray *c6_b_y = NULL;
  (void)chartInstance;
  for (c6_i42 = 0; c6_i42 < 33; c6_i42++) {
    c6_u[c6_i42] = c6_varargin_1[c6_i42];
  }

  c6_y = NULL;
  sf_mex_assign(&c6_y, sf_mex_create("y", c6_u, 10, 0U, 1U, 0U, 2, 1, 33), false);
  for (c6_i43 = 0; c6_i43 < 14; c6_i43++) {
    c6_b_u[c6_i43] = c6_varargin_2[c6_i43];
  }

  c6_b_y = NULL;
  sf_mex_assign(&c6_b_y, sf_mex_create("y", c6_b_u, 10, 0U, 1U, 0U, 2, 1, 14),
                false);
  sf_mex_call_debug(sfGlobalDebugInstanceStruct, "warning", 0U, 1U, 14,
                    sf_mex_call_debug(sfGlobalDebugInstanceStruct, "message", 1U,
    2U, 14, c6_y, 14, c6_b_y));
}

static void c6_eml_scalar_eg(SFc6_TwoDOF_PID_joint_spaceInstanceStruct
  *chartInstance)
{
  (void)chartInstance;
}

static void c6_eml_xgemm(SFc6_TwoDOF_PID_joint_spaceInstanceStruct
  *chartInstance, real_T c6_A[4], real_T c6_B[2], real_T c6_C[2], real_T c6_b_C
  [2])
{
  int32_T c6_i44;
  int32_T c6_i45;
  real_T c6_b_A[4];
  int32_T c6_i46;
  real_T c6_b_B[2];
  for (c6_i44 = 0; c6_i44 < 2; c6_i44++) {
    c6_b_C[c6_i44] = c6_C[c6_i44];
  }

  for (c6_i45 = 0; c6_i45 < 4; c6_i45++) {
    c6_b_A[c6_i45] = c6_A[c6_i45];
  }

  for (c6_i46 = 0; c6_i46 < 2; c6_i46++) {
    c6_b_B[c6_i46] = c6_B[c6_i46];
  }

  c6_b_eml_xgemm(chartInstance, c6_b_A, c6_b_B, c6_b_C);
}

static void c6_f_emlrt_marshallIn(SFc6_TwoDOF_PID_joint_spaceInstanceStruct
  *chartInstance, const mxArray *c6_sprintf, const char_T *c6_identifier, char_T
  c6_y[14])
{
  emlrtMsgIdentifier c6_thisId;
  c6_thisId.fIdentifier = c6_identifier;
  c6_thisId.fParent = NULL;
  c6_g_emlrt_marshallIn(chartInstance, sf_mex_dup(c6_sprintf), &c6_thisId, c6_y);
  sf_mex_destroy(&c6_sprintf);
}

static void c6_g_emlrt_marshallIn(SFc6_TwoDOF_PID_joint_spaceInstanceStruct
  *chartInstance, const mxArray *c6_u, const emlrtMsgIdentifier *c6_parentId,
  char_T c6_y[14])
{
  char_T c6_cv1[14];
  int32_T c6_i47;
  (void)chartInstance;
  sf_mex_import(c6_parentId, sf_mex_dup(c6_u), c6_cv1, 1, 10, 0U, 1, 0U, 2, 1,
                14);
  for (c6_i47 = 0; c6_i47 < 14; c6_i47++) {
    c6_y[c6_i47] = c6_cv1[c6_i47];
  }

  sf_mex_destroy(&c6_u);
}

static const mxArray *c6_d_sf_marshallOut(void *chartInstanceVoid, void
  *c6_inData)
{
  const mxArray *c6_mxArrayOutData = NULL;
  int32_T c6_u;
  const mxArray *c6_y = NULL;
  SFc6_TwoDOF_PID_joint_spaceInstanceStruct *chartInstance;
  chartInstance = (SFc6_TwoDOF_PID_joint_spaceInstanceStruct *)chartInstanceVoid;
  c6_mxArrayOutData = NULL;
  c6_u = *(int32_T *)c6_inData;
  c6_y = NULL;
  sf_mex_assign(&c6_y, sf_mex_create("y", &c6_u, 6, 0U, 0U, 0U, 0), false);
  sf_mex_assign(&c6_mxArrayOutData, c6_y, false);
  return c6_mxArrayOutData;
}

static int32_T c6_h_emlrt_marshallIn(SFc6_TwoDOF_PID_joint_spaceInstanceStruct
  *chartInstance, const mxArray *c6_u, const emlrtMsgIdentifier *c6_parentId)
{
  int32_T c6_y;
  int32_T c6_i48;
  (void)chartInstance;
  sf_mex_import(c6_parentId, sf_mex_dup(c6_u), &c6_i48, 1, 6, 0U, 0, 0U, 0);
  c6_y = c6_i48;
  sf_mex_destroy(&c6_u);
  return c6_y;
}

static void c6_d_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c6_mxArrayInData, const char_T *c6_varName, void *c6_outData)
{
  const mxArray *c6_b_sfEvent;
  const char_T *c6_identifier;
  emlrtMsgIdentifier c6_thisId;
  int32_T c6_y;
  SFc6_TwoDOF_PID_joint_spaceInstanceStruct *chartInstance;
  chartInstance = (SFc6_TwoDOF_PID_joint_spaceInstanceStruct *)chartInstanceVoid;
  c6_b_sfEvent = sf_mex_dup(c6_mxArrayInData);
  c6_identifier = c6_varName;
  c6_thisId.fIdentifier = c6_identifier;
  c6_thisId.fParent = NULL;
  c6_y = c6_h_emlrt_marshallIn(chartInstance, sf_mex_dup(c6_b_sfEvent),
    &c6_thisId);
  sf_mex_destroy(&c6_b_sfEvent);
  *(int32_T *)c6_outData = c6_y;
  sf_mex_destroy(&c6_mxArrayInData);
}

static uint8_T c6_i_emlrt_marshallIn(SFc6_TwoDOF_PID_joint_spaceInstanceStruct
  *chartInstance, const mxArray *c6_b_is_active_c6_TwoDOF_PID_joint_space, const
  char_T *c6_identifier)
{
  uint8_T c6_y;
  emlrtMsgIdentifier c6_thisId;
  c6_thisId.fIdentifier = c6_identifier;
  c6_thisId.fParent = NULL;
  c6_y = c6_j_emlrt_marshallIn(chartInstance, sf_mex_dup
    (c6_b_is_active_c6_TwoDOF_PID_joint_space), &c6_thisId);
  sf_mex_destroy(&c6_b_is_active_c6_TwoDOF_PID_joint_space);
  return c6_y;
}

static uint8_T c6_j_emlrt_marshallIn(SFc6_TwoDOF_PID_joint_spaceInstanceStruct
  *chartInstance, const mxArray *c6_u, const emlrtMsgIdentifier *c6_parentId)
{
  uint8_T c6_y;
  uint8_T c6_u0;
  (void)chartInstance;
  sf_mex_import(c6_parentId, sf_mex_dup(c6_u), &c6_u0, 1, 3, 0U, 0, 0U, 0);
  c6_y = c6_u0;
  sf_mex_destroy(&c6_u);
  return c6_y;
}

static void c6_b_eml_xgemm(SFc6_TwoDOF_PID_joint_spaceInstanceStruct
  *chartInstance, real_T c6_A[4], real_T c6_B[2], real_T c6_C[2])
{
  int32_T c6_i49;
  int32_T c6_i50;
  int32_T c6_i51;
  (void)chartInstance;
  for (c6_i49 = 0; c6_i49 < 2; c6_i49++) {
    c6_C[c6_i49] = 0.0;
    c6_i50 = 0;
    for (c6_i51 = 0; c6_i51 < 2; c6_i51++) {
      c6_C[c6_i49] += c6_A[c6_i50 + c6_i49] * c6_B[c6_i51];
      c6_i50 += 2;
    }
  }
}

static void init_dsm_address_info(SFc6_TwoDOF_PID_joint_spaceInstanceStruct
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

void sf_c6_TwoDOF_PID_joint_space_get_check_sum(mxArray *plhs[])
{
  ((real_T *)mxGetPr((plhs[0])))[0] = (real_T)(1346481059U);
  ((real_T *)mxGetPr((plhs[0])))[1] = (real_T)(236207059U);
  ((real_T *)mxGetPr((plhs[0])))[2] = (real_T)(45389413U);
  ((real_T *)mxGetPr((plhs[0])))[3] = (real_T)(1616806948U);
}

mxArray *sf_c6_TwoDOF_PID_joint_space_get_autoinheritance_info(void)
{
  const char *autoinheritanceFields[] = { "checksum", "inputs", "parameters",
    "outputs", "locals" };

  mxArray *mxAutoinheritanceInfo = mxCreateStructMatrix(1,1,5,
    autoinheritanceFields);

  {
    mxArray *mxChecksum = mxCreateString("wds9mxLqtz7VUDhFwlPm2B");
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

    mxArray *mxData = mxCreateStructMatrix(1,2,3,dataFields);

    {
      mxArray *mxSize = mxCreateDoubleMatrix(1,2,mxREAL);
      double *pr = mxGetPr(mxSize);
      pr[0] = (double)(2);
      pr[1] = (double)(2);
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
    mxSetField(mxAutoinheritanceInfo,0,"outputs",mxData);
  }

  {
    mxSetField(mxAutoinheritanceInfo,0,"locals",mxCreateDoubleMatrix(0,0,mxREAL));
  }

  return(mxAutoinheritanceInfo);
}

mxArray *sf_c6_TwoDOF_PID_joint_space_third_party_uses_info(void)
{
  mxArray * mxcell3p = mxCreateCellMatrix(1,0);
  return(mxcell3p);
}

mxArray *sf_c6_TwoDOF_PID_joint_space_updateBuildInfo_args_info(void)
{
  mxArray *mxBIArgs = mxCreateCellMatrix(1,0);
  return mxBIArgs;
}

static const mxArray *sf_get_sim_state_info_c6_TwoDOF_PID_joint_space(void)
{
  const char *infoFields[] = { "chartChecksum", "varInfo" };

  mxArray *mxInfo = mxCreateStructMatrix(1, 1, 2, infoFields);
  const char *infoEncStr[] = {
    "100 S1x3'type','srcId','name','auxInfo'{{M[1],M[7],T\"QP\",},{M[1],M[5],T\"invJ\",},{M[8],M[0],T\"is_active_c6_TwoDOF_PID_joint_space\",}}"
  };

  mxArray *mxVarInfo = sf_mex_decode_encoded_mx_struct_array(infoEncStr, 3, 10);
  mxArray *mxChecksum = mxCreateDoubleMatrix(1, 4, mxREAL);
  sf_c6_TwoDOF_PID_joint_space_get_check_sum(&mxChecksum);
  mxSetField(mxInfo, 0, infoFields[0], mxChecksum);
  mxSetField(mxInfo, 0, infoFields[1], mxVarInfo);
  return mxInfo;
}

static void chart_debug_initialization(SimStruct *S, unsigned int
  fullDebuggerInitialization)
{
  if (!sim_mode_is_rtw_gen(S)) {
    SFc6_TwoDOF_PID_joint_spaceInstanceStruct *chartInstance;
    ChartRunTimeInfo * crtInfo = (ChartRunTimeInfo *)(ssGetUserData(S));
    ChartInfoStruct * chartInfo = (ChartInfoStruct *)(crtInfo->instanceInfo);
    chartInstance = (SFc6_TwoDOF_PID_joint_spaceInstanceStruct *)
      chartInfo->chartInstance;
    if (ssIsFirstInitCond(S) && fullDebuggerInitialization==1) {
      /* do this only if simulation is starting */
      {
        unsigned int chartAlreadyPresent;
        chartAlreadyPresent = sf_debug_initialize_chart
          (sfGlobalDebugInstanceStruct,
           _TwoDOF_PID_joint_spaceMachineNumber_,
           6,
           1,
           1,
           0,
           4,
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
          _SFD_SET_DATA_PROPS(1,2,0,1,"invJ");
          _SFD_SET_DATA_PROPS(2,1,1,0,"V");
          _SFD_SET_DATA_PROPS(3,2,0,1,"QP");
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
        _SFD_CV_INIT_EML_FCN(0,0,"eML_blk_kernel",0,-1,222);

        {
          unsigned int dimVector[1];
          dimVector[0]= 2;
          _SFD_SET_DATA_COMPILED_PROPS(0,SF_DOUBLE,1,&(dimVector[0]),0,0,0,0.0,
            1.0,0,0,(MexFcnForType)c6_sf_marshallOut,(MexInFcnForType)NULL);
        }

        {
          unsigned int dimVector[2];
          dimVector[0]= 2;
          dimVector[1]= 2;
          _SFD_SET_DATA_COMPILED_PROPS(1,SF_DOUBLE,2,&(dimVector[0]),0,0,0,0.0,
            1.0,0,0,(MexFcnForType)c6_b_sf_marshallOut,(MexInFcnForType)
            c6_b_sf_marshallIn);
        }

        {
          unsigned int dimVector[1];
          dimVector[0]= 2;
          _SFD_SET_DATA_COMPILED_PROPS(2,SF_DOUBLE,1,&(dimVector[0]),0,0,0,0.0,
            1.0,0,0,(MexFcnForType)c6_sf_marshallOut,(MexInFcnForType)NULL);
        }

        {
          unsigned int dimVector[1];
          dimVector[0]= 2;
          _SFD_SET_DATA_COMPILED_PROPS(3,SF_DOUBLE,1,&(dimVector[0]),0,0,0,0.0,
            1.0,0,0,(MexFcnForType)c6_sf_marshallOut,(MexInFcnForType)
            c6_sf_marshallIn);
        }

        {
          real_T (*c6_Q)[2];
          real_T (*c6_invJ)[4];
          real_T (*c6_V)[2];
          real_T (*c6_QP)[2];
          c6_QP = (real_T (*)[2])ssGetOutputPortSignal(chartInstance->S, 2);
          c6_V = (real_T (*)[2])ssGetInputPortSignal(chartInstance->S, 1);
          c6_invJ = (real_T (*)[4])ssGetOutputPortSignal(chartInstance->S, 1);
          c6_Q = (real_T (*)[2])ssGetInputPortSignal(chartInstance->S, 0);
          _SFD_SET_DATA_VALUE_PTR(0U, *c6_Q);
          _SFD_SET_DATA_VALUE_PTR(1U, *c6_invJ);
          _SFD_SET_DATA_VALUE_PTR(2U, *c6_V);
          _SFD_SET_DATA_VALUE_PTR(3U, *c6_QP);
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
  return "7NxuqQBPPHTX2OtN5rGiM";
}

static void sf_opaque_initialize_c6_TwoDOF_PID_joint_space(void
  *chartInstanceVar)
{
  chart_debug_initialization(((SFc6_TwoDOF_PID_joint_spaceInstanceStruct*)
    chartInstanceVar)->S,0);
  initialize_params_c6_TwoDOF_PID_joint_space
    ((SFc6_TwoDOF_PID_joint_spaceInstanceStruct*) chartInstanceVar);
  initialize_c6_TwoDOF_PID_joint_space
    ((SFc6_TwoDOF_PID_joint_spaceInstanceStruct*) chartInstanceVar);
}

static void sf_opaque_enable_c6_TwoDOF_PID_joint_space(void *chartInstanceVar)
{
  enable_c6_TwoDOF_PID_joint_space((SFc6_TwoDOF_PID_joint_spaceInstanceStruct*)
    chartInstanceVar);
}

static void sf_opaque_disable_c6_TwoDOF_PID_joint_space(void *chartInstanceVar)
{
  disable_c6_TwoDOF_PID_joint_space((SFc6_TwoDOF_PID_joint_spaceInstanceStruct*)
    chartInstanceVar);
}

static void sf_opaque_gateway_c6_TwoDOF_PID_joint_space(void *chartInstanceVar)
{
  sf_gateway_c6_TwoDOF_PID_joint_space
    ((SFc6_TwoDOF_PID_joint_spaceInstanceStruct*) chartInstanceVar);
}

extern const mxArray* sf_internal_get_sim_state_c6_TwoDOF_PID_joint_space
  (SimStruct* S)
{
  ChartRunTimeInfo * crtInfo = (ChartRunTimeInfo *)(ssGetUserData(S));
  ChartInfoStruct * chartInfo = (ChartInfoStruct *)(crtInfo->instanceInfo);
  mxArray *plhs[1] = { NULL };

  mxArray *prhs[4];
  int mxError = 0;
  prhs[0] = mxCreateString("chart_simctx_raw2high");
  prhs[1] = mxCreateDoubleScalar(ssGetSFuncBlockHandle(S));
  prhs[2] = (mxArray*) get_sim_state_c6_TwoDOF_PID_joint_space
    ((SFc6_TwoDOF_PID_joint_spaceInstanceStruct*)chartInfo->chartInstance);/* raw sim ctx */
  prhs[3] = (mxArray*) sf_get_sim_state_info_c6_TwoDOF_PID_joint_space();/* state var info */
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

extern void sf_internal_set_sim_state_c6_TwoDOF_PID_joint_space(SimStruct* S,
  const mxArray *st)
{
  ChartRunTimeInfo * crtInfo = (ChartRunTimeInfo *)(ssGetUserData(S));
  ChartInfoStruct * chartInfo = (ChartInfoStruct *)(crtInfo->instanceInfo);
  mxArray *plhs[1] = { NULL };

  mxArray *prhs[3];
  int mxError = 0;
  prhs[0] = mxCreateString("chart_simctx_high2raw");
  prhs[1] = mxDuplicateArray(st);      /* high level simctx */
  prhs[2] = (mxArray*) sf_get_sim_state_info_c6_TwoDOF_PID_joint_space();/* state var info */
  mxError = sf_mex_call_matlab(1, plhs, 3, prhs, "sfprivate");
  mxDestroyArray(prhs[0]);
  mxDestroyArray(prhs[1]);
  mxDestroyArray(prhs[2]);
  if (mxError || plhs[0] == NULL) {
    sf_mex_error_message("Stateflow Internal Error: \nError calling 'chart_simctx_high2raw'.\n");
  }

  set_sim_state_c6_TwoDOF_PID_joint_space
    ((SFc6_TwoDOF_PID_joint_spaceInstanceStruct*)chartInfo->chartInstance,
     mxDuplicateArray(plhs[0]));
  mxDestroyArray(plhs[0]);
}

static const mxArray* sf_opaque_get_sim_state_c6_TwoDOF_PID_joint_space
  (SimStruct* S)
{
  return sf_internal_get_sim_state_c6_TwoDOF_PID_joint_space(S);
}

static void sf_opaque_set_sim_state_c6_TwoDOF_PID_joint_space(SimStruct* S,
  const mxArray *st)
{
  sf_internal_set_sim_state_c6_TwoDOF_PID_joint_space(S, st);
}

static void sf_opaque_terminate_c6_TwoDOF_PID_joint_space(void *chartInstanceVar)
{
  if (chartInstanceVar!=NULL) {
    SimStruct *S = ((SFc6_TwoDOF_PID_joint_spaceInstanceStruct*)
                    chartInstanceVar)->S;
    ChartRunTimeInfo * crtInfo = (ChartRunTimeInfo *)(ssGetUserData(S));
    if (sim_mode_is_rtw_gen(S) || sim_mode_is_external(S)) {
      sf_clear_rtw_identifier(S);
      unload_TwoDOF_PID_joint_space_optimization_info();
    }

    finalize_c6_TwoDOF_PID_joint_space
      ((SFc6_TwoDOF_PID_joint_spaceInstanceStruct*) chartInstanceVar);
    utFree((void *)chartInstanceVar);
    if (crtInfo != NULL) {
      utFree((void *)crtInfo);
    }

    ssSetUserData(S,NULL);
  }
}

static void sf_opaque_init_subchart_simstructs(void *chartInstanceVar)
{
  initSimStructsc6_TwoDOF_PID_joint_space
    ((SFc6_TwoDOF_PID_joint_spaceInstanceStruct*) chartInstanceVar);
}

extern unsigned int sf_machine_global_initializer_called(void);
static void mdlProcessParameters_c6_TwoDOF_PID_joint_space(SimStruct *S)
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
    initialize_params_c6_TwoDOF_PID_joint_space
      ((SFc6_TwoDOF_PID_joint_spaceInstanceStruct*)(chartInfo->chartInstance));
  }
}

static void mdlSetWorkWidths_c6_TwoDOF_PID_joint_space(SimStruct *S)
{
  if (sim_mode_is_rtw_gen(S) || sim_mode_is_external(S)) {
    mxArray *infoStruct = load_TwoDOF_PID_joint_space_optimization_info();
    int_T chartIsInlinable =
      (int_T)sf_is_chart_inlinable(sf_get_instance_specialization(),infoStruct,6);
    ssSetStateflowIsInlinable(S,chartIsInlinable);
    ssSetRTWCG(S,sf_rtw_info_uint_prop(sf_get_instance_specialization(),
                infoStruct,6,"RTWCG"));
    ssSetEnableFcnIsTrivial(S,1);
    ssSetDisableFcnIsTrivial(S,1);
    ssSetNotMultipleInlinable(S,sf_rtw_info_uint_prop
      (sf_get_instance_specialization(),infoStruct,6,
       "gatewayCannotBeInlinedMultipleTimes"));
    sf_update_buildInfo(sf_get_instance_specialization(),infoStruct,6);
    if (chartIsInlinable) {
      ssSetInputPortOptimOpts(S, 0, SS_REUSABLE_AND_LOCAL);
      ssSetInputPortOptimOpts(S, 1, SS_REUSABLE_AND_LOCAL);
      sf_mark_chart_expressionable_inputs(S,sf_get_instance_specialization(),
        infoStruct,6,2);
      sf_mark_chart_reusable_outputs(S,sf_get_instance_specialization(),
        infoStruct,6,2);
    }

    {
      unsigned int outPortIdx;
      for (outPortIdx=1; outPortIdx<=2; ++outPortIdx) {
        ssSetOutputPortOptimizeInIR(S, outPortIdx, 1U);
      }
    }

    {
      unsigned int inPortIdx;
      for (inPortIdx=0; inPortIdx < 2; ++inPortIdx) {
        ssSetInputPortOptimizeInIR(S, inPortIdx, 1U);
      }
    }

    sf_set_rtw_dwork_info(S,sf_get_instance_specialization(),infoStruct,6);
    ssSetHasSubFunctions(S,!(chartIsInlinable));
  } else {
  }

  ssSetOptions(S,ssGetOptions(S)|SS_OPTION_WORKS_WITH_CODE_REUSE);
  ssSetChecksum0(S,(1868378158U));
  ssSetChecksum1(S,(2208474697U));
  ssSetChecksum2(S,(3343394301U));
  ssSetChecksum3(S,(3649461868U));
  ssSetmdlDerivatives(S, NULL);
  ssSetExplicitFCSSCtrl(S,1);
  ssSupportsMultipleExecInstances(S,1);
}

static void mdlRTW_c6_TwoDOF_PID_joint_space(SimStruct *S)
{
  if (sim_mode_is_rtw_gen(S)) {
    ssWriteRTWStrParam(S, "StateflowChartType", "Embedded MATLAB");
  }
}

static void mdlStart_c6_TwoDOF_PID_joint_space(SimStruct *S)
{
  SFc6_TwoDOF_PID_joint_spaceInstanceStruct *chartInstance;
  ChartRunTimeInfo * crtInfo = (ChartRunTimeInfo *)utMalloc(sizeof
    (ChartRunTimeInfo));
  chartInstance = (SFc6_TwoDOF_PID_joint_spaceInstanceStruct *)utMalloc(sizeof
    (SFc6_TwoDOF_PID_joint_spaceInstanceStruct));
  memset(chartInstance, 0, sizeof(SFc6_TwoDOF_PID_joint_spaceInstanceStruct));
  if (chartInstance==NULL) {
    sf_mex_error_message("Could not allocate memory for chart instance.");
  }

  chartInstance->chartInfo.chartInstance = chartInstance;
  chartInstance->chartInfo.isEMLChart = 1;
  chartInstance->chartInfo.chartInitialized = 0;
  chartInstance->chartInfo.sFunctionGateway =
    sf_opaque_gateway_c6_TwoDOF_PID_joint_space;
  chartInstance->chartInfo.initializeChart =
    sf_opaque_initialize_c6_TwoDOF_PID_joint_space;
  chartInstance->chartInfo.terminateChart =
    sf_opaque_terminate_c6_TwoDOF_PID_joint_space;
  chartInstance->chartInfo.enableChart =
    sf_opaque_enable_c6_TwoDOF_PID_joint_space;
  chartInstance->chartInfo.disableChart =
    sf_opaque_disable_c6_TwoDOF_PID_joint_space;
  chartInstance->chartInfo.getSimState =
    sf_opaque_get_sim_state_c6_TwoDOF_PID_joint_space;
  chartInstance->chartInfo.setSimState =
    sf_opaque_set_sim_state_c6_TwoDOF_PID_joint_space;
  chartInstance->chartInfo.getSimStateInfo =
    sf_get_sim_state_info_c6_TwoDOF_PID_joint_space;
  chartInstance->chartInfo.zeroCrossings = NULL;
  chartInstance->chartInfo.outputs = NULL;
  chartInstance->chartInfo.derivatives = NULL;
  chartInstance->chartInfo.mdlRTW = mdlRTW_c6_TwoDOF_PID_joint_space;
  chartInstance->chartInfo.mdlStart = mdlStart_c6_TwoDOF_PID_joint_space;
  chartInstance->chartInfo.mdlSetWorkWidths =
    mdlSetWorkWidths_c6_TwoDOF_PID_joint_space;
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

void c6_TwoDOF_PID_joint_space_method_dispatcher(SimStruct *S, int_T method,
  void *data)
{
  switch (method) {
   case SS_CALL_MDL_START:
    mdlStart_c6_TwoDOF_PID_joint_space(S);
    break;

   case SS_CALL_MDL_SET_WORK_WIDTHS:
    mdlSetWorkWidths_c6_TwoDOF_PID_joint_space(S);
    break;

   case SS_CALL_MDL_PROCESS_PARAMETERS:
    mdlProcessParameters_c6_TwoDOF_PID_joint_space(S);
    break;

   default:
    /* Unhandled method */
    sf_mex_error_message("Stateflow Internal Error:\n"
                         "Error calling c6_TwoDOF_PID_joint_space_method_dispatcher.\n"
                         "Can't handle method %d.\n", method);
    break;
  }
}
