// Copyright 2021 DeepMind Technologies Limited
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef SIMCORE_SRC_ENGINE_ENGINE_UTIL_BLAS_H_
#define SIMCORE_SRC_ENGINE_ENGINE_UTIL_BLAS_H_

#include <math.h>

#include <simcore/SIM_export.h>
#include <simcore/SIM_tnum.h>

#ifdef __cplusplus
extern "C" {
#endif

//------------------------------ standard library functions ----------------------------------------

#if !defined(SIM_USESINGLE)
  #define sim_math_sqrt    sqrt
  #define sim_math_exp     exp
  #define sim_math_sin     sin
  #define sim_math_cos     cos
  #define sim_math_tan     tan
  #define sim_math_asin    asin
  #define sim_math_acos    acos
  #define sim_math_atan2   atan2
  #define sim_math_tanh    tanh
  #define sim_math_pow     pow
  #define sim_math_abs     fabs
  #define sim_math_log     log
  #define sim_math_log10   log10
  #define sim_math_floor   floor
  #define sim_math_ceil    ceil

#else
  #define sim_math_sqrt    sqrtf
  #define sim_math_exp     expf
  #define sim_math_sin     sinf
  #define sim_math_cos     cosf
  #define sim_math_tan     tanf
  #define sim_math_asin    asinf
  #define sim_math_acos    acosf
  #define sim_math_atan2   atan2f
  #define sim_math_tanh    tanhf
  #define sim_math_pow     powf
  #define sim_math_abs     fabsf
  #define sim_math_log     logf
  #define sim_math_log10   log10f
  #define sim_math_floor   floorf
  #define sim_math_ceil    ceilf
#endif  // !defined(SIM_USESINGLE)


//------------------------------ 3D vector and matrix-vector operations ----------------------------

// res = 0
SIM_API void sim_math_zero_3(sim_scalar_t res[3]);

// vec1 == vec2
SIM_API int sim_math_equal3(const sim_scalar_t vec1[3], const sim_scalar_t vec2[3]);

// res = vec
SIM_API void sim_math_copy_3(sim_scalar_t res[3], const sim_scalar_t vec[3]);

// res = mat
void sim_math_copy9(sim_scalar_t res[9], const sim_scalar_t mat[9]);

// res = vec*scl
SIM_API void sim_math_scale_3(sim_scalar_t res[3], const sim_scalar_t vec[3], sim_scalar_t scl);

// res = vec1 + vec2
SIM_API void sim_math_add3(sim_scalar_t res[3], const sim_scalar_t vec1[3], const sim_scalar_t vec2[3]);

// res = vec1 - vec2
SIM_API void sim_math_sub_3(sim_scalar_t res[3], const sim_scalar_t vec1[3], const sim_scalar_t vec2[3]);

// res += vec
SIM_API void sim_math_add_to_3(sim_scalar_t res[3], const sim_scalar_t vec[3]);

// res -= vec
SIM_API void sim_math_subFrom3(sim_scalar_t res[3], const sim_scalar_t vec[3]);

// res += vec*scl
SIM_API void sim_math_add_to_scale_3(sim_scalar_t res[3], const sim_scalar_t vec[3], sim_scalar_t scl);

// res = vec1 + vec2*scl
SIM_API void sim_math_addScl3(sim_scalar_t res[3], const sim_scalar_t vec1[3], const sim_scalar_t vec2[3], sim_scalar_t scl);

// normalize vector, return length before normalization, set to [1, 0, 0] if norm is tiny
SIM_API sim_scalar_t sim_math_normalize_3(sim_scalar_t vec[3]);

// compute vector length (without normalizing)
SIM_API sim_scalar_t sim_math_norm3(const sim_scalar_t vec[3]);

// vector dot-product
SIM_API sim_scalar_t sim_math_dot_3(const sim_scalar_t vec1[3], const sim_scalar_t vec2[3]);

// Cartesian distance between 3D vectors
SIM_API sim_scalar_t sim_math_dist3(const sim_scalar_t pos1[3], const sim_scalar_t pos2[3]);

// multiply 3-by-3 matrix by vector
SIM_API void sim_math_mul_mat_vec_3(sim_scalar_t res[3], const sim_scalar_t mat[9], const sim_scalar_t vec[3]);

// multiply transposed 3-by-3 matrix by vector
SIM_API void sim_math_mulMatTVec3(sim_scalar_t res[3], const sim_scalar_t mat[9], const sim_scalar_t vec[3]);

// multiply 3x3 matrices
SIM_API void sim_math_mulMatMat3(sim_scalar_t res[9], const sim_scalar_t mat1[9], const sim_scalar_t mat2[9]);

// multiply 3x3 matrices, first argument transposed
SIM_API void sim_math_mulMatTMat3(sim_scalar_t res[9], const sim_scalar_t mat1[9], const sim_scalar_t mat2[9]);

// multiply 3x3 matrices, second argument transposed
SIM_API void sim_math_mulMatMatT3(sim_scalar_t res[9], const sim_scalar_t mat1[9], const sim_scalar_t mat2[9]);

//------------------------------ 4D/quaternion operations ------------------------------------------

// res = 0
SIM_API void sim_math_zero4(sim_scalar_t res[4]);

// res = (1,0,0,0)
SIM_API void sim_math_unit4(sim_scalar_t res[4]);

// res = vec
SIM_API void sim_math_copy4(sim_scalar_t res[4], const sim_scalar_t data[4]);

// normalize vector, return length before normalization
SIM_API sim_scalar_t sim_math_normalize4(sim_scalar_t vec[4]);


//------------------------------ general vector operations -----------------------------------------

// res = 0
SIM_API void sim_math_zero(sim_scalar_t* res, int n);

// res = 0, at given indices
void sim_math_zeroInd(sim_scalar_t* res, int n, const int* ind);

// res = val
SIM_API void sim_math_fill(sim_scalar_t* res, sim_scalar_t val, int n);

// res = vec
SIM_API void sim_math_copy(sim_scalar_t* res, const sim_scalar_t* vec, int n);

// res = vec, at given indices
void sim_math_copyInd(sim_scalar_t* res, const sim_scalar_t* vec, const int* ind, int n);

// sum(vec)
SIM_API sim_scalar_t sim_math_sum(const sim_scalar_t* vec, int n);

// sum(abs(vec))
SIM_API sim_scalar_t sim_math_L1(const sim_scalar_t* vec, int n);

// res = vec*scl
SIM_API void sim_math_scl(sim_scalar_t* res, const sim_scalar_t* vec, sim_scalar_t scl, int n);

// res = vec1 + vec2
SIM_API void sim_math_add(sim_scalar_t* res, const sim_scalar_t* vec1, const sim_scalar_t* vec2, int n);

// res = vec1 + vec2, at given indices
void sim_math_addInd(sim_scalar_t* res, const sim_scalar_t* vec1, const sim_scalar_t* vec2, const int* ind, int n);

// res = vec1 - vec2
SIM_API void sim_math_sub(sim_scalar_t* res, const sim_scalar_t* vec1, const sim_scalar_t* vec2, int n);

// res = vec1 - vec2, at selected indices
void sim_math_subInd(sim_scalar_t* res, const sim_scalar_t* vec1, const sim_scalar_t* vec2, const int* ind, int n);

// res += vec
SIM_API void sim_math_addTo(sim_scalar_t* res, const sim_scalar_t* vec, int n);

// res += vec, at selected indices
void sim_math_addToInd(sim_scalar_t* res, const sim_scalar_t* vec, const int* ind, int n);

// res -= vec
SIM_API void sim_math_subFrom(sim_scalar_t* res, const sim_scalar_t* vec, int n);

// res += vec*scl
SIM_API void sim_math_addToScl(sim_scalar_t* res, const sim_scalar_t* vec, sim_scalar_t scl, int n);

// res += vec*scl, at given indices
void sim_math_addToSclInd(sim_scalar_t* res, const sim_scalar_t* vec, const int* ind, sim_scalar_t scl, int n);

// res = vec1 + vec2*scl
SIM_API void sim_math_addScl(sim_scalar_t* res, const sim_scalar_t* vec1, const sim_scalar_t* vec2, sim_scalar_t scl, int n);

// normalize vector, return length before normalization
SIM_API sim_scalar_t sim_math_normalize(sim_scalar_t* res, int n);

// compute vector length (without normalizing)
SIM_API sim_scalar_t sim_math_norm(const sim_scalar_t* res, int n);

// vector dot-product
SIM_API sim_scalar_t sim_math_dot(const sim_scalar_t* vec1, const sim_scalar_t* vec2, int n);

// vector dot-product, at given indices
sim_scalar_t sim_math_dotInd(const sim_scalar_t* vec1, const sim_scalar_t* vec2, const int* ind, int n);

//------------------------------ matrix-vector operations ------------------------------------------

// multiply matrix and vector
SIM_API void sim_math_mulMatVec(sim_scalar_t* res, const sim_scalar_t* mat, const sim_scalar_t* vec, int nr, int nc);

// multiply transposed matrix and vector
SIM_API void sim_math_mulMatTVec(sim_scalar_t* res, const sim_scalar_t* mat, const sim_scalar_t* vec, int nr, int nc);

// multiply square matrix with vectors on both sides: return vec1'*mat*vec2
SIM_API sim_scalar_t sim_math_mulVecMatVec(const sim_scalar_t* vec1, const sim_scalar_t* mat, const sim_scalar_t* vec2, int n);


//------------------------------ matrix operations -------------------------------------------------

// transpose matrix
SIM_API void sim_math_transpose(sim_scalar_t* res, const sim_scalar_t* mat, int nr, int nc);

// symmetrize square matrix res = (mat + mat')/2
SIM_API void sim_math_symmetrize(sim_scalar_t* res, const sim_scalar_t* mat, int n);

// identity matrix
SIM_API void sim_math_eye(sim_scalar_t* mat, int n);

// copy selected rows:  res[ind, :] = mat[ind, :]
void sim_math_copyRows(sim_scalar_t* res, const sim_scalar_t* mat, const int* ind, int n, int nc);

//------------------------------ matrix-matrix operations ------------------------------------------

// multiply matrices
SIM_API void sim_math_mulMatMat(sim_scalar_t* res, const sim_scalar_t* mat1, const sim_scalar_t* mat2,
                         int r1, int c1, int c2);

// multiply matrices, second argument transposed
SIM_API void sim_math_mulMatMatT(sim_scalar_t* res, const sim_scalar_t* mat1, const sim_scalar_t* mat2,
                          int r1, int c1, int r2);

// multiply matrices, first argument transposed
SIM_API void sim_math_mulMatTMat(sim_scalar_t* res, const sim_scalar_t* mat1, const sim_scalar_t* mat2,
                          int r1, int c1, int c2);

// compute M'*diag*M (diag=NULL: compute M'*M), upper triangle optional
void sim_math_sqrMatTD_impl(sim_scalar_t* res, const sim_scalar_t* mat, const sim_scalar_t* diag, int nr, int nc,
                       int flg_upper);

// compute M'*diag*M (diag=NULL: compute M'*M)
SIM_API void sim_math_sqrMatTD(sim_scalar_t* res, const sim_scalar_t* mat, const sim_scalar_t* diag, int nr, int nc);

#ifdef __cplusplus
}
#endif
#endif  // SIMCORE_SRC_ENGINE_ENGINE_UTIL_BLAS_H_
