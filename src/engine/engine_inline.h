// Copyright 2025 DeepMind Technologies Limited
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

#ifndef SIMCORE_SRC_ENGINE_ENGINE_INLINE_H_
#define SIMCORE_SRC_ENGINE_ENGINE_INLINE_H_

#include <simcore/SIM_tnum.h>
#include <simcore/core_api.h>

#ifdef __cplusplus
  #define restrict __restrict__
  extern "C" {
#endif

/* =================================================================================================

Private, high efficiency inlined functions for internal engine use.
Inlining on its own does nothing due to LTO, so criteria for SIM_i_ functions are:
- More efficient assembly output due to `restrict`.
- Skipped copies due to non-alias guarantee (e.g., compare sim_math_cross and sim_math_internal_cross).

SIM_i_ functions:
- Should be modified as little as possible from their SIM_u_ counterparts.
- Should be used only on the hotpath to prevent future bugs due to aliasing.

================================================================================================= */


//------------------------------ 3D vector and matrix-vector operations ----------------------------

// res = vec
static inline
void sim_math_internal_copy_3(sim_scalar_t* restrict res, const sim_scalar_t *vec) {
  res[0] = vec[0];
  res[1] = vec[1];
  res[2] = vec[2];
}


// res = vec*scl
static inline
void sim_math_internal_scl3(sim_scalar_t* restrict res, const sim_scalar_t vec[3], sim_scalar_t scl) {
  res[0] = vec[0] * scl;
  res[1] = vec[1] * scl;
  res[2] = vec[2] * scl;
}


// res = vec1 + vec2
static inline
void sim_math_internal_add3(sim_scalar_t* restrict res, const sim_scalar_t vec1[3], const sim_scalar_t vec2[3]) {
  res[0] = vec1[0] + vec2[0];
  res[1] = vec1[1] + vec2[1];
  res[2] = vec1[2] + vec2[2];
}


// res = vec1 - vec2
static inline
void sim_math_internal_sub3(sim_scalar_t* restrict res, const sim_scalar_t vec1[3], const sim_scalar_t vec2[3]) {
  res[0] = vec1[0] - vec2[0];
  res[1] = vec1[1] - vec2[1];
  res[2] = vec1[2] - vec2[2];
}


// res += vec
static inline
void sim_math_internal_addTo3(sim_scalar_t* restrict res, const sim_scalar_t vec[3]) {
  res[0] += vec[0];
  res[1] += vec[1];
  res[2] += vec[2];
}


// res -= vec
static inline
void sim_math_internal_subFrom3(sim_scalar_t* restrict res, const sim_scalar_t vec[3]) {
  res[0] -= vec[0];
  res[1] -= vec[1];
  res[2] -= vec[2];
}


// res += vec*scl
static inline
void sim_math_internal_addToScl3(sim_scalar_t* restrict res, const sim_scalar_t vec[3], sim_scalar_t scl) {
  res[0] += vec[0] * scl;
  res[1] += vec[1] * scl;
  res[2] += vec[2] * scl;
}


// res = vec1 + vec2*scl
static inline
void sim_math_internal_addScl3(sim_scalar_t* restrict res, const sim_scalar_t* vec1, const sim_scalar_t* vec2, sim_scalar_t scl) {
  res[0] = vec1[0] + scl*vec2[0];
  res[1] = vec1[1] + scl*vec2[1];
  res[2] = vec1[2] + scl*vec2[2];
}


// normalize vector, return length before normalization
// (for use in this file only)
static inline
sim_scalar_t sim_math_internal__normalize3(sim_scalar_t vec[3]) {
  sim_scalar_t norm = sim_math_sqrt(vec[0]*vec[0] + vec[1]*vec[1] + vec[2]*vec[2]);

  if (norm < SIM_MINVAL) {
    vec[0] = 1;
    vec[1] = 0;
    vec[2] = 0;
  } else {
    sim_scalar_t normInv = 1/norm;
    vec[0] *= normInv;
    vec[1] *= normInv;
    vec[2] *= normInv;
  }

  return norm;
}


// multiply vector by 3D rotation matrix
static inline
void sim_math_internal_mulMatVec3(sim_scalar_t* restrict res, const sim_scalar_t mat[9], const sim_scalar_t vec[3]) {
  res[0] = mat[0]*vec[0] + mat[1]*vec[1] + mat[2]*vec[2];
  res[1] = mat[3]*vec[0] + mat[4]*vec[1] + mat[5]*vec[2];
  res[2] = mat[6]*vec[0] + mat[7]*vec[1] + mat[8]*vec[2];
}


// multiply vector by transposed 3D rotation matrix
static inline
void sim_math_internal_mulMatTVec3(sim_scalar_t* restrict res, const sim_scalar_t mat[9], const sim_scalar_t vec[3]) {
  res[0] = mat[0]*vec[0] + mat[3]*vec[1] + mat[6]*vec[2];
  res[1] = mat[1]*vec[0] + mat[4]*vec[1] + mat[7]*vec[2];
  res[2] = mat[2]*vec[0] + mat[5]*vec[1] + mat[8]*vec[2];
}


// multiply 3x3 matrices,
static inline
void sim_math_internal_mulMatMat3(sim_scalar_t* restrict res, const sim_scalar_t mat1[9], const sim_scalar_t mat2[9]) {
  res[0] = mat1[0]*mat2[0] + mat1[1]*mat2[3] + mat1[2]*mat2[6];
  res[1] = mat1[0]*mat2[1] + mat1[1]*mat2[4] + mat1[2]*mat2[7];
  res[2] = mat1[0]*mat2[2] + mat1[1]*mat2[5] + mat1[2]*mat2[8];
  res[3] = mat1[3]*mat2[0] + mat1[4]*mat2[3] + mat1[5]*mat2[6];
  res[4] = mat1[3]*mat2[1] + mat1[4]*mat2[4] + mat1[5]*mat2[7];
  res[5] = mat1[3]*mat2[2] + mat1[4]*mat2[5] + mat1[5]*mat2[8];
  res[6] = mat1[6]*mat2[0] + mat1[7]*mat2[3] + mat1[8]*mat2[6];
  res[7] = mat1[6]*mat2[1] + mat1[7]*mat2[4] + mat1[8]*mat2[7];
  res[8] = mat1[6]*mat2[2] + mat1[7]*mat2[5] + mat1[8]*mat2[8];
}


// multiply 3x3 matrices, first argument transposed
static inline
void sim_math_internal_mulMatTMat3(sim_scalar_t* restrict res, const sim_scalar_t mat1[9], const sim_scalar_t mat2[9]) {
  res[0] = mat1[0]*mat2[0] + mat1[3]*mat2[3] + mat1[6]*mat2[6];
  res[1] = mat1[0]*mat2[1] + mat1[3]*mat2[4] + mat1[6]*mat2[7];
  res[2] = mat1[0]*mat2[2] + mat1[3]*mat2[5] + mat1[6]*mat2[8];
  res[3] = mat1[1]*mat2[0] + mat1[4]*mat2[3] + mat1[7]*mat2[6];
  res[4] = mat1[1]*mat2[1] + mat1[4]*mat2[4] + mat1[7]*mat2[7];
  res[5] = mat1[1]*mat2[2] + mat1[4]*mat2[5] + mat1[7]*mat2[8];
  res[6] = mat1[2]*mat2[0] + mat1[5]*mat2[3] + mat1[8]*mat2[6];
  res[7] = mat1[2]*mat2[1] + mat1[5]*mat2[4] + mat1[8]*mat2[7];
  res[8] = mat1[2]*mat2[2] + mat1[5]*mat2[5] + mat1[8]*mat2[8];
}


//------------------------------ 4D vector and matrix-vector operations ----------------------------

// res = vec
static inline
void sim_math_internal_copy4(sim_scalar_t* restrict res, const sim_scalar_t data[4]) {
  res[0] = data[0];
  res[1] = data[1];
  res[2] = data[2];
  res[3] = data[3];
}


// normalize vector, return length before normalization
// (for use in this file only)
static inline
sim_scalar_t sim_math_internal__normalize4(sim_scalar_t vec[4]) {
  sim_scalar_t norm = sim_math_sqrt(vec[0]*vec[0] + vec[1]*vec[1] + vec[2]*vec[2] + vec[3]*vec[3]);

  if (norm < SIM_MINVAL) {
    vec[0] = 1;
    vec[1] = 0;
    vec[2] = 0;
    vec[3] = 0;
  } else if (sim_math_abs(norm - 1) > SIM_MINVAL) {
    sim_scalar_t normInv = 1/norm;
    vec[0] *= normInv;
    vec[1] *= normInv;
    vec[2] *= normInv;
    vec[3] *= normInv;
  }

  return norm;
}


//------------------------------ quaternion operations ---------------------------------------------

// rotate vector by quaternion
static inline
void sim_math_internal_rotVecQuat(sim_scalar_t* restrict res, const sim_scalar_t vec[3], const sim_scalar_t quat[4]) {
  // null quat: copy vec
  if (quat[0] == 1 && quat[1] == 0 && quat[2] == 0 && quat[3] == 0) {
    sim_math_internal_copy_3(res, vec);
  }

  // regular processing
  else {
    sim_scalar_t tmp[3];
    // tmp = q_w * v + cross(q_xyz, v)
    tmp[0] = quat[0]*vec[0] + quat[2]*vec[2] - quat[3]*vec[1];
    tmp[1] = quat[0]*vec[1] + quat[3]*vec[0] - quat[1]*vec[2];
    tmp[2] = quat[0]*vec[2] + quat[1]*vec[1] - quat[2]*vec[0];
    // res = v + 2 * cross(q_xyz, t)
    res[0] = vec[0] + 2 * (quat[2]*tmp[2] - quat[3]*tmp[1]);
    res[1] = vec[1] + 2 * (quat[3]*tmp[0] - quat[1]*tmp[2]);
    res[2] = vec[2] + 2 * (quat[1]*tmp[1] - quat[2]*tmp[0]);
  }
}


// negate quaternion
static inline
void sim_math_internal_negQuat(sim_scalar_t* restrict res, const sim_scalar_t quat[4]) {
  res[0] = quat[0];
  res[1] = -quat[1];
  res[2] = -quat[2];
  res[3] = -quat[3];
}


// multiply quaternions
static inline
void sim_math_internal_mulQuat(sim_scalar_t* restrict res, const sim_scalar_t qa[4], const sim_scalar_t qb[4]) {
  res[0] = qa[0]*qb[0] - qa[1]*qb[1] - qa[2]*qb[2] - qa[3]*qb[3];
  res[1] = qa[0]*qb[1] + qa[1]*qb[0] + qa[2]*qb[3] - qa[3]*qb[2];
  res[2] = qa[0]*qb[2] - qa[1]*qb[3] + qa[2]*qb[0] + qa[3]*qb[1];
  res[3] = qa[0]*qb[3] + qa[1]*qb[2] - qa[2]*qb[1] + qa[3]*qb[0];
}


// multiply quaternion and axis
static inline
void sim_math_internal_mulQuatAxis(sim_scalar_t* restrict res, const sim_scalar_t quat[4], const sim_scalar_t axis[3]) {
  res[0] = -quat[1]*axis[0] - quat[2]*axis[1] - quat[3]*axis[2];
  res[1] =  quat[0]*axis[0] + quat[2]*axis[2] - quat[3]*axis[1];
  res[2] =  quat[0]*axis[1] + quat[3]*axis[0] - quat[1]*axis[2];
  res[3] =  quat[0]*axis[2] + quat[1]*axis[1] - quat[2]*axis[0];
}


// convert axisAngle to quaternion
static inline
void sim_math_internal_axisAngle2Quat(sim_scalar_t* restrict res, const sim_scalar_t axis[3], sim_scalar_t angle) {
  // zero angle: null quat
  if (angle == 0) {
    res[0] = 1;
    res[1] = 0;
    res[2] = 0;
    res[3] = 0;
  }

  // regular processing
  else {
    sim_scalar_t s = sim_math_sin(angle*0.5);
    res[0] = sim_math_cos(angle*0.5);
    res[1] = axis[0]*s;
    res[2] = axis[1]*s;
    res[3] = axis[2]*s;
  }
}


// convert quaternion (corresponding to orientation difference) to 3D velocity
static inline
void sim_math_internal_quat2Vel(sim_scalar_t* restrict res, const sim_scalar_t quat[4], sim_scalar_t dt) {
  sim_scalar_t axis[3] = {quat[1], quat[2], quat[3]};
  sim_scalar_t sin_a_2 = sim_math_internal__normalize3(axis);
  sim_scalar_t speed = 2 * sim_math_atan2(sin_a_2, quat[0]);

  // when axis-angle is larger than pi, rotation is in the opposite direction
  if (speed > SIM_PI) {
    speed -= 2*SIM_PI;
  }
  speed /= dt;

  sim_math_internal_scl3(res, axis, speed);
}


// Subtract quaternions, express as 3D velocity: qb*quat(res) = qa.
static inline
void sim_math_internal_subQuat(sim_scalar_t* restrict res, const sim_scalar_t qa[4], const sim_scalar_t qb[4]) {
  // qdif = neg(qb)*qa
  sim_scalar_t qneg[4], qdif[4];
  sim_math_internal_negQuat(qneg, qb);
  sim_math_internal_mulQuat(qdif, qneg, qa);

  // convert to 3D velocity
  sim_math_internal_quat2Vel(res, qdif, 1);
}


// convert 3D rotation matrix to quaternion
static inline
void sim_math_internal_mat2Quat(sim_scalar_t* restrict quat, const sim_scalar_t mat[9]) {
  // q0 largest
  if (mat[0]+mat[4]+mat[8]>0) {
    quat[0] = 0.5 * sim_math_sqrt(1 + mat[0] + mat[4] + mat[8]);
    quat[1] = 0.25 * (mat[7] - mat[5]) / quat[0];
    quat[2] = 0.25 * (mat[2] - mat[6]) / quat[0];
    quat[3] = 0.25 * (mat[3] - mat[1]) / quat[0];
  }

  // q1 largest
  else if (mat[0]>mat[4] && mat[0]>mat[8]) {
    quat[1] = 0.5 * sim_math_sqrt(1 + mat[0] - mat[4] - mat[8]);
    quat[0] = 0.25 * (mat[7] - mat[5]) / quat[1];
    quat[2] = 0.25 * (mat[1] + mat[3]) / quat[1];
    quat[3] = 0.25 * (mat[2] + mat[6]) / quat[1];
  }

  // q2 largest
  else if (mat[4]>mat[8]) {
    quat[2] = 0.5 * sim_math_sqrt(1 - mat[0] + mat[4] - mat[8]);
    quat[0] = 0.25 * (mat[2] - mat[6]) / quat[2];
    quat[1] = 0.25 * (mat[1] + mat[3]) / quat[2];
    quat[3] = 0.25 * (mat[5] + mat[7]) / quat[2];
  }

  // q3 largest
  else {
    quat[3] = 0.5 * sim_math_sqrt(1 - mat[0] - mat[4] + mat[8]);
    quat[0] = 0.25 * (mat[3] - mat[1]) / quat[3];
    quat[1] = 0.25 * (mat[2] + mat[6]) / quat[3];
    quat[2] = 0.25 * (mat[5] + mat[7]) / quat[3];
  }

  sim_math_internal__normalize4(quat);
}


// integrate quaternion given 3D angular velocity
static inline
void sim_math_internal_quatIntegrate(sim_scalar_t* restrict quat, const sim_scalar_t vel[3], sim_scalar_t scale) {
  sim_scalar_t angle, tmp[4], qrot[4];

  // form local rotation quaternion, apply
  sim_math_internal_copy_3(tmp, vel);
  angle = scale * sim_math_internal__normalize3(tmp);
  sim_math_internal_axisAngle2Quat(qrot, tmp, angle);
  sim_math_internal__normalize4(quat);
  sim_math_internal_copy4(tmp, quat);
  sim_math_internal_mulQuat(quat, tmp, qrot);
}


//------------------------------ spatial algebra ---------------------------------------------------

// vector cross-product, 3D
static inline
void sim_math_internal_cross(sim_scalar_t* restrict res, const sim_scalar_t a[3], const sim_scalar_t b[3]) {
  res[0] = a[1]*b[2] - a[2]*b[1];
  res[1] = a[2]*b[0] - a[0]*b[2];
  res[2] = a[0]*b[1] - a[1]*b[0];
}


// cross-product for motion vector
static inline
void sim_math_internal_crossMotion(sim_scalar_t* restrict res, const sim_scalar_t vel[6], const sim_scalar_t v[6]) {
  res[0] = -vel[2]*v[1] + vel[1]*v[2];
  res[1] =  vel[2]*v[0] - vel[0]*v[2];
  res[2] = -vel[1]*v[0] + vel[0]*v[1];
  res[3] = -vel[2]*v[4] + vel[1]*v[5];
  res[4] =  vel[2]*v[3] - vel[0]*v[5];
  res[5] = -vel[1]*v[3] + vel[0]*v[4];

  res[3] += -vel[5]*v[1] + vel[4]*v[2];
  res[4] +=  vel[5]*v[0] - vel[3]*v[2];
  res[5] += -vel[4]*v[0] + vel[3]*v[1];
}


// cross-product for force vectors
static inline
void sim_math_internal_crossForce(sim_scalar_t* restrict res, const sim_scalar_t vel[6], const sim_scalar_t f[6]) {
  res[0] = -vel[2]*f[1] + vel[1]*f[2];
  res[1] =  vel[2]*f[0] - vel[0]*f[2];
  res[2] = -vel[1]*f[0] + vel[0]*f[1];
  res[3] = -vel[2]*f[4] + vel[1]*f[5];
  res[4] =  vel[2]*f[3] - vel[0]*f[5];
  res[5] = -vel[1]*f[3] + vel[0]*f[4];

  res[0] += -vel[5]*f[4] + vel[4]*f[5];
  res[1] +=  vel[5]*f[3] - vel[3]*f[5];
  res[2] += -vel[4]*f[3] + vel[3]*f[4];
}


// 6D vector dot-product
static inline
sim_scalar_t sim_math_internal_dot6(const sim_scalar_t vec1[6], const sim_scalar_t vec2[6]) {
  // match order of operations to sim_math_dot
  return ((vec1[0] * vec2[0] + vec1[2] * vec2[2]) +
          (vec1[1] * vec2[1] + vec1[3] * vec2[3])) +
          (vec1[4] * vec2[4] + vec1[5] * vec2[5]);
}


// res = vec
static inline
void sim_math_internal_copy6(sim_scalar_t* restrict res, const sim_scalar_t *vec) {
  res[0] = vec[0];
  res[1] = vec[1];
  res[2] = vec[2];
  res[3] = vec[3];
  res[4] = vec[4];
  res[5] = vec[5];
}


// res = vec
static inline
void sim_math_internal_copy9(sim_scalar_t* restrict res, const sim_scalar_t data[9]) {
  res[0] = data[0];
  res[1] = data[1];
  res[2] = data[2];
  res[3] = data[3];
  res[4] = data[4];
  res[5] = data[5];
  res[6] = data[6];
  res[7] = data[7];
  res[8] = data[8];
}


#ifdef __cplusplus
  }  // extern "C"
  #undef restrict
#endif

#endif  // SIMCORE_SRC_ENGINE_ENGINE_INLINE_H_
