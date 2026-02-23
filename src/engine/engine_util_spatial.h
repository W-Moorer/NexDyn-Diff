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

#ifndef SIMCORE_SRC_ENGINE_ENGINE_UTIL_SPATIAL_H_
#define SIMCORE_SRC_ENGINE_ENGINE_UTIL_SPATIAL_H_

#include <simcore/SIM_export.h>
#include <simcore/SIM_tnum.h>

#ifdef __cplusplus
extern "C" {
#endif
//------------------------------ quaternion operations ---------------------------------------------

// rotate vector by quaternion
SIM_API void sim_math_rotVecQuat(sim_scalar_t res[3], const sim_scalar_t vec[3], const sim_scalar_t quat[4]);

// compute conjugate quaternion, corresponding to opposite rotation
SIM_API void sim_math_negQuat(sim_scalar_t res[4], const sim_scalar_t quat[4]);

// multiply quaternions
SIM_API void sim_math_mulQuat(sim_scalar_t res[4], const sim_scalar_t quat1[4], const sim_scalar_t quat2[4]);

// multiply quaternion and axis
SIM_API void sim_math_mulQuatAxis(sim_scalar_t res[4], const sim_scalar_t quat[4], const sim_scalar_t axis[3]);

// convert axisAngle to quaternion
SIM_API void sim_math_axisAngle2Quat(sim_scalar_t res[4], const sim_scalar_t axis[3], sim_scalar_t angle);

// convert quaternion (corresponding to orientation difference) to 3D velocity
SIM_API void sim_math_quat2Vel(sim_scalar_t res[3], const sim_scalar_t quat[4], sim_scalar_t dt);

// subtract quaternions, convert to 3D velocity: qb*quat(res) = qa
SIM_API void sim_math_subQuat(sim_scalar_t res[3], const sim_scalar_t qa[4], const sim_scalar_t qb[4]);

// convert quaternion to 3D rotation matrix
SIM_API void sim_math_quat2Mat(sim_scalar_t res[9], const sim_scalar_t quat[4]);

// convert 3D rotation matrix to quaternion
SIM_API void sim_math_mat2Quat(sim_scalar_t quat[4], const sim_scalar_t mat[9]);

// time-derivative of quaternion, given 3D rotational velocity
SIM_API void sim_math_derivQuat(sim_scalar_t res[4], const sim_scalar_t quat[4], const sim_scalar_t vel[3]);

// integrate quaternion given 3D angular velocity
SIM_API void sim_math_quatIntegrate(sim_scalar_t quat[4], const sim_scalar_t vel[3], sim_scalar_t scale);

// compute quaternion performing rotation from z-axis to given vector
SIM_API void sim_math_quatZ2Vec(sim_scalar_t quat[4], const sim_scalar_t vec[3]);

// extract 3D rotation from an arbitrary 3x3 matrix by refining the input quaternion
// returns the number of iterations required to converge
SIM_API int sim_math_mat2Rot(sim_scalar_t quat[4], const sim_scalar_t mat[9]);


//------------------------------ pose operations (pos, quat) ---------------------------------------

// multiply two poses
SIM_API void sim_math_mulPose(sim_scalar_t posres[3], sim_scalar_t quatres[4],
                       const sim_scalar_t pos1[3], const sim_scalar_t quat1[4],
                       const sim_scalar_t pos2[3], const sim_scalar_t quat2[4]);

// compute conjugate pose, corresponding to the opposite spatial transformation
SIM_API void sim_math_negPose(sim_scalar_t posres[3], sim_scalar_t quatres[4],
                       const sim_scalar_t pos[3], const sim_scalar_t quat[4]);

// transform vector by pose
SIM_API void sim_math_trnVecPose(sim_scalar_t res[3], const sim_scalar_t pos[3], const sim_scalar_t quat[4],
                          const sim_scalar_t vec[3]);

// convert sequence of Euler angles (radians) to quaternion
// seq[0,1,2] must be in 'xyzXYZ', lower/upper-case mean intrinsic/extrinsic rotations
SIM_API void sim_math_euler2Quat(sim_scalar_t quat[4], const sim_scalar_t euler[3], const char* seq);

//------------------------------ spatial algebra ---------------------------------------------------

// vector cross-product, 3D
SIM_API void sim_math_cross(sim_scalar_t res[3], const sim_scalar_t a[3], const sim_scalar_t b[3]);

// cross-product for motion vector
void sim_math_crossMotion(sim_scalar_t res[6], const sim_scalar_t vel[6], const sim_scalar_t v[6]);

// cross-product for force vectors
void sim_math_crossForce(sim_scalar_t res[6], const sim_scalar_t vel[6], const sim_scalar_t f[6]);

// express inertia in com-based frame
void sim_math_inertCom(sim_scalar_t* res, const sim_scalar_t inert[3], const sim_scalar_t mat[9],
                  const sim_scalar_t dif[3], sim_scalar_t mass);

// express motion axis in com-based frame
void sim_math_dofCom(sim_scalar_t* res, const sim_scalar_t axis[3], const sim_scalar_t offset[3]);

// multiply 6D vector (rotation, translation) by 6D inertia matrix
SIM_API void sim_math_mulInertVec(sim_scalar_t* res, const sim_scalar_t inert[10], const sim_scalar_t vec[6]);

// multiply dof matrix by vector
void sim_math_mulDofVec(sim_scalar_t* res, const sim_scalar_t* mat, const sim_scalar_t* vec, int n);

// coordinate transform of 6D motion or force vector in rotation:translation format
//  rotnew2old is 3-by-3, NULL means no rotation; flg_force specifies force or motion type
SIM_API void sim_math_transformSpatial(sim_scalar_t res[6], const sim_scalar_t vec[6], int flg_force,
                                const sim_scalar_t newpos[3], const sim_scalar_t oldpos[3],
                                const sim_scalar_t rotnew2old[9]);

// make 3D frame given X axis (and possibly Y axis)
void sim_math_makeFrame(sim_scalar_t frame[9]);

#ifdef __cplusplus
}
#endif
#endif  // SIMCORE_SRC_ENGINE_ENGINE_UTIL_SPATIAL_H_
