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

#include "engine/engine_util_spatial.h"

#include <math.h>

#include <simcore/SIM_model.h>
#include "engine/engine_inline.h"
#include "engine/engine_util_blas.h"
#include "engine/engine_util_errmem.h"


//------------------------------ quaternion operations ---------------------------------------------

// rotate vector by quaternion
void sim_math_rotVecQuat(sim_scalar_t res[3], const sim_scalar_t vec[3], const sim_scalar_t quat[4]) {
  // zero vec: zero res
  if (vec[0] == 0 && vec[1] == 0 && vec[2] == 0) {
    sim_math_zero_3(res);
  }

  // null quat: copy vec
  else if (quat[0] == 1 && quat[1] == 0 && quat[2] == 0 && quat[3] == 0) {
    sim_math_internal_copy_3(res, vec);
  }

  // regular processing
  else {
    // tmp = q_w * v + cross(q_xyz, v)
    sim_scalar_t tmp[3] = {
      quat[0]*vec[0] + quat[2]*vec[2] - quat[3]*vec[1],
      quat[0]*vec[1] + quat[3]*vec[0] - quat[1]*vec[2],
      quat[0]*vec[2] + quat[1]*vec[1] - quat[2]*vec[0]
    };

    // res = v + 2 * cross(q_xyz, t)
    res[0] = vec[0] + 2 * (quat[2]*tmp[2] - quat[3]*tmp[1]);
    res[1] = vec[1] + 2 * (quat[3]*tmp[0] - quat[1]*tmp[2]);
    res[2] = vec[2] + 2 * (quat[1]*tmp[1] - quat[2]*tmp[0]);
  }
}


// negate quaternion
void sim_math_negQuat(sim_scalar_t res[4], const sim_scalar_t quat[4]) {
  res[0] = quat[0];
  res[1] = -quat[1];
  res[2] = -quat[2];
  res[3] = -quat[3];
}


// multiply quaternions
void sim_math_mulQuat(sim_scalar_t res[4], const sim_scalar_t qa[4], const sim_scalar_t qb[4]) {
  sim_scalar_t tmp[4] = {
    qa[0]*qb[0] - qa[1]*qb[1] - qa[2]*qb[2] - qa[3]*qb[3],
    qa[0]*qb[1] + qa[1]*qb[0] + qa[2]*qb[3] - qa[3]*qb[2],
    qa[0]*qb[2] - qa[1]*qb[3] + qa[2]*qb[0] + qa[3]*qb[1],
    qa[0]*qb[3] + qa[1]*qb[2] - qa[2]*qb[1] + qa[3]*qb[0]
  };
  res[0] = tmp[0];
  res[1] = tmp[1];
  res[2] = tmp[2];
  res[3] = tmp[3];
}


// multiply quaternion and axis
void sim_math_mulQuatAxis(sim_scalar_t res[4], const sim_scalar_t quat[4], const sim_scalar_t axis[3]) {
  sim_scalar_t tmp[4] = {
    -quat[1]*axis[0] - quat[2]*axis[1] - quat[3]*axis[2],
    quat[0]*axis[0] + quat[2]*axis[2] - quat[3]*axis[1],
    quat[0]*axis[1] + quat[3]*axis[0] - quat[1]*axis[2],
    quat[0]*axis[2] + quat[1]*axis[1] - quat[2]*axis[0]
  };
  res[0] = tmp[0];
  res[1] = tmp[1];
  res[2] = tmp[2];
  res[3] = tmp[3];
}


// convert axisAngle to quaternion
void sim_math_axisAngle2Quat(sim_scalar_t res[4], const sim_scalar_t axis[3], sim_scalar_t angle) {
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
void sim_math_quat2Vel(sim_scalar_t res[3], const sim_scalar_t quat[4], sim_scalar_t dt) {
  sim_scalar_t axis[3] = {quat[1], quat[2], quat[3]};
  sim_scalar_t sin_a_2 = sim_math_normalize_3(axis);
  sim_scalar_t speed = 2 * sim_math_atan2(sin_a_2, quat[0]);

  // when axis-angle is larger than pi, rotation is in the opposite direction
  if (speed > SIM_PI) {
    speed -= 2*SIM_PI;
  }
  speed /= dt;

  sim_math_internal_scl3(res, axis, speed);
}


// Subtract quaternions, express as 3D velocity: qb*quat(res) = qa.
void sim_math_subQuat(sim_scalar_t res[3], const sim_scalar_t qa[4], const sim_scalar_t qb[4]) {
  // qdif = neg(qb)*qa
  sim_scalar_t qneg[4], qdif[4];
  sim_math_internal_negQuat(qneg, qb);
  sim_math_internal_mulQuat(qdif, qneg, qa);

  // convert to 3D velocity
  sim_math_internal_quat2Vel(res, qdif, 1);
}


// convert quaternion to 3D rotation matrix
void sim_math_quat2Mat(sim_scalar_t res[9], const sim_scalar_t quat[4]) {
  // null quat: identity
  if (quat[0] == 1 && quat[1] == 0 && quat[2] == 0 && quat[3] == 0) {
    res[0] = 1;
    res[1] = 0;
    res[2] = 0;
    res[3] = 0;
    res[4] = 1;
    res[5] = 0;
    res[6] = 0;
    res[7] = 0;
    res[8] = 1;
  }

  // regular processing
  else {
    sim_scalar_t q00 = quat[0]*quat[0];
    sim_scalar_t q01 = quat[0]*quat[1];
    sim_scalar_t q02 = quat[0]*quat[2];
    sim_scalar_t q03 = quat[0]*quat[3];
    sim_scalar_t q11 = quat[1]*quat[1];
    sim_scalar_t q12 = quat[1]*quat[2];
    sim_scalar_t q13 = quat[1]*quat[3];
    sim_scalar_t q22 = quat[2]*quat[2];
    sim_scalar_t q23 = quat[2]*quat[3];
    sim_scalar_t q33 = quat[3]*quat[3];

    res[0] = q00 + q11 - q22 - q33;
    res[4] = q00 - q11 + q22 - q33;
    res[8] = q00 - q11 - q22 + q33;

    res[1] = 2*(q12 - q03);
    res[2] = 2*(q13 + q02);
    res[3] = 2*(q12 + q03);
    res[5] = 2*(q23 - q01);
    res[6] = 2*(q13 - q02);
    res[7] = 2*(q23 + q01);
  }
}


// convert 3D rotation matrix to quaternion
void sim_math_mat2Quat(sim_scalar_t quat[4], const sim_scalar_t mat[9]) {
  // q0 largest
  if (mat[0]+mat[4]+mat[8] > 0) {
    quat[0] = 0.5 * sim_math_sqrt(1 + mat[0] + mat[4] + mat[8]);
    quat[1] = 0.25 * (mat[7] - mat[5]) / quat[0];
    quat[2] = 0.25 * (mat[2] - mat[6]) / quat[0];
    quat[3] = 0.25 * (mat[3] - mat[1]) / quat[0];
  }

  // q1 largest
  else if (mat[0] > mat[4] && mat[0] > mat[8]) {
    quat[1] = 0.5 * sim_math_sqrt(1 + mat[0] - mat[4] - mat[8]);
    quat[0] = 0.25 * (mat[7] - mat[5]) / quat[1];
    quat[2] = 0.25 * (mat[1] + mat[3]) / quat[1];
    quat[3] = 0.25 * (mat[2] + mat[6]) / quat[1];
  }

  // q2 largest
  else if (mat[4] > mat[8]) {
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

  sim_math_normalize4(quat);
}


// time-derivative of quaternion, given 3D rotational velocity
void sim_math_derivQuat(sim_scalar_t res[4], const sim_scalar_t quat[4], const sim_scalar_t vel[3]) {
  res[0] = 0.5*(-vel[0]*quat[1] - vel[1]*quat[2] - vel[2]*quat[3]);
  res[1] = 0.5*( vel[0]*quat[0] + vel[1]*quat[3] - vel[2]*quat[2]);
  res[2] = 0.5*(-vel[0]*quat[3] + vel[1]*quat[0] + vel[2]*quat[1]);
  res[3] = 0.5*( vel[0]*quat[2] - vel[1]*quat[1] + vel[2]*quat[0]);
}


// integrate quaternion given 3D angular velocity
void sim_math_quatIntegrate(sim_scalar_t quat[4], const sim_scalar_t vel[3], sim_scalar_t scale) {
  sim_scalar_t angle, tmp[4], qrot[4];

  // form local rotation quaternion, apply
  sim_math_internal_copy_3(tmp, vel);
  angle = scale * sim_math_normalize_3(tmp);
  sim_math_internal_axisAngle2Quat(qrot, tmp, angle);
  sim_math_normalize4(quat);
  sim_math_mulQuat(quat, quat, qrot);
}


// compute quaternion performing rotation from z-axis to given vector
void sim_math_quatZ2Vec(sim_scalar_t quat[4], const sim_scalar_t vec[3]) {
  sim_scalar_t axis[3], a, vn[3] = {vec[0], vec[1], vec[2]}, z[3] = {0, 0, 1};

  // set default result to no-rotation quaternion
  quat[0] = 1;
  sim_math_zero_3(quat+1);

  // normalize vector; if too small, no rotation
  if (sim_math_normalize_3(vn) < SIM_MINVAL) {
    return;
  }

  // compute angle and axis
  sim_math_internal_cross(axis, z, vn);
  a = sim_math_normalize_3(axis);

  // almost parallel
  if (sim_math_abs(a) < SIM_MINVAL) {
    // opposite: 180 deg rotation around x axis
    if (sim_math_dot_3(vn, z) < 0) {
      quat[0] = 0;
      quat[1] = 1;
    }

    return;
  }

  // make quaternion from angle and axis
  a = sim_math_atan2(a, sim_math_dot_3(vn, z));
  sim_math_internal_axisAngle2Quat(quat, axis, a);
}


// extract 3D rotation from an arbitrary 3x3 matrix
static const sim_scalar_t rotEPS = 1e-9;
int sim_math_mat2Rot(sim_scalar_t quat[4], const sim_scalar_t mat[9]) {
  // M眉ller, Matthias, Jan Bender, Nuttapong Chentanez, and Miles Macklin. "A
  // robust method to extract the rotational part of deformations." In
  // Proceedings of the 9th International Conference on Motion in Games, pp.
  // 55-60. 2016.

  int iter;
  sim_scalar_t col1_mat[3] = {mat[0], mat[3], mat[6]};
  sim_scalar_t col2_mat[3] = {mat[1], mat[4], mat[7]};
  sim_scalar_t col3_mat[3] = {mat[2], mat[5], mat[8]};
  for (iter = 0; iter < 500; iter++) {
    sim_scalar_t rot[9];
    sim_math_quat2Mat(rot, quat);
    sim_scalar_t col1_rot[3] = {rot[0], rot[3], rot[6]};
    sim_scalar_t col2_rot[3] = {rot[1], rot[4], rot[7]};
    sim_scalar_t col3_rot[3] = {rot[2], rot[5], rot[8]};
    sim_scalar_t omega[3], vec1[3], vec2[3], vec3[3];
    sim_math_internal_cross(vec1, col1_rot, col1_mat);
    sim_math_internal_cross(vec2, col2_rot, col2_mat);
    sim_math_internal_cross(vec3, col3_rot, col3_mat);
    sim_math_internal_add3(omega, vec1, vec2);
    sim_math_internal_addTo3(omega, vec3);
    sim_math_scale_3(omega, omega, 1.0 / (sim_math_abs(sim_math_dot_3(col1_rot, col1_mat) +
                                          sim_math_dot_3(col2_rot, col2_mat) +
                                          sim_math_dot_3(col3_rot, col3_mat)) + SIM_MINVAL));
    sim_scalar_t w = sim_math_normalize_3(omega);
    if (w < rotEPS) {
      break;
    }
    sim_scalar_t qrot[4];
    sim_math_internal_axisAngle2Quat(qrot, omega, w);
    sim_math_mulQuat(quat, qrot, quat);
    sim_math_normalize4(quat);
  }
  return iter;
}


//------------------------------ pose operations (quat, pos) ---------------------------------------

// multiply two poses
void sim_math_mulPose(sim_scalar_t posres[3], sim_scalar_t quatres[4],
                 const sim_scalar_t pos1[3], const sim_scalar_t quat1[4],
                 const sim_scalar_t pos2[3], const sim_scalar_t quat2[4]) {
  // quatres = quat1*quat2
  sim_math_internal_mulQuat(quatres, quat1, quat2);
  sim_math_normalize4(quatres);

  // posres = quat1*pos2 + pos1
  sim_math_internal_rotVecQuat(posres, pos2, quat1);
  sim_math_internal_addTo3(posres, pos1);
}


// negate pose
void sim_math_negPose(sim_scalar_t posres[3], sim_scalar_t quatres[4], const sim_scalar_t pos[3], const sim_scalar_t quat[4]) {
  // qres = neg(quat)
  sim_math_internal_negQuat(quatres, quat);

  // pres = -neg(quat)*pos
  sim_math_internal_rotVecQuat(posres, pos, quatres);
  sim_math_scale_3(posres, posres, -1);
}


// transform vector by pose
void sim_math_trnVecPose(sim_scalar_t res[3], const sim_scalar_t pos[3], const sim_scalar_t quat[4], const sim_scalar_t vec[3]) {
  // res = quat*vec + pos
  sim_math_internal_rotVecQuat(res, vec, quat);
  sim_math_internal_addTo3(res, pos);
}


//------------------------------ spatial algebra ---------------------------------------------------

// vector cross-product, 3D
void sim_math_cross(sim_scalar_t res[3], const sim_scalar_t a[3], const sim_scalar_t b[3]) {
  sim_scalar_t tmp[3] = {
    a[1]*b[2] - a[2]*b[1],
    a[2]*b[0] - a[0]*b[2],
    a[0]*b[1] - a[1]*b[0]
  };
  res[0] = tmp[0];
  res[1] = tmp[1];
  res[2] = tmp[2];
}


// cross-product for motion vector
void sim_math_crossMotion(sim_scalar_t res[6], const sim_scalar_t vel[6], const sim_scalar_t v[6]) {
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
void sim_math_crossForce(sim_scalar_t res[6], const sim_scalar_t vel[6], const sim_scalar_t f[6]) {
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


// express inertia in com-based frame
void sim_math_inertCom(sim_scalar_t* restrict res, const sim_scalar_t inert[3], const sim_scalar_t mat[9],
                  const sim_scalar_t dif[3], sim_scalar_t mass) {
  // tmp = diag(inert) * mat'  (mat is local-to-global rotation)
  sim_scalar_t tmp[9] = {mat[0]*inert[0], mat[3]*inert[0], mat[6]*inert[0],
                   mat[1]*inert[1], mat[4]*inert[1], mat[7]*inert[1],
                   mat[2]*inert[2], mat[5]*inert[2], mat[8]*inert[2]};

  // res_rot = mat * diag(inert) * mat'
  res[0] = mat[0]*tmp[0] + mat[1]*tmp[3] + mat[2]*tmp[6];
  res[1] = mat[3]*tmp[1] + mat[4]*tmp[4] + mat[5]*tmp[7];
  res[2] = mat[6]*tmp[2] + mat[7]*tmp[5] + mat[8]*tmp[8];
  res[3] = mat[0]*tmp[1] + mat[1]*tmp[4] + mat[2]*tmp[7];
  res[4] = mat[0]*tmp[2] + mat[1]*tmp[5] + mat[2]*tmp[8];
  res[5] = mat[3]*tmp[2] + mat[4]*tmp[5] + mat[5]*tmp[8];

  // res_rot -= mass * dif_cross * dif_cross
  res[0] += mass*(dif[1]*dif[1] + dif[2]*dif[2]);
  res[1] += mass*(dif[0]*dif[0] + dif[2]*dif[2]);
  res[2] += mass*(dif[0]*dif[0] + dif[1]*dif[1]);
  res[3] -= mass*dif[0]*dif[1];
  res[4] -= mass*dif[0]*dif[2];
  res[5] -= mass*dif[1]*dif[2];

  // res_tran = mass * dif
  res[6] = mass*dif[0];
  res[7] = mass*dif[1];
  res[8] = mass*dif[2];

  // res_mass = mass
  res[9] = mass;
}


// multiply 6D vector (rotation, translation) by 6D inertia matrix
void sim_math_mulInertVec(sim_scalar_t* restrict res, const sim_scalar_t i[10], const sim_scalar_t v[6]) {
  res[0] = i[0]*v[0] + i[3]*v[1] + i[4]*v[2] - i[8]*v[4] + i[7]*v[5];
  res[1] = i[3]*v[0] + i[1]*v[1] + i[5]*v[2] + i[8]*v[3] - i[6]*v[5];
  res[2] = i[4]*v[0] + i[5]*v[1] + i[2]*v[2] - i[7]*v[3] + i[6]*v[4];
  res[3] = i[8]*v[1] - i[7]*v[2] + i[9]*v[3];
  res[4] = i[6]*v[2] - i[8]*v[0] + i[9]*v[4];
  res[5] = i[7]*v[0] - i[6]*v[1] + i[9]*v[5];
}


// express motion axis in com-based frame
void sim_math_dofCom(sim_scalar_t* restrict res, const sim_scalar_t axis[3], const sim_scalar_t offset[3]) {
  // hinge
  if (offset) {
    sim_math_internal_copy_3(res, axis);
    sim_math_internal_cross(res+3, axis, offset);
  }

  // slide
  else {
    sim_math_zero_3(res);
    sim_math_internal_copy_3(res+3, axis);
  }
}


// multiply dof matrix (6-by-n, transposed) by vector (n-by-1)
void sim_math_mulDofVec(sim_scalar_t* res, const sim_scalar_t* dof, const sim_scalar_t* vec, int n) {
  if (n == 1) {
    sim_math_scl(res, dof, vec[0], 6);
  } else if (n <= 0) {
    sim_math_zero(res, 6);
  } else {
    sim_math_mulMatTVec(res, dof, vec, n, 6);
  }
}


// transform 6D motion or force vector between frames
//   flg_force: determines vector type (motion or force)
//   rotnew2old: rotation that maps vectors from new to old frame,
//               its columns are the new frame's axes, expressed in the old frame
//   oldpos and newpos are expressed in old frame
void sim_math_transformSpatial(sim_scalar_t res[6], const sim_scalar_t vec[6], int flg_force,
                          const sim_scalar_t newpos[3], const sim_scalar_t oldpos[3],
                          const sim_scalar_t rotnew2old[9]) {
  sim_scalar_t cros[3], dif[3], tran[6];

  // apply translation
  sim_math_copy(tran, vec, 6);
  sim_math_internal_sub3(dif, newpos, oldpos);
  if (flg_force) {
    sim_math_internal_cross(cros, dif, vec+3);
    sim_math_internal_sub3(tran, vec, cros);
  } else {
    sim_math_internal_cross(cros, dif, vec);
    sim_math_internal_sub3(tran+3, vec+3, cros);
  }

  // if provided, apply old -> new rotation
  if (rotnew2old) {
    sim_math_internal_mulMatTVec3(res, rotnew2old, tran);
    sim_math_internal_mulMatTVec3(res+3, rotnew2old, tran+3);
  }

  // otherwise copy
  else {
    sim_math_internal_copy6(res, tran);
  }
}


// make 3D frame given X axis (normal) and possibly Y axis (tangent 1)
void sim_math_makeFrame(sim_scalar_t frame[9]) {
  sim_scalar_t tmp[3];

  // normalize xaxis
  if (sim_math_normalize_3(frame) < 0.5) {
    SIM_ERROR("xaxis of contact frame undefined");
  }

  // if yaxis undefined, set yaxis to (0,1,0) if possible, otherwise (0,0,1)
  if (sim_math_dot_3(frame+3, frame+3) < 0.25) {
    sim_math_zero_3(frame+3);

    if (frame[1] < 0.5 && frame[1] > -0.5) {
      frame[4] = 1;
    } else {
      frame[5] = 1;
    }
  }

  // make yaxis orthogonal to xaxis
  sim_math_internal_scl3(tmp, frame, sim_math_dot_3(frame, frame+3));
  sim_math_internal_subFrom3(frame+3, tmp);
  sim_math_normalize_3(frame+3);

  // zaxis = cross(xaxis, yaxis)
  sim_math_internal_cross(frame+6, frame, frame+3);
}


// convert sequence of Euler angles (radians) to quaternion
// seq[0,1,2] must be in 'xyzXYZ', lower/upper-case mean intrinsic/extrinsic rotations
void sim_math_euler2Quat(sim_scalar_t quat[4], const sim_scalar_t euler[3], const char* seq) {
  if (strnlen(seq, 4) != 3) {
    SIM_ERROR("seq must contain exactly 3 characters");
  }

  // init
  sim_scalar_t tmp[4] = {1, 0, 0, 0};

  // loop over euler angles, accumulate rotations
  for (int i=0; i<3; i++) {
    // construct quaternion rotation
    sim_scalar_t rot[4] = {cos(euler[i]/2), 0, 0, 0};
    sim_scalar_t sa = sin(euler[i]/2);
    if (seq[i]=='x' || seq[i]=='X') {
      rot[1] = sa;
    } else if (seq[i]=='y' || seq[i]=='Y') {
      rot[2] = sa;
    } else if (seq[i]=='z' || seq[i]=='Z') {
      rot[3] = sa;
    } else {
      SIM_ERROR("seq[%d] is '%c', should be one of x, y, z, X, Y, Z", i, seq[i]);
    }

    // accumulate rotation
    if (seq[i]=='x' || seq[i]=='y' || seq[i]=='z') {
      sim_math_mulQuat(tmp, tmp, rot);  // moving axes: post-multiply
    } else {
      sim_math_mulQuat(tmp, rot, tmp);  // fixed axes: pre-multiply
    }
  }

  sim_math_internal_copy4(quat, tmp);
}
