// Copyright 2022 DeepMind Technologies Limited
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

#include "engine/engine_derivative.h"

#include <simcore/SIM_data.h>
#include <simcore/SIM_model.h>
#include <simcore/SIM_san.h>  // IWYU pragma: keep
#include "engine/engine_core_util.h"
#include "engine/engine_crossplatform.h"
#include "engine/engine_memory.h"
#include "engine/engine_passive.h"
#include "engine/engine_sleep.h"
#include "engine/engine_support.h"
#include "engine/engine_util_blas.h"
#include "engine/engine_util_errmem.h"
#include "engine/engine_util_misc.h"
#include "engine/engine_util_spatial.h"
#include "engine/engine_util_sparse.h"



//------------------------- derivatives of spatial algebra -----------------------------------------


// derivatives of cross product, Da and Db are 3x3
static void SIM_d_cross(const sim_scalar_t a[3], const sim_scalar_t b[3],
                      sim_scalar_t* restrict Da, sim_scalar_t* restrict Db) {
  // derivative w.r.t a
  if (Da) {
    sim_math_zero(Da, 9);
    Da[1] =  b[2];
    Da[2] = -b[1];
    Da[3] = -b[2];
    Da[5] =  b[0];
    Da[6] =  b[1];
    Da[7] = -b[0];
  }

  // derivative w.r.t b
  if (Db) {
    sim_math_zero(Db, 9);
    Db[1] = -a[2];
    Db[2] =  a[1];
    Db[3] =  a[2];
    Db[5] = -a[0];
    Db[6] = -a[1];
    Db[7] =  a[0];
  }
}


// derivative of sim_math_crossMotion w.r.t velocity
static void SIM_d_crossMotion_vel(sim_scalar_t D[36], const sim_scalar_t v[6]) {
  sim_math_zero(D, 36);

  // res[0] = -vel[2]*v[1] + vel[1]*v[2]
  D[0 + 2] = -v[1];
  D[0 + 1] = v[2];

  // res[1] =  vel[2]*v[0] - vel[0]*v[2]
  D[6 + 2] = v[0];
  D[6 + 0] = -v[2];

  // res[2] = -vel[1]*v[0] + vel[0]*v[1]
  D[12 + 1] = -v[0];
  D[12 + 0] = v[1];

  // res[3] = -vel[2]*v[4] + vel[1]*v[5] - vel[5]*v[1] + vel[4]*v[2]
  D[18 + 2] = -v[4];
  D[18 + 1] = v[5];
  D[18 + 5] = -v[1];
  D[18 + 4] = v[2];

  // res[4] =  vel[2]*v[3] - vel[0]*v[5] + vel[5]*v[0] - vel[3]*v[2]
  D[24 + 2] = v[3];
  D[24 + 0] = -v[5];
  D[24 + 5] = v[0];
  D[24 + 3] = -v[2];

  // res[5] = -vel[1]*v[3] + vel[0]*v[4] - vel[4]*v[0] + vel[3]*v[1]
  D[30 + 1] = -v[3];
  D[30 + 0] = v[4];
  D[30 + 4] = -v[0];
  D[30 + 3] = v[1];
}


// derivative of sim_math_crossForce w.r.t. velocity
static void SIM_d_crossForce_vel(sim_scalar_t D[36], const sim_scalar_t f[6]) {
  sim_math_zero(D, 36);

  // res[0] = -vel[2]*f[1] + vel[1]*f[2] - vel[5]*f[4] + vel[4]*f[5]
  D[0 + 2] = -f[1];
  D[0 + 1] = f[2];
  D[0 + 5] = -f[4];
  D[0 + 4] = f[5];

  // res[1] =  vel[2]*f[0] - vel[0]*f[2] + vel[5]*f[3] - vel[3]*f[5]
  D[6 + 2] = f[0];
  D[6 + 0] = -f[2];
  D[6 + 5] = f[3];
  D[6 + 3] = -f[5];

  // res[2] = -vel[1]*f[0] + vel[0]*f[1] - vel[4]*f[3] + vel[3]*f[4]
  D[12 + 1] = -f[0];
  D[12 + 0] = f[1];
  D[12 + 4] = -f[3];
  D[12 + 3] = f[4];

  // res[3] = -vel[2]*f[4] + vel[1]*f[5]
  D[18 + 2] = -f[4];
  D[18 + 1] = f[5];

  // res[4] =  vel[2]*f[3] - vel[0]*f[5]
  D[24 + 2] = f[3];
  D[24 + 0] = -f[5];

  // res[5] = -vel[1]*f[3] + vel[0]*f[4]
  D[30 + 1] = -f[3];
  D[30 + 0] = f[4];
}


// derivative of sim_math_crossForce w.r.t. force
static void SIM_d_crossForce_frc(sim_scalar_t D[36], const sim_scalar_t vel[6]) {
  sim_math_zero(D, 36);

  // res[0] = -vel[2]*f[1] + vel[1]*f[2] - vel[5]*f[4] + vel[4]*f[5]
  D[0 + 1] = -vel[2];
  D[0 + 2] = vel[1];
  D[0 + 4] = -vel[5];
  D[0 + 5] = vel[4];

  // res[1] =  vel[2]*f[0] - vel[0]*f[2] + vel[5]*f[3] - vel[3]*f[5]
  D[6 + 0] = vel[2];
  D[6 + 2] = -vel[0];
  D[6 + 3] = vel[5];
  D[6 + 5] = -vel[3];

  // res[2] = -vel[1]*f[0] + vel[0]*f[1] - vel[4]*f[3] + vel[3]*f[4]
  D[12 + 0] = -vel[1];
  D[12 + 1] = vel[0];
  D[12 + 3] = -vel[4];
  D[12 + 4] = vel[3];

  // res[3] = -vel[2]*f[4] + vel[1]*f[5]
  D[18 + 4] = -vel[2];
  D[18 + 5] = vel[1];

  // res[4] =  vel[2]*f[3] - vel[0]*f[5]
  D[24 + 3] = vel[2];
  D[24 + 5] = -vel[0];

  // res[5] = -vel[1]*f[3] + vel[0]*f[4]
  D[30 + 3] = -vel[1];
  D[30 + 4] = vel[0];
}


// derivative of sim_math_mulInertVec w.r.t vel
static void SIM_d_mulInertVec_vel(sim_scalar_t D[36], const sim_scalar_t i[10]) {
  sim_math_zero(D, 36);

  // res[0] = i[0]*v[0] + i[3]*v[1] + i[4]*v[2] - i[8]*v[4] + i[7]*v[5]
  D[0 + 0] = i[0];
  D[0 + 1] = i[3];
  D[0 + 2] = i[4];
  D[0 + 4] = -i[8];
  D[0 + 5] = i[7];

  // res[1] = i[3]*v[0] + i[1]*v[1] + i[5]*v[2] + i[8]*v[3] - i[6]*v[5]
  D[6 + 0] = i[3];
  D[6 + 1] = i[1];
  D[6 + 2] = i[5];
  D[6 + 3] = i[8];
  D[6 + 5] = -i[6];

  // res[2] = i[4]*v[0] + i[5]*v[1] + i[2]*v[2] - i[7]*v[3] + i[6]*v[4]
  D[12 + 0] = i[4];
  D[12 + 1] = i[5];
  D[12 + 2] = i[2];
  D[12 + 3] = -i[7];
  D[12 + 4] = i[6];

  // res[3] = i[8]*v[1] - i[7]*v[2] + i[9]*v[3]
  D[18 + 1] = i[8];
  D[18 + 2] = -i[7];
  D[18 + 3] = i[9];

  // res[4] = i[6]*v[2] - i[8]*v[0] + i[9]*v[4]
  D[24 + 2] = i[6];
  D[24 + 0] = -i[8];
  D[24 + 4] = i[9];

  // res[5] = i[7]*v[0] - i[6]*v[1] + i[9]*v[5]
  D[30 + 0] = i[7];
  D[30 + 1] = -i[6];
  D[30 + 5] = i[9];
}


// derivative of sim_math_subQuat w.r.t inputs
void SIM_d_subQuat(const sim_scalar_t qa[4], const sim_scalar_t qb[4], sim_scalar_t Da[9], sim_scalar_t Db[9]) {
  // no outputs, quick return
  if (!Da && !Db) {
    return;
  }

  // compute axis-angle quaternion difference
  sim_scalar_t axis[3];
  sim_math_subQuat(axis, qa, qb);

  // normalize axis, get half-angle
  sim_scalar_t half_angle = 0.5 * sim_math_normalize_3(axis);

  // identity
  sim_scalar_t Da_tmp[9] = {
    1, 0, 0,
    0, 1, 0,
    0, 0, 1
  };

  // add term linear in cross product matrix K
  sim_scalar_t K[9] = {
    0, -axis[2], axis[1],
    axis[2], 0, -axis[0],
    -axis[1], axis[0], 0
  };
  sim_math_addToScl(Da_tmp, K, half_angle, 9);

  // add term linear in K * K
  sim_scalar_t KK[9];
  sim_math_mulMatMat3(KK, K, K);
  sim_scalar_t coef = 1.0 - (half_angle < 6e-8 ? 1.0 : half_angle / sim_math_tan(half_angle));
  sim_math_addToScl(Da_tmp, KK, coef, 9);

  if (Da) {
    sim_math_copy9(Da, Da_tmp);
  }

  if (Db) {  // Db = -Da^T
    sim_math_transpose(Db, Da_tmp, 3, 3);
    sim_math_scl(Db, Db, -1.0, 9);
  }
}


// derivative of sim_math_quatIntegrate w.r.t scaled velocity
//  reference: https://arxiv.org/abs/1711.02508, Eq. 183
void SIM_d_quatIntegrate(const sim_scalar_t vel[3], sim_scalar_t scale,
                       sim_scalar_t Dquat[9], sim_scalar_t Dvel[9], sim_scalar_t Dscale[3]) {
  // scaled velocity
  sim_scalar_t s[3] = {scale*vel[0], scale*vel[1], scale*vel[2]};

  // 3 basis matrices
  sim_scalar_t eye[9] = {
    1, 0, 0,
    0, 1, 0,
    0, 0, 1
  };
  sim_scalar_t cross[9] = {
    0,     s[2], -s[1],
   -s[2],  0,     s[0],
    s[1], -s[0],  0
  };
  sim_scalar_t outer[9] = {
    s[0]*s[0], s[0]*s[1], s[0]*s[2],
    s[1]*s[0], s[1]*s[1], s[1]*s[2],
    s[2]*s[0], s[2]*s[1], s[2]*s[2]
  };

  // squared norm, norm of s
  sim_scalar_t xx = sim_math_dot_3(s, s);
  sim_scalar_t x = sim_math_sqrt(xx);

  // 4 coefficients: a=cos(x), b=sin(x)/x, c=(1-cos(x))/x^2, d=(x-sin(x))/x^3
  sim_scalar_t a = sim_math_cos(x);
  sim_scalar_t b, c, d;

  // x is not small: use full expressions
  if (sim_math_abs(x) > 1.0/32) {
    b = sim_math_sin(x) / x;
    c = (1.0 - a) / xx;
    d = (1.0 - b) / xx;
  }

  // |x| <= 1/32: use 6th order Taylor expansion (Horner form)
  else {
    b =  1 + xx/6  * (xx/20 * (1 - xx/42) - 1);
    c = (1 + xx/12 * (xx/30 * (1 - xx/56) - 1)) / 2;
    d = (1 + xx/20 * (xx/42 * (1 - xx/72) - 1)) / 6;
  }

  // derivatives
  sim_scalar_t Dvel_[9];
  for (int i=0; i < 9; i++) {
    if (Dquat)          Dquat[i] = a*eye[i] + b*cross[i] + c*outer[i];
    if (Dvel || Dscale) Dvel_[i] = b*eye[i] + c*cross[i] + d*outer[i];
  }
  if (Dvel) sim_math_copy9(Dvel, Dvel_);
  if (Dscale) sim_math_mul_mat_vec_3(Dscale, Dvel_, vel);
}


//------------------------- dense derivatives of component functions -------------------------------
// no longer used, except in tests

// derivative of cvel, cdof_dot w.r.t qvel (dense version)
static void SIM_d_comVel_vel_dense(const sim_model_t* m, sim_data_t* d, sim_scalar_t* Dcvel, sim_scalar_t* Dcdofdot) {
  int nv = m->nv, nbody = m->nbody;
  sim_scalar_t mat[36];

  // clear Dcvel
  sim_math_zero(Dcvel, nbody*6*nv);

  // forward pass over bodies: accumulate Dcvel, set Dcdofdot
  for (int i=1; i < nbody; i++) {
    // Dcvel = Dcvel_parent
    sim_math_copy(Dcvel+i*6*nv, Dcvel+m->body_parentid[i]*6*nv, 6*nv);

    // Dcvel += D(cdof * qvel),  Dcdofdot = D(cvel x cdof)
    for (int j=m->body_dofadr[i]; j < m->body_dofadr[i]+m->body_dofnum[i]; j++) {
      switch ((SIM_tJoint) m->jnt_type[m->dof_jntid[j]]) {
      case SIM_JNT_FREE:
        // Dcdofdot = 0
        sim_math_zero(Dcdofdot+j*6*nv, 18*nv);

        // Dcvel += cdof * (D qvel)
        for (int k=0; k < 6; k++) {
          Dcvel[i*6*nv + k*nv + j+0] += d->cdof[(j+0)*6 + k];
          Dcvel[i*6*nv + k*nv + j+1] += d->cdof[(j+1)*6 + k];
          Dcvel[i*6*nv + k*nv + j+2] += d->cdof[(j+2)*6 + k];
        }

        // continue with rotations
        j += 3;
        SIM_FALLTHROUGH;

      case SIM_JNT_BALL:
        // Dcdofdot = D crossMotion(cvel, cdof)
        for (int k=0; k < 3; k++) {
          SIM_d_crossMotion_vel(mat, d->cdof+6*(j+k));
          sim_math_mulMatMat(Dcdofdot+(j+k)*6*nv, mat, Dcvel+i*6*nv, 6, 6, nv);
        }

        // Dcvel += cdof * (D qvel)
        for (int k=0; k < 6; k++) {
          Dcvel[i*6*nv + k*nv + j+0] += d->cdof[(j+0)*6 + k];
          Dcvel[i*6*nv + k*nv + j+1] += d->cdof[(j+1)*6 + k];
          Dcvel[i*6*nv + k*nv + j+2] += d->cdof[(j+2)*6 + k];
        }

        // adjust for 3-dof joint
        j += 2;
        break;

      default:
        // Dcdofdot = D crossMotion(cvel, cdof) * Dcvel
        SIM_d_crossMotion_vel(mat, d->cdof+6*j);
        sim_math_mulMatMat(Dcdofdot+j*6*nv, mat, Dcvel+i*6*nv, 6, 6, nv);

        // Dcvel += cdof * (D qvel)
        for (int k=0; k < 6; k++) {
          Dcvel[i*6*nv + k*nv + j] += d->cdof[j*6 + k];
        }
      }
    }
  }
}


// subtract (d qfrc_bias / d qvel) from qDeriv (dense version)
void SIM_d_rne_vel_dense(const sim_model_t* m, sim_data_t* d) {
  int nv = m->nv, nbody = m->nbody;
  sim_scalar_t mat[36], mat1[36], mat2[36], dmul[36], tmp[6];

  sim_markStack(d);
  sim_scalar_t* Dcvel = SIM_STACK_ALLOC(d, nbody*6*nv, sim_scalar_t);
  sim_scalar_t* Dcdofdot = SIM_STACK_ALLOC(d, nv*6*nv, sim_scalar_t);
  sim_scalar_t* Dcacc = SIM_STACK_ALLOC(d, nbody*6*nv, sim_scalar_t);
  sim_scalar_t* Dcfrcbody = SIM_STACK_ALLOC(d, nbody*6*nv, sim_scalar_t);
  sim_scalar_t* row = SIM_STACK_ALLOC(d, nv, sim_scalar_t);

  // compute Dcvel and Dcdofdot
  SIM_d_comVel_vel_dense(m, d, Dcvel, Dcdofdot);

  // clear Dcacc
  sim_math_zero(Dcacc, nbody*6*nv);

  // forward pass over bodies: accumulate Dcacc, set Dcfrcbody
  for (int i=1; i < nbody; i++) {
    // Dcacc = Dcacc_parent
    sim_math_copy(Dcacc + i*6*nv, Dcacc + m->body_parentid[i]*6*nv, 6*nv);

    // Dcacc += D(cdofdot * qvel)
    for (int j=m->body_dofadr[i]; j < m->body_dofadr[i]+m->body_dofnum[i]; j++) {
      // Dcacc += cdofdot * (D qvel)
      for (int k=0; k < 6; k++) {
        Dcacc[i*6*nv + k*nv + j] += d->cdof_dot[j*6 + k];
      }

      // Dcacc += (D cdofdot) * qvel
      sim_math_addToScl(Dcacc+i*6*nv, Dcdofdot+j*6*nv, d->qvel[j], 6*nv);
    }

    //---------- Dcfrcbody = D(cinert * cacc + cvel x (cinert * cvel))

    // Dcfrcbody = (D mul / D cacc) * Dcacc
    SIM_d_mulInertVec_vel(dmul, d->cinert+10*i);
    sim_math_mulMatMat(Dcfrcbody+i*6*nv, dmul, Dcacc+i*6*nv, 6, 6, nv);

    // mat = (D cross / D cvel) + (D cross / D mul) * (D mul / D cvel)
    sim_math_mulInertVec(tmp, d->cinert+10*i, d->cvel+i*6);
    SIM_d_crossForce_vel(mat, tmp);
    SIM_d_crossForce_frc(mat1, d->cvel+i*6);
    sim_math_mulMatMat(mat2, mat1, dmul, 6, 6, 6);
    sim_math_addTo(mat, mat2, 36);

    // Dcfrcbody += mat * Dcvel  (use body 0 as temp)
    sim_math_mulMatMat(Dcfrcbody, mat, Dcvel+i*6*nv, 6, 6, nv);
    sim_math_addTo(Dcfrcbody+i*6*nv, Dcfrcbody, 6*nv);
  }

  // clear world Dcfrcbody, for style
  sim_math_zero(Dcfrcbody, 6*nv);

  // backward pass over bodies: accumulate Dcfrcbody
  for (int i=nbody-1; i > 0; i--) {
    if (m->body_parentid[i]) {
      sim_math_addTo(Dcfrcbody+m->body_parentid[i]*6*nv, Dcfrcbody+i*6*nv, 6*nv);
    }
  }

  // qDeriv -= D(cdof * cfrc_body)
  for (int i=0; i < nv; i++) {
    for (int k=0; k < 6; k++) {
      // compute D(cdof * cfrc_body), store in row
      sim_math_scl(row, Dcfrcbody + (m->dof_bodyid[i]*6+k)*nv, d->cdof[i*6+k], nv);

      // dense to sparse: qDeriv -= row
      int end = m->D_rowadr[i] + m->D_rownnz[i];
      for (int adr=m->D_rowadr[i]; adr < end; adr++) {
        d->qDeriv[adr] -= row[m->D_colind[adr]];
      }
    }
  }

  sim_freeStack(d);
}


//------------------------- sparse derivatives of component functions ------------------------------
// internal sparse format: dense body/dof x sparse dof x 6   (inner size is 6)

// copy sparse B-row from parent, shared ancestors only
static void copyFromParent(const sim_model_t* m, sim_data_t* d, sim_scalar_t* mat, int n) {
  // return if this is world or parent is world
  if (n == 0 || m->body_weldid[m->body_parentid[n]] == 0) {
    return;
  }

  // count dofs in ancestors
  int ndof = 0;
  int np = m->body_weldid[m->body_parentid[n]];
  while (np > 0) {
    // add self dofs
    ndof += m->body_dofnum[np];

    // advance to parent
    np = m->body_weldid[m->body_parentid[np]];
  }

  // copy: guaranteed to be at beginning of sparse array, due to sorting
  sim_math_copy(mat + 6*m->B_rowadr[n], mat + 6*m->B_rowadr[m->body_parentid[n]], 6*ndof);
}


// add sparse B-row to parent, all overlapping nonzeros
static void addToParent(const sim_model_t* m, sim_data_t* d, sim_scalar_t* mat, int n) {
  // return if this is world or parent is world
  if (n == 0 || m->body_weldid[m->body_parentid[n]] == 0) {
    return;
  }

  // find matching nonzeros
  int np = m->body_parentid[n];
  int i = 0, ip = 0;
  while (i < m->B_rownnz[n] && ip < m->B_rownnz[np]) {
    // columns match
    if (m->B_colind[m->B_rowadr[n] + i] == m->B_colind[m->B_rowadr[np] + ip]) {
      sim_math_addTo(mat + 6*(m->B_rowadr[np] + ip), mat + 6*(m->B_rowadr[n] + i), 6);

      // advance both
      i++;
      ip++;
    }

    // mismatch columns: advance parent
    else if (m->B_colind[m->B_rowadr[n] + i] > m->B_colind[m->B_rowadr[np] + ip]) {
      ip++;
    }

    // child nonzeroes must be subset of parent; SHOULD NOT OCCUR
    else {
      SIM_ERROR("child nonzeroes must be subset of parent");
    }
  }
}


// derivative of cvel, cdof_dot w.r.t qvel
static void SIM_d_comVel_vel(const sim_model_t* m, sim_data_t* d, sim_scalar_t* Dcvel, sim_scalar_t* Dcdofdot) {
  int nv = m->nv, nM = m->nM;
  int sleep_filter = SIM_ENABLED(SIM_ENBL_SLEEP) && d->nbody_awake < m->nbody;
  int nbody = sleep_filter ? d->nbody_awake : m->nbody;
  int* Badr = m->B_rowadr, * Dadr = m->D_rowadr;
  sim_scalar_t mat[36], matT[36];   // 6x6 matrices

  // forward pass over bodies: accumulate Dcvel, set Dcdofdot
  for (int b=1; b < nbody; b++) {
    int i = sleep_filter ? d->body_awake_ind[b] : b;

    // Dcvel = Dcvel_parent
    copyFromParent(m, d, Dcvel, i);

    // process all dofs of this body
    int doflast = m->body_dofadr[i] + m->body_dofnum[i];
    for (int j = m->body_dofadr[i]; j < doflast; j++) {
      // number of dof ancestors of dof j
      int Jadr = (j < nv - 1 ? m->dof_Madr[j + 1] : nM) - (m->dof_Madr[j] + 1);

      // Dcvel += D(cdof * qvel),  Dcdofdot = D(cvel x cdof)
      switch ((SIM_tJoint) m->jnt_type[m->dof_jntid[j]]) {
      case SIM_JNT_FREE:
        // Dcdofdot = 0 (already cleared)

        // Dcvel += cdof * D(qvel)
        sim_math_addTo(Dcvel + 6*(Badr[i] + Jadr + 0), d->cdof + 6*(j + 0), 6);
        sim_math_addTo(Dcvel + 6*(Badr[i] + Jadr + 1), d->cdof + 6*(j + 1), 6);
        sim_math_addTo(Dcvel + 6*(Badr[i] + Jadr + 2), d->cdof + 6*(j + 2), 6);

        // continue with rotations
        j += 3;
        Jadr += 3;
        SIM_FALLTHROUGH;

      case SIM_JNT_BALL:
        // Dcdofdot = Dcvel * D crossMotion(cvel, cdof)
        for (int dj=0; dj < 3; dj++) {
          SIM_d_crossMotion_vel(mat, d->cdof + 6 * (j + dj));
          sim_math_transpose(matT, mat, 6, 6);
          sim_math_mulMatMat(Dcdofdot + 6*Dadr[j + dj], Dcvel + 6*Badr[i], matT, Jadr + dj, 6, 6);
        }

        // Dcvel += cdof * (D qvel)
        sim_math_addTo(Dcvel + 6*(Badr[i] + Jadr + 0), d->cdof + 6*(j + 0), 6);
        sim_math_addTo(Dcvel + 6*(Badr[i] + Jadr + 1), d->cdof + 6*(j + 1), 6);
        sim_math_addTo(Dcvel + 6*(Badr[i] + Jadr + 2), d->cdof + 6*(j + 2), 6);

        // adjust for 3-dof joint
        j += 2;
        break;

      case SIM_JNT_HINGE:
      case SIM_JNT_SLIDE:
        // Dcdofdot = D crossMotion(cvel, cdof) * Dcvel
        SIM_d_crossMotion_vel(mat, d->cdof + 6 * j);
        sim_math_transpose(matT, mat, 6, 6);
        sim_math_mulMatMat(Dcdofdot + 6*Dadr[j], Dcvel + 6*Badr[i], matT, Jadr, 6, 6);

        // Dcvel += cdof * (D qvel)
        sim_math_addTo(Dcvel + 6*(Badr[i] + Jadr), d->cdof + 6*j, 6);
        break;

      default:
        SIM_ERROR("unknown joint type");
      }
    }
  }
}


// subtract d qfrc_bias / d qvel from qDeriv
static void SIM_d_rne_vel(const sim_model_t* m, sim_data_t* d) {
  int nM = m->nM;
  int sleep_filter = SIM_ENABLED(SIM_ENBL_SLEEP) && d->nbody_awake < m->nbody;
  int nbody = sleep_filter ? d->nbody_awake : m->nbody;
  int nparent = sleep_filter ? d->nparent_awake : m->nbody;
  int mnv = m->nv;
  int nv = sleep_filter ? d->nv_awake : mnv;

  const int* Badr = m->B_rowadr;
  const int* Dadr = m->D_rowadr;
  const int* Bnnz = m->B_rownnz;

  sim_scalar_t mat[36], mat1[36], mat2[36], dmul[36], tmp[6];

  sim_markStack(d);
  sim_scalar_t* Dcdofdot = SIM_STACK_ALLOC(d, 6*m->nD, sim_scalar_t);
  sim_scalar_t* Dcvel = SIM_STACK_ALLOC(d, 6*m->nB, sim_scalar_t);
  sim_scalar_t* Dcacc = SIM_STACK_ALLOC(d, 6*m->nB, sim_scalar_t);
  sim_scalar_t* Dcfrcbody = SIM_STACK_ALLOC(d, 6*m->nB, sim_scalar_t);
  sim_scalar_t* row = SIM_STACK_ALLOC(d, m->nv, sim_scalar_t);

  // clear
  if (!sleep_filter) {
    sim_math_zero(Dcdofdot,  6*m->nD);
    sim_math_zero(Dcvel,     6*m->nB);
    sim_math_zero(Dcacc,     6*m->nB);
    sim_math_zero(Dcfrcbody, 6*m->nB);
  } else {
    for (int i = 0; i < nv; i++) {
      int dof = d->dof_awake_ind[i];
      sim_math_zero(Dcdofdot + 6*m->D_rowadr[dof], 6*m->D_rownnz[dof]);
    }

    for (int i = 0; i < nbody; i++) {
      int body = d->body_awake_ind[i];
      int adr = 6*m->B_rowadr[body];
      int nnz = 6*m->B_rownnz[body];
      sim_math_zero(Dcvel     + adr, nnz);
      sim_math_zero(Dcacc     + adr, nnz);
      sim_math_zero(Dcfrcbody + adr, nnz);
    }
  }

  // compute Dcvel and Dcdofdot
  SIM_d_comVel_vel(m, d, Dcvel, Dcdofdot);

  // forward pass over bodies: accumulate Dcacc, set Dcfrcbody
  for (int b=1; b < nbody; b++) {
    int i = sleep_filter ? d->body_awake_ind[b] : b;

    // Dcacc = Dcacc_parent
    copyFromParent(m, d, Dcacc, i);

    // process all dofs of this body
    int doflast = m->body_dofadr[i] + m->body_dofnum[i];
    for (int j=m->body_dofadr[i]; j < doflast; j++) {
      // number of dof ancestors of dof j
      int Jadr = (j < mnv - 1 ? m->dof_Madr[j + 1] : nM) - (m->dof_Madr[j] + 1);

      // Dcacc += cdofdot * (D qvel)
      sim_math_addTo(Dcacc + 6*(Badr[i] + Jadr), d->cdof_dot + 6*j, 6);

      // Dcacc += (D cdofdot) * qvel
      // Dcacc[row i] and Dcdofdot[row j] have identical sparsity
      sim_math_addToScl(Dcacc + 6*Badr[i], Dcdofdot + 6*Dadr[j], d->qvel[j], 6*Bnnz[i]);
    }

    //---------- Dcfrcbody = D(cinert * cacc + cvel x (cinert * cvel))

    // Dcfrcbody = (D mul / D cacc) * Dcacc
    SIM_d_mulInertVec_vel(dmul, d->cinert + 10*i);
    sim_math_transpose(mat1, dmul, 6, 6);
    sim_math_mulMatMat(Dcfrcbody + 6*Badr[i], Dcacc + 6*Badr[i], mat1, Bnnz[i], 6, 6);

    // mat = (D cross / D cvel) + (D cross / D mul) * (D mul / D cvel)
    sim_math_mulInertVec(tmp, d->cinert + 10*i, d->cvel + i*6);
    SIM_d_crossForce_vel(mat, tmp);
    SIM_d_crossForce_frc(mat1, d->cvel + i*6);
    sim_math_mulMatMat(mat2, mat1, dmul, 6, 6, 6);
    sim_math_addTo(mat, mat2, 36);

    // Dcfrcbody += mat * Dcvel  (use worldbody as temp)
    sim_math_transpose(mat1, mat, 6, 6);
    sim_math_mulMatMat(Dcfrcbody, Dcvel + 6*Badr[i], mat1, Bnnz[i], 6, 6);
    sim_math_addTo(Dcfrcbody + 6*Badr[i], Dcfrcbody, 6*Bnnz[i]);
  }

  // clear worldbody Dcfrcbody
  sim_math_zero(Dcfrcbody, 6*Bnnz[0]);

  // backward pass over bodies: accumulate Dcfrcbody
  for (int b=nparent-1; b > 0; b--) {
    int i = sleep_filter ? d->parent_awake_ind[b] : b;
    addToParent(m, d, Dcfrcbody, i);
  }

  // process all dofs, update qDeriv
  for (int v=0; v < nv; v++) {
    int j = sleep_filter ? d->dof_awake_ind[v] : v;

    // get body index
    int i = m->dof_bodyid[j];

    // qDeriv -= D(cdof * cfrc_body)
    sim_math_mulMatVec(row, Dcfrcbody + 6*Badr[i], d->cdof + 6*j, Bnnz[i], 6);
    sim_math_subFrom(d->qDeriv + Dadr[j], row, Bnnz[i]);
  }

  sim_freeStack(d);
}


//--------------------- utility functions for (d force / d vel) Jacobians --------------------------

// add J'*B*J to qDeriv
static void addJTBJ(const sim_model_t* m, sim_data_t* d, const sim_scalar_t* J, const sim_scalar_t* B, int n) {
  int nv = m->nv;

  // allocate dense row
  sim_markStack(d);
  sim_scalar_t* row = SIM_STACK_ALLOC(d, nv, sim_scalar_t);

  // process non-zero elements of B
  for (int i=0; i < n; i++) {
    for (int j=0; j < n; j++) {
      if (!B[i*n+j]) {
        continue;
      }
      // process non-zero elements of J(i,:)
      for (int k=0; k < nv; k++) {
        if (J[i*nv+k]) {
          // row = J(i,k)*B(i,j)*J(j,:)
          sim_math_scl(row, J+j*nv, J[i*nv+k] * B[i*n+j], nv);

          // add row to qDeriv(k,:)
          int rownnz_k = m->D_rownnz[k];
          for (int s=0; s < rownnz_k; s++) {
            int adr = m->D_rowadr[k] + s;
            d->qDeriv[adr] += row[m->D_colind[adr]];
          }
        }
      }
    }
  }

  sim_freeStack(d);
}


// add J'*B*J to qDeriv, sparse version
static void addJTBJSparse(
  const sim_model_t* m, sim_data_t* d, const sim_scalar_t* J,
  const sim_scalar_t* B, int n, int offset,
  const int* J_rownnz, const int* J_rowadr, const int* J_colind) {

  // compute qDeriv(k,p) += sum_{i,j} ( J(i,k)*B(i,j)*J(j,p) )
  for (int i = 0; i < n; i++) {
    for (int j = 0; j < n; j++) {
      if (!B[i*n+j]) {
        continue;
      }

      // loop over non-zero elements of J(i,:)
      int nnz_i = J_rownnz[offset+i];
      int adr_i = J_rowadr[offset+i];
      int nnz_j = J_rownnz[offset+j];
      int adr_j = J_rowadr[offset+j];
      for (int k = 0; k < nnz_i; k++) {
        int ik = adr_i + k;
        int colik = J_colind[ik];

        // qDeriv(k,:) += J(j,:) * J(i,k)*B(i,j)
        sim_math_addToSclSparseInc(d->qDeriv + m->D_rowadr[colik], J + adr_j,
                              m->D_rownnz[colik], m->D_colind + m->D_rowadr[colik],
                              nnz_j, J_colind + adr_j,
                              J[ik]*B[i*n+j]);
      }
    }
  }
}


//----------------------------- derivatives of actuator forces -------------------------------------

// derivative of sim_math_muscleGain w.r.t velocity
static sim_scalar_t SIM_d_muscleGain_vel(sim_scalar_t len, sim_scalar_t vel, const sim_scalar_t lengthrange[2], sim_scalar_t acc0,
                                 const sim_scalar_t prm[9]) {
  // unpack parameters
  sim_scalar_t range[2] = {prm[0], prm[1]};
  sim_scalar_t force    = prm[2];
  sim_scalar_t scale    = prm[3];
  sim_scalar_t lmin     = prm[4];
  sim_scalar_t lmax     = prm[5];
  sim_scalar_t vmax     = prm[6];
  sim_scalar_t fvmax    = prm[8];

  // scale force if negative
  if (force < 0) {
    force = scale / sim_math_max(SIM_MINVAL, acc0);
  }

  // optimum length
  sim_scalar_t L0 = (lengthrange[1]-lengthrange[0]) / sim_math_max(SIM_MINVAL, range[1]-range[0]);

  // normalized length and velocity
  sim_scalar_t L = range[0] + (len-lengthrange[0]) / sim_math_max(SIM_MINVAL, L0);
  sim_scalar_t V = vel / sim_math_max(SIM_MINVAL, L0*vmax);

  // length curve
  sim_scalar_t FL = sim_math_muscleGainLength(L, lmin, lmax);

  // velocity curve
  sim_scalar_t dFV;
  sim_scalar_t y = fvmax-1;
  if (V <= -1) {
    // FV = 0
    dFV = 0;
  } else if (V <= 0) {
    // FV = (V+1)*(V+1)
    dFV = 2*V + 2;
  } else if (V <= y) {
    // FV = fvmax - (y-V)*(y-V) / sim_math_max(SIM_MINVAL, y)
    dFV = (-2*V + 2*y) / sim_math_max(SIM_MINVAL, y);
  } else {
    // FV = fvmax
    dFV = 0;
  }

  // compute FVL and scale, make it negative
  return -force*FL*dFV/sim_math_max(SIM_MINVAL, L0*vmax);
}


//--------------------- utility functions for (d force / d pos) * vec Jacobians --------------------

// add J'*B*J*vec to res, sparse version
static void addJTBJ_mulSparse(const sim_model_t* m, sim_data_t* d, sim_scalar_t* res, const sim_scalar_t* vec,
                              const int* J_rownnz, const int* J_rowadr, const int* J_colind,
                              const sim_scalar_t* J, const sim_scalar_t* B, int n) {
  // allocate temp vectors
  sim_markStack(d);
  sim_scalar_t* Jv = SIM_STACK_ALLOC(d, n, sim_scalar_t);
  sim_scalar_t* BJv = SIM_STACK_ALLOC(d, n, sim_scalar_t);

  // Jv = J*vec (Sparse Matrix-Vector Multiplication)
  sim_math_zero(Jv, n);
  for (int i=0; i < n; i++) {
    int nnz = J_rownnz[i];
    int adr = J_rowadr[i];
    for (int k=0; k < nnz; k++) {
      Jv[i] += J[adr + k] * vec[J_colind[adr + k]];
    }
  }

  // BJv = B*Jv (Dense Matrix-Vector Multiplication)
  sim_math_mulMatVec(BJv, B, Jv, n, n);

  // res += J'*BJv (Sparse Transpose Matrix-Vector Multiplication)
  for (int i=0; i < n; i++) {
    int nnz = J_rownnz[i];
    int adr = J_rowadr[i];
    sim_scalar_t val = BJv[i];
    for (int k=0; k < nnz; k++) {
      res[J_colind[adr + k]] += J[adr + k] * val;
    }
  }

  sim_freeStack(d);
}


// operation type for flex interpolation derivative kernel
typedef enum {
  SIM_FLEXOP_VEC,    // res += J'*K*J*vec
  SIM_FLEXOP_ADDH    // H -= J'*K*J to H (dense)
} SIM_tFlexOp;

// shared kernel for flex interpolation derivatives, scale = s1 + s2*damping
//  op: operation type (VEC, or ADDH)
//  res: output vector (VEC) or dense H matrix (ADDH)
//  vec: input vector for VEC operation, NULL otherwise
//  dof_indices, ndof: DOF mapping for ADDH, ignored otherwise
static void SIM_d_flexInterp_kernel(const sim_model_t* m, sim_data_t* d, SIM_tFlexOp op,
                                  sim_scalar_t* res, const sim_scalar_t* vec, sim_scalar_t s1, sim_scalar_t s2,
                                  const int* dof_indices, int ndof) {
  int nv = m->nv;

  // build global2local map for ADDH
  int* global2local = NULL;
  if (op == SIM_FLEXOP_ADDH) {
    sim_markStack(d);
    global2local = SIM_STACK_ALLOC(d, nv, int);
    sim_math_fillInt(global2local, -1, nv);
    for (int i=0; i<ndof; i++) {
      global2local[dof_indices[i]] = i;
    }
  }

  // loop over flexes
  for (int f=0; f < m->nflex; f++) {
    // only process flex_interp
    if (!m->flex_interp[f]) {
      continue;
    }

    // get stiffness and damping
    sim_scalar_t* k = m->flex_stiffness + 21*m->flex_elemadr[f];

    // skip if rigid or no stiffness
    if (m->flex_rigid[f] || k[0] == 0) {
      continue;
    }

    // compute scale
    sim_scalar_t damping = m->flex_damping[f];
    sim_scalar_t scale = s1 + s2 * damping;

    // skip if scale is zero
    if (scale == 0) {
      continue;
    }

    int nodenum = m->flex_nodenum[f];
    int* bodyid = m->flex_nodebodyid + m->flex_nodeadr[f];

    // standard stack allocation
    sim_markStack(d);
    sim_scalar_t* xpos = SIM_STACK_ALLOC(d, 3*nodenum, sim_scalar_t);
    sim_scalar_t* K_rot = SIM_STACK_ALLOC(d, 9*nodenum*nodenum, sim_scalar_t);

    // sparse Jacobian allocations
    int dim = 3 * nodenum;
    int* rownnz = SIM_STACK_ALLOC(d, dim, int);
    int* rowadr = SIM_STACK_ALLOC(d, dim, int);
    sim_scalar_t* J_val = SIM_STACK_ALLOC(d, dim*nv, sim_scalar_t);
    int* J_colind = SIM_STACK_ALLOC(d, dim*nv, int);

    // temp allocations for chain
    int* chain_colind = SIM_STACK_ALLOC(d, nv, int);
    sim_scalar_t* blk_jac = SIM_STACK_ALLOC(d, 3*nv, sim_scalar_t);

    // compute positions, rotation and Jacobian
    sim_scalar_t quat[4] = {1, 0, 0, 0};
    sim_flexInterpState(m, d, f, xpos, NULL, quat);

    // compute generalized stiffness in global frame: K_rot = R * K * R^T
    sim_scalar_t R[9];
    sim_math_quat2Mat(R, quat);       // R = R_global2local
    sim_scalar_t RT[9];
    sim_math_transpose(RT, R, 3, 3);  // RT = R_local2global

    // blockwise rotation: K_rot(i,j) = scale * RT * K_local(i,j) * R
    // note: k stores -K, so K_rot = scale * (-K_phys)
    for (int i=0; i < nodenum; i++) {
      for (int j=0; j < nodenum; j++) {
        sim_scalar_t blk[9], tmp[9];

        // get K_local(i,j)
        int adr = (3*i)*(3*nodenum) + 3*j;
        for (int r=0; r < 3; r++) {
          for (int c=0; c < 3; c++) {
            blk[3*r+c] = k[adr + r*(3*nodenum) + c];
          }
        }

        // tmp = K * R
        sim_math_mulMatMat3(tmp, blk, R);

        // blk = RT * tmp = RT * K * R
        sim_math_mulMatMat3(blk, RT, tmp);

        // store scaled into K_rot
        for (int r=0; r < 3; r++) {
          for (int c=0; c < 3; c++) {
            K_rot[adr + r*(3*nodenum) + c] = scale * blk[3*r+c];
          }
        }
      }
    }

    // construct sparse Jacobian J_val
    int current_adr = 0;
    for (int i=0; i < nodenum; i++) {
        // get chain for this node
        int chain_nnz = sim_bodyChain(m, bodyid[i], chain_colind);

        // compute sparse Jacobian for this node (3 rows)
        sim_jacSparse(m, d, blk_jac, NULL, xpos+3*i, bodyid[i], chain_nnz, chain_colind);

        // copy to sparse structure
        for (int r=0; r<3; r++) {
            int row_idx = 3*i + r;
            rownnz[row_idx] = chain_nnz;
            rowadr[row_idx] = current_adr;

            for (int idx=0; idx<chain_nnz; idx++) {
                J_colind[current_adr] = chain_colind[idx];
                J_val[current_adr] = blk_jac[r*chain_nnz + idx];
                current_adr++;
            }
        }
    }

    // perform operation
    if (op == SIM_FLEXOP_VEC) {
      // res += J^T * K_rot * J * vec
      addJTBJ_mulSparse(m, d, res, vec, rownnz, rowadr, J_colind, J_val, K_rot, dim);
    } else if (op == SIM_FLEXOP_ADDH) {
      // H += -J^T * K_rot * J
      // H is dense ndof x ndof

      // reuse stack for J_reduced (but now we extract from sparse J)
      sim_scalar_t* J_reduced = SIM_STACK_ALLOC(d, dim*ndof, sim_scalar_t);
      sim_math_zero(J_reduced, dim*ndof);

      // extract columns of J into J_reduced
      for (int i=0; i<dim; i++) {
          int nnz = rownnz[i];
          int adr = rowadr[i];
          for (int idx=0; idx<nnz; idx++) {
              int global_col = J_colind[adr + idx];
              int local_idx = global2local[global_col];
              if (local_idx >= 0) {
                  J_reduced[i*ndof + local_idx] = J_val[adr + idx];
              }
          }
      }

      // H -= J_reduced^T * K_rot * J_reduced
      // K_rot * J_reduced (dim x ndof)
      sim_scalar_t* KJ = SIM_STACK_ALLOC(d, dim*ndof, sim_scalar_t);
      sim_math_mulMatMat(KJ, K_rot, J_reduced, dim, dim, ndof);

      // H[i, j] -= sum_k J_reduced[k, i] * KJ[k, j]
      for (int i=0; i<ndof; i++) {
        for (int j=0; j<ndof; j++) {
          sim_scalar_t val = 0;
          for (int dim_idx=0; dim_idx<dim; dim_idx++) {
            val += J_reduced[dim_idx*ndof + i] * KJ[dim_idx*ndof + j];
          }
          // res is H
          res[i*ndof + j] -= val;
        }
      }
    }

    sim_freeStack(d);
  }

  if (op == SIM_FLEXOP_ADDH) {
    sim_freeStack(d);  // free global2local
  }
}



// compute res += (h^2 + h*damping) * J'*K*J * vec, for all interpolated flexes
void SIM_d_flexInterp_mulKD(const sim_model_t* m, sim_data_t* d, sim_scalar_t* res, const sim_scalar_t* vec, sim_scalar_t h) {
  // s1=h*h, s2=h => scale = h*h + h*damping
  SIM_d_flexInterp_kernel(m, d, SIM_FLEXOP_VEC, res, vec, h * h, h, NULL, 0);
}


// add (h^2 + h*damping) * J'*K*J to dense matrix H, for all interpolated flexes
//  H: dense ndof x ndof matrix
//  dof_indices: maps local indices to global DOFs
void SIM_d_flexInterp_addH(const sim_model_t* m, sim_data_t* d, sim_scalar_t* H, const int* dof_indices, int ndof, sim_scalar_t h) {
  SIM_d_flexInterp_kernel(m, d, SIM_FLEXOP_ADDH, H, NULL, h * h, h, dof_indices, ndof);
}





// add (d qfrc_actuator / d qvel) to qDeriv
void SIM_d_actuator_vel(const sim_model_t* m, sim_data_t* d) {
  int nu = m->nu;
  int sleep_filter = SIM_ENABLED(SIM_ENBL_SLEEP) && d->ntree_awake < m->ntree;

  // disabled: nothing to add
  if (SIM_DISABLED(SIM_DSBL_ACTUATION)) {
    return;
  }

  // process actuators
  for (int i=0; i < nu; i++) {
    // skip if disabled
    if (sim_actuatorDisabled(m, i)) {
      continue;
    }

    // skip if sleeping
    if (sleep_filter && sim_sleepState(m, d, SIM_OBJ_ACTUATOR, i) == sim_spec_ASLEEP) {
      continue;
    }

    // skip if force is clamped by forcerange
    if (m->actuator_forcelimited[i]) {
      sim_scalar_t force = d->actuator_force[i];
      sim_scalar_t* range = m->actuator_forcerange + 2*i;
      if (force <= range[0] || force >= range[1]) {
        continue;
      }
    }

    sim_scalar_t bias_vel = 0, gain_vel = 0;

    // affine bias
    if (m->actuator_biastype[i] == SIM_BIAS_AFFINE) {
      // extract bias info: prm = [const, kp, kv]
      bias_vel = (m->actuator_biasprm + SIM_NBIAS*i)[2];
    }

    // affine gain
    if (m->actuator_gaintype[i] == SIM_GAIN_AFFINE) {
      // extract bias info: prm = [const, kp, kv]
      gain_vel = (m->actuator_gainprm + SIM_NGAIN*i)[2];
    }

    // muscle gain
    else if (m->actuator_gaintype[i] == SIM_GAIN_MUSCLE) {
      gain_vel = SIM_d_muscleGain_vel(d->actuator_length[i],
                                    d->actuator_velocity[i],
                                    m->actuator_lengthrange+2*i,
                                    m->actuator_acc0[i],
                                    m->actuator_gainprm + SIM_NGAIN*i);
    }

    // force = gain .* [ctrl/act]
    if (gain_vel != 0) {
      if (m->actuator_dyntype[i] == SIM_DYN_NONE) {
        bias_vel += gain_vel * d->ctrl[i];
      } else {
        int act_adr = m->actuator_actadr[i] + m->actuator_actnum[i] - 1;
        sim_scalar_t act = d->act[act_adr];

        // use next activation if actearly is set (matching forward pass)
        if (m->actuator_actearly[i]) {
          act = sim_nextActivation(m, d, i, act_adr, d->act_dot[act_adr]);
        }

        bias_vel += gain_vel * act;
      }
    }

    // add
    if (bias_vel != 0) {
      addJTBJSparse(m, d, d->actuator_moment, &bias_vel, 1, i,
                    d->moment_rownnz, d->moment_rowadr, d->moment_colind);
    }
  }
}


//----------------- utilities for ellipsoid-based fluid force derivatives --------------------------

static inline sim_scalar_t pow2(const sim_scalar_t val) {
  return val*val;
}


static inline sim_scalar_t ellipsoid_max_moment(const sim_scalar_t size[3], const int dir) {
  const sim_scalar_t d0 = size[dir];
  const sim_scalar_t d1 = size[(dir+1) % 3];
  const sim_scalar_t d2 = size[(dir+2) % 3];
  return 8.0/15.0 * SIM_PI * d0 * pow2(pow2(sim_math_max(d1, d2)));
}


// add 3x3 matrix D to one of the four quadrants of the 6x6 matrix B
//   row_quad and col_quad should be either 0 or 1 (not checked)
static void addToQuadrant(sim_scalar_t* restrict B, const sim_scalar_t D[9], int col_quad, int row_quad) {
  int r = 3*row_quad, c = 3*col_quad;
  B[6*(c+0) + r+0] += D[0];
  B[6*(c+0) + r+1] += D[1];
  B[6*(c+0) + r+2] += D[2];
  B[6*(c+1) + r+0] += D[3];
  B[6*(c+1) + r+1] += D[4];
  B[6*(c+1) + r+2] += D[5];
  B[6*(c+2) + r+0] += D[6];
  B[6*(c+2) + r+1] += D[7];
  B[6*(c+2) + r+2] += D[8];
}


//----------------- components of ellipsoid-based fluid force derivatives --------------------------

// forces due to fluid mass moving with the body, B is 6x6
static void SIM_d_addedMassForces(
  sim_scalar_t* restrict B, const sim_scalar_t local_vels[6], const sim_scalar_t fluid_density,
  const sim_scalar_t virtual_mass[3], const sim_scalar_t virtual_inertia[3]) {
  const sim_scalar_t lin_vel[3] = {local_vels[3], local_vels[4], local_vels[5]};
  const sim_scalar_t ang_vel[3] = {local_vels[0], local_vels[1], local_vels[2]};
  const sim_scalar_t virtual_lin_mom[3] = {
    fluid_density * virtual_mass[0] * lin_vel[0],
    fluid_density * virtual_mass[1] * lin_vel[1],
    fluid_density * virtual_mass[2] * lin_vel[2]
  };
  const sim_scalar_t virtual_ang_mom[3] = {
    fluid_density * virtual_inertia[0] * ang_vel[0],
    fluid_density * virtual_inertia[1] * ang_vel[1],
    fluid_density * virtual_inertia[2] * ang_vel[2]
  };
  sim_scalar_t Da[9];
  sim_scalar_t Db[9];

  // force[:3] += cross(virtual_ang_mom, ang_vel)
  SIM_d_cross(virtual_ang_mom, ang_vel, Da, Db);
  addToQuadrant(B, Db, 0, 0);
  for (int i=0; i < 9; ++i) {
    Da[i] *= fluid_density * virtual_inertia[i % 3];
  }
  addToQuadrant(B, Da, 0, 0);

  // force[:3] += cross(virtual_lin_mom, lin_vel)
  SIM_d_cross(virtual_lin_mom, lin_vel, Da, Db);
  addToQuadrant(B, Db, 0, 1);
  for (int i=0; i < 9; ++i) {
    Da[i] *= fluid_density * virtual_mass[i % 3];
  }
  addToQuadrant(B, Da, 0, 1);

  // force[3:] += cross(virtual_lin_mom, ang_vel)
  SIM_d_cross(virtual_lin_mom, ang_vel, Da, Db);
  addToQuadrant(B, Db, 1, 0);
  for (int i=0; i < 9; ++i) {
    Da[i] *= fluid_density * virtual_mass[i % 3];
  }
  addToQuadrant(B, Da, 1, 1);
}


// torque due to motion in the fluid, D is 3x3
static inline void SIM_d_viscous_torque(
  sim_scalar_t* restrict D, const sim_scalar_t lvel[6], const sim_scalar_t fluid_density,
  const sim_scalar_t fluid_viscosity, const sim_scalar_t size[3],
  const sim_scalar_t slender_drag_coef, const sim_scalar_t ang_drag_coef) {
  const sim_scalar_t d_max = sim_math_max(sim_math_max(size[0], size[1]), size[2]);
  const sim_scalar_t d_min = sim_math_min(sim_math_min(size[0], size[1]), size[2]);
  const sim_scalar_t d_mid = size[0] + size[1] + size[2] - d_max - d_min;
  // viscous force and torque in Stokes flow, analytical for spherical bodies
  const sim_scalar_t eq_sphere_D = 2.0/3.0 * (size[0] + size[1] + size[2]);
  const sim_scalar_t lin_visc_torq_coef = SIM_PI * eq_sphere_D*eq_sphere_D*eq_sphere_D;

  // moments of inertia used to compute angular quadratic drag
  const sim_scalar_t I_max = 8.0/15.0 * SIM_PI * d_mid * (d_max*d_max)*(d_max*d_max);
  const sim_scalar_t II[3] = {
    ellipsoid_max_moment(size, 0),
    ellipsoid_max_moment(size, 1),
    ellipsoid_max_moment(size, 2)
  };
  const sim_scalar_t x = lvel[0], y = lvel[1], z = lvel[2];
  const sim_scalar_t mom_coef[3] = {
    ang_drag_coef*II[0] + slender_drag_coef*(I_max - II[0]),
    ang_drag_coef*II[1] + slender_drag_coef*(I_max - II[1]),
    ang_drag_coef*II[2] + slender_drag_coef*(I_max - II[2])
  };
  const sim_scalar_t mom_visc[3] = {
    x * mom_coef[0],
    y * mom_coef[1],
    z * mom_coef[2]
  };
  const sim_scalar_t density = fluid_density / sim_math_max(SIM_MINVAL, sim_math_norm3(mom_visc));

  // -density * [x, y, z] * mom_coef^2
  const sim_scalar_t mom_sq[3] = {
    -density * x * mom_coef[0] * mom_coef[0],
    -density * y * mom_coef[1] * mom_coef[1],
    -density * z * mom_coef[2] * mom_coef[2]
  };
  const sim_scalar_t lin_coef = fluid_viscosity * lin_visc_torq_coef;

  // initialize
  sim_math_zero(D, 9);

  // set diagonal
  D[0] = D[4] = D[8] = x*mom_sq[0] + y*mom_sq[1] + z*mom_sq[2] - lin_coef;

  // add outer product
  sim_math_add_to_scale_3(D, mom_sq, x);
  sim_math_add_to_scale_3(D+3, mom_sq, y);
  sim_math_add_to_scale_3(D+6, mom_sq, z);
}


// drag due to motion in the fluid, D is 3x3
static inline void SIM_d_viscous_drag(
  sim_scalar_t* restrict D, const sim_scalar_t lvel[6], const sim_scalar_t fluid_density,
  const sim_scalar_t fluid_viscosity, const sim_scalar_t size[3],
  const sim_scalar_t blunt_drag_coef, const sim_scalar_t slender_drag_coef) {
  const sim_scalar_t d_max = sim_math_max(sim_math_max(size[0], size[1]), size[2]);
  const sim_scalar_t d_min = sim_math_min(sim_math_min(size[0], size[1]), size[2]);
  const sim_scalar_t d_mid = size[0] + size[1] + size[2] - d_max - d_min;
  // viscous force and torque in Stokes flow, analytical for spherical bodies
  const sim_scalar_t eq_sphere_D = 2.0/3.0 * (size[0] + size[1] + size[2]);
  const sim_scalar_t A_max = SIM_PI * d_max * d_mid;

  const sim_scalar_t a = pow2(size[1] * size[2]);
  const sim_scalar_t b = pow2(size[2] * size[0]);
  const sim_scalar_t c = pow2(size[0] * size[1]);
  const sim_scalar_t aa = a*a, bb = b*b, cc = c*c;

  const sim_scalar_t x = lvel[3], y = lvel[4], z = lvel[5];
  const sim_scalar_t xx = x*x, yy = y*y, zz = z*z, xy=x*y, yz=y*z, xz=x*z;

  const sim_scalar_t proj_denom = aa*xx + bb*yy + cc*zz;
  const sim_scalar_t proj_num = a*xx + b*yy + c*zz;
  const sim_scalar_t dA_coef = SIM_PI / sim_math_max(SIM_MINVAL,
                                        sim_math_sqrt(proj_num*proj_num*proj_num * proj_denom));

  const sim_scalar_t A_proj = SIM_PI * sim_math_sqrt(proj_denom/sim_math_max(SIM_MINVAL, proj_num));

  const sim_scalar_t norm = sim_math_sqrt(xx + yy + zz);
  const sim_scalar_t inv_norm = 1.0 / sim_math_max(SIM_MINVAL, norm);

  const sim_scalar_t lin_coef = fluid_viscosity * 3.0 * SIM_PI * eq_sphere_D;
  const sim_scalar_t quad_coef = fluid_density * (
    A_proj*blunt_drag_coef + slender_drag_coef*(A_max - A_proj));
  const sim_scalar_t Aproj_coef = fluid_density * norm * (blunt_drag_coef - slender_drag_coef);

  const sim_scalar_t dAproj_dv[3] = {
    Aproj_coef * dA_coef * a * x * (b * yy * (a - b) + c * zz * (a - c)),
    Aproj_coef * dA_coef * b * y * (a * xx * (b - a) + c * zz * (b - c)),
    Aproj_coef * dA_coef * c * z * (a * xx * (c - a) + b * yy * (c - b))
  };

  // outer product
  D[0] = xx;  D[1] = xy;  D[2] = xz;
  D[3] = xy;  D[4] = yy;  D[5] = yz;
  D[6] = xz;  D[7] = yz;  D[8] = zz;

  // diag(D) += dot([x y z], [x y z])
  sim_scalar_t inner = xx + yy + zz;
  D[0] += inner;
  D[4] += inner;
  D[8] += inner;

  // scale by -quad_coef*inv_norm
  sim_math_scl(D, D, -quad_coef*inv_norm, 9);

  // D += outer_product(-[x y z], dAproj_dv)
  sim_math_add_to_scale_3(D+0, dAproj_dv, -x);
  sim_math_add_to_scale_3(D+3, dAproj_dv, -y);
  sim_math_add_to_scale_3(D+6, dAproj_dv, -z);

  // diag(D) -= lin_coef
  D[0] -= lin_coef;
  D[4] -= lin_coef;
  D[8] -= lin_coef;
}


// Kutta lift due to motion in the fluid, D is 3x3
static inline void SIM_d_kutta_lift(
  sim_scalar_t* restrict D, const sim_scalar_t lvel[6], const sim_scalar_t fluid_density,
  const sim_scalar_t size[3], const sim_scalar_t kutta_lift_coef) {
  const sim_scalar_t a = pow2(size[1] * size[2]);
  const sim_scalar_t b = pow2(size[2] * size[0]);
  const sim_scalar_t c = pow2(size[0] * size[1]);
  const sim_scalar_t aa = a*a, bb = b*b, cc = c*c;
  const sim_scalar_t x = lvel[3], y = lvel[4], z = lvel[5];
  const sim_scalar_t xx = x*x, yy = y*y, zz = z*z, xy=x*y, yz=y*z, xz=x*z;

  const sim_scalar_t proj_denom = aa * xx + bb * yy + cc * zz;
  const sim_scalar_t proj_num = a * xx + b * yy + c * zz;
  const sim_scalar_t norm2 = xx + yy + zz;
  const sim_scalar_t df_denom = SIM_PI * kutta_lift_coef * fluid_density / sim_math_max(
    SIM_MINVAL, sim_math_sqrt(proj_denom * proj_num * norm2));

  const sim_scalar_t dfx_coef = yy * (a - b) + zz * (a - c);
  const sim_scalar_t dfy_coef = xx * (b - a) + zz * (b - c);
  const sim_scalar_t dfz_coef = xx * (c - a) + yy * (c - b);
  const sim_scalar_t proj_term = proj_num / sim_math_max(SIM_MINVAL, proj_denom);
  const sim_scalar_t cos_term = proj_num / sim_math_max(SIM_MINVAL, norm2);

  // cosA = proj_num/(norm*proj_denom), A_proj = pi*sqrt(proj_denom/proj_num)
  // F = cosA * A_proj * (([a,b,c] * vel) \times vel) \times vel
  // derivative obtained with SymPy

  D[0] = a-a;  D[1] = b-a;  D[2] = c-a;
  D[3] = a-b;  D[4] = b-b;  D[5] = c-b;
  D[6] = a-c;  D[7] = b-c;  D[8] = c-c;
  sim_math_scl(D, D, 2 * proj_num, 9);

  const sim_scalar_t inner_term[3] = {
    aa * proj_term - a + cos_term,
    bb * proj_term - b + cos_term,
    cc * proj_term - c + cos_term
  };
  sim_math_add_to_scale_3(D + 0, inner_term, dfx_coef);
  sim_math_add_to_scale_3(D + 3, inner_term, dfy_coef);
  sim_math_add_to_scale_3(D + 6, inner_term, dfz_coef);

  D[0] *= xx;  D[1] *= xy;  D[2] *= xz;
  D[3] *= xy;  D[4] *= yy;  D[5] *= yz;
  D[6] *= xz;  D[7] *= yz;  D[8] *= zz;

  D[0] -= dfx_coef * proj_num;
  D[4] -= dfy_coef * proj_num;
  D[8] -= dfz_coef * proj_num;

  sim_math_scl(D, D, df_denom, 9);
}


// Magnus force due to motion in the fluid, B is 6x6
static inline void SIM_d_magnus_force(
  sim_scalar_t* restrict B, const sim_scalar_t lvel[6], const sim_scalar_t fluid_density,
  const sim_scalar_t size[3], const sim_scalar_t magnus_lift_coef) {
  const sim_scalar_t volume = 4.0/3.0 * SIM_PI * size[0] * size[1] * size[2];

  // magnus_coef = magnus_lift_coef * fluid_density * volume
  const sim_scalar_t magnus_coef = magnus_lift_coef * fluid_density * volume;

  sim_scalar_t D_lin[9], D_ang[9];

  // premultiply by magnus_coef
  const sim_scalar_t lin_vel[3] = {
    magnus_coef * lvel[3], magnus_coef * lvel[4], magnus_coef * lvel[5]
  };
  const sim_scalar_t ang_vel[3] = {
    magnus_coef * lvel[0], magnus_coef * lvel[1], magnus_coef * lvel[2]
  };

  // force[3:] += magnus_coef * cross(ang_vel, lin_vel)
  SIM_d_cross(ang_vel, lin_vel, D_ang, D_lin);

  addToQuadrant(B, D_ang, 1, 0);
  addToQuadrant(B, D_lin, 1, 1);
}


//----------------- fluid force derivatives, ellipsoid and inertia-box models ----------------------

// fluid forces based on ellipsoid approximation
void SIM_d_ellipsoidFluid(const sim_model_t* m, sim_data_t* d, int bodyid) {
  sim_markStack(d);

  int nv = m->nv;
  int nnz = nv;
  int rownnz[6], rowadr[6];
  sim_scalar_t* J = SIM_STACK_ALLOC(d, 6*nv, sim_scalar_t);
  sim_scalar_t* tmp = SIM_STACK_ALLOC(d, 3*nv, sim_scalar_t);
  int* colind = SIM_STACK_ALLOC(d, 6*nv, int);
  int* colind_compressed = SIM_STACK_ALLOC(d, 6*nv, int);

  sim_scalar_t lvel[6], wind[6], lwind[6];
  sim_scalar_t geom_interaction_coef, magnus_lift_coef, kutta_lift_coef;
  sim_scalar_t semiaxes[3], virtual_mass[3], virtual_inertia[3];
  sim_scalar_t blunt_drag_coef, slender_drag_coef, ang_drag_coef;

  if (sim_isSparse(m)) {
    // get sparse body Jacobian structure
    nnz = sim_bodyChain(m, bodyid, colind);

    // prepare rownnz, rowadr, colind for all 6 rows
    for (int i=0; i < 6; i++) {
      rownnz[i] = nnz;
      rowadr[i] = i == 0 ? 0 : rowadr[i-1] + nnz;
      for (int k=0; k < nnz; k++) {
        colind_compressed[i*nnz+k] = colind[k];
      }
    }
  }

  for (int j=0; j < m->body_geomnum[bodyid]; j++) {
    const int geomid = m->body_geomadr[bodyid] + j;

    sim_math_geomSemiAxes(semiaxes, m->geom_size + 3*geomid, m->geom_type[geomid]);

    readFluidGeomInteraction(
      m->geom_fluid + SIM_NFLUID*geomid, &geom_interaction_coef,
      &blunt_drag_coef, &slender_drag_coef, &ang_drag_coef,
      &kutta_lift_coef, &magnus_lift_coef,
      virtual_mass, virtual_inertia);

    // scales all forces, read from SIMCF as boolean (0.0 or 1.0)
    if (geom_interaction_coef == 0.0) {
      continue;
    }

    // map from CoM-centered to local body-centered 6D velocity
    sim_objectVelocity(m, d, SIM_OBJ_GEOM, geomid, lvel, 1);
    // compute wind in local coordinates
    sim_math_zero(wind, 6);
    sim_math_copy_3(wind+3, m->opt.wind);
    sim_math_transformSpatial(lwind, wind, 0,
                         d->geom_xpos + 3*geomid,  // Frame of ref's origin.
                         d->subtree_com + 3*m->body_rootid[bodyid],
                         d->geom_xmat + 9*geomid);  // Frame of ref's orientation.
    // subtract translational component from grom velocity
    sim_math_subFrom3(lvel+3, lwind+3);

    // get geom global Jacobian: rotation then translation
    if (sim_isSparse(m)) {
      sim_jacSparse(m, d, J+3*nnz, J, d->geom_xpos+3*geomid, m->geom_bodyid[geomid], nnz, colind);
    } else {
      sim_jacGeom(m, d, J+3*nv, J, geomid);
    }

    // rotate (compressed) Jacobian to local frame
    sim_math_mulMatTMat(tmp, d->geom_xmat+9*geomid, J, 3, 3, nnz);
    sim_math_copy(J, tmp, 3*nnz);
    sim_math_mulMatTMat(tmp, d->geom_xmat+9*geomid, J+3*nnz, 3, 3, nnz);
    sim_math_copy(J+3*nnz, tmp, 3*nnz);

    sim_scalar_t B[36], D[9];
    sim_math_zero(B, 36);
    SIM_d_magnus_force(B, lvel, m->opt.density, semiaxes, magnus_lift_coef);

    SIM_d_kutta_lift(D, lvel, m->opt.density, semiaxes, kutta_lift_coef);
    addToQuadrant(B, D, 1, 1);

    SIM_d_viscous_drag(D, lvel, m->opt.density, m->opt.viscosity, semiaxes,
                     blunt_drag_coef, slender_drag_coef);
    addToQuadrant(B, D, 1, 1);

    SIM_d_viscous_torque(D, lvel, m->opt.density, m->opt.viscosity, semiaxes,
                       slender_drag_coef, ang_drag_coef);
    addToQuadrant(B, D, 0, 0);

    SIM_d_addedMassForces(B, lvel, m->opt.density, virtual_mass, virtual_inertia);

    // make B symmetric if integrator is IMPLICITFAST
    if (m->opt.integrator == SIM_INT_IMPLICITFAST) {
      sim_math_symmetrize(B, B, 6);
    }

    if (sim_isSparse(m)) {
      addJTBJSparse(m, d, J, B, 6, 0, rownnz, rowadr, colind_compressed);
    } else {
      addJTBJ(m, d, J, B, 6);
    }
  }

  sim_freeStack(d);
}


// fluid forces based on inertia-box approximation
void SIM_d_inertiaBoxFluid(const sim_model_t* m, sim_data_t* d, int i) {
  sim_markStack(d);

  int nv = m->nv;
  int rownnz[6], rowadr[6];
  sim_scalar_t* J = SIM_STACK_ALLOC(d, 6*nv, sim_scalar_t);
  sim_scalar_t* tmp = SIM_STACK_ALLOC(d, 3*nv, sim_scalar_t);
  int* colind = SIM_STACK_ALLOC(d, 6*nv, int);

  sim_scalar_t lvel[6], wind[6], lwind[6], box[3], B;
  sim_scalar_t* inertia = m->body_inertia + 3*i;

  // equivalent inertia box
  box[0] = sim_math_sqrt(sim_math_max(SIM_MINVAL,
                            (inertia[1] + inertia[2] - inertia[0])) / m->body_mass[i] * 6.0);
  box[1] = sim_math_sqrt(sim_math_max(SIM_MINVAL,
                            (inertia[0] + inertia[2] - inertia[1])) / m->body_mass[i] * 6.0);
  box[2] = sim_math_sqrt(sim_math_max(SIM_MINVAL,
                            (inertia[0] + inertia[1] - inertia[2])) / m->body_mass[i] * 6.0);

  // map from CoM-centered to local body-centered 6D velocity
  sim_objectVelocity(m, d, SIM_OBJ_BODY, i, lvel, 1);

  // compute wind in local coordinates
  sim_math_zero(wind, 6);
  sim_math_copy_3(wind+3, m->opt.wind);
  sim_math_transformSpatial(lwind, wind, 0, d->xipos+3*i,
                       d->subtree_com+3*m->body_rootid[i], d->ximat+9*i);

  // subtract translational component from body velocity
  sim_math_subFrom3(lvel+3, lwind+3);

  // init with dense
  int nnz = nv;

  // sparse Jacobian
  if (sim_isSparse(m)) {
    // get sparse body Jacobian structure
    nnz = sim_bodyChain(m, i, colind);

    // get sparse jacBodyCom
    sim_jacSparse(m, d, J+3*nnz, J, d->xipos+3*i, i, nnz, colind);

    // prepare rownnz, rowadr, colind for all 6 rows
    rownnz[0] = nnz;
    rowadr[0] = 0;
    for (int j=1; j < 6; j++) {
      rownnz[j] = nnz;
      rowadr[j] = rowadr[j-1] + nnz;
      for (int k=0; k < nnz; k++) {
        colind[j*nnz+k] = colind[k];
      }
    }
  }

  // dense Jacobian
  else {
    sim_jacBodyCom(m, d, J+3*nv, J, i);
  }

  // rotate (compressed) Jacobian to local frame
  sim_math_mulMatTMat(tmp, d->ximat+9*i, J, 3, 3, nnz);
  sim_math_copy(J, tmp, 3*nnz);
  sim_math_mulMatTMat(tmp, d->ximat+9*i, J+3*nnz, 3, 3, nnz);
  sim_math_copy(J+3*nnz, tmp, 3*nnz);

  // add viscous force and torque
  if (m->opt.viscosity > 0) {
    // diameter of sphere approximation
    sim_scalar_t diam = (box[0] + box[1] + box[2])/3.0;

    // sim_math_scale_3(lfrc, lvel, -SIM_PI*diam*diam*diam*m->opt.viscosity)
    B = -SIM_PI*diam*diam*diam*m->opt.viscosity;
    for (int j=0; j < 3; j++) {
      if (sim_isSparse(m)) {
        addJTBJSparse(m, d, J, &B, 1, j, rownnz, rowadr, colind);
      } else {
        addJTBJ(m, d, J+j*nv, &B, 1);
      }
    }

    // sim_math_scale_3(lfrc+3, lvel+3, -3.0*SIM_PI*diam*m->opt.viscosity);
    B = -3.0*SIM_PI*diam*m->opt.viscosity;
    for (int j=0; j < 3; j++) {
      if (sim_isSparse(m)) {
        addJTBJSparse(m, d, J, &B, 1, 3+j, rownnz, rowadr, colind);
      } else {
        addJTBJ(m, d, J+3*nv+j*nv, &B, 1);
      }
    }
  }

  // add lift and drag force and torque
  if (m->opt.density > 0) {
    // lfrc[0] -= m->opt.density*box[0]*(box[1]*box[1]*box[1]*box[1]+box[2]*box[2]*box[2]*box[2])*
    //            sim_math_abs(lvel[0])*lvel[0]/64.0;
    B = -m->opt.density*box[0]*(box[1]*box[1]*box[1]*box[1]+box[2]*box[2]*box[2]*box[2])*
        2*sim_math_abs(lvel[0])/64.0;
    if (sim_isSparse(m)) {
      addJTBJSparse(m, d, J, &B, 1, 0, rownnz, rowadr, colind);
    } else {
      addJTBJ(m, d, J, &B, 1);
    }

    // lfrc[1] -= m->opt.density*box[1]*(box[0]*box[0]*box[0]*box[0]+box[2]*box[2]*box[2]*box[2])*
    //            sim_math_abs(lvel[1])*lvel[1]/64.0;
    B = -m->opt.density*box[1]*(box[0]*box[0]*box[0]*box[0]+box[2]*box[2]*box[2]*box[2])*
        2*sim_math_abs(lvel[1])/64.0;
    if (sim_isSparse(m)) {
      addJTBJSparse(m, d, J, &B, 1, 1, rownnz, rowadr, colind);
    } else {
      addJTBJ(m, d, J+nv, &B, 1);
    }

    // lfrc[2] -= m->opt.density*box[2]*(box[0]*box[0]*box[0]*box[0]+box[1]*box[1]*box[1]*box[1])*
    //            sim_math_abs(lvel[2])*lvel[2]/64.0;
    B = -m->opt.density*box[2]*(box[0]*box[0]*box[0]*box[0]+box[1]*box[1]*box[1]*box[1])*
        2*sim_math_abs(lvel[2])/64.0;
    if (sim_isSparse(m)) {
      addJTBJSparse(m, d, J, &B, 1, 2, rownnz, rowadr, colind);
    } else {
      addJTBJ(m, d, J+2*nv, &B, 1);
    }

    // lfrc[3] -= 0.5*m->opt.density*box[1]*box[2]*sim_math_abs(lvel[3])*lvel[3];
    B = -0.5*m->opt.density*box[1]*box[2]*2*sim_math_abs(lvel[3]);
    if (sim_isSparse(m)) {
      addJTBJSparse(m, d, J, &B, 1, 3, rownnz, rowadr, colind);
    } else {
      addJTBJ(m, d, J+3*nv, &B, 1);
    }

    // lfrc[4] -= 0.5*m->opt.density*box[0]*box[2]*sim_math_abs(lvel[4])*lvel[4];
    B = -0.5*m->opt.density*box[0]*box[2]*2*sim_math_abs(lvel[4]);
    if (sim_isSparse(m)) {
      addJTBJSparse(m, d, J, &B, 1, 4, rownnz, rowadr, colind);
    } else {
      addJTBJ(m, d, J+4*nv, &B, 1);
    }

    // lfrc[5] -= 0.5*m->opt.density*box[0]*box[1]*sim_math_abs(lvel[5])*lvel[5];
    B = -0.5*m->opt.density*box[0]*box[1]*2*sim_math_abs(lvel[5]);
    if (sim_isSparse(m)) {
      addJTBJSparse(m, d, J, &B, 1, 5, rownnz, rowadr, colind);
    } else {
      addJTBJ(m, d, J+5*nv, &B, 1);
    }
  }

  sim_freeStack(d);
}


//------------------------- derivatives of passive forces ------------------------------------------

// add (d qfrc_passive / d qvel) to qDeriv
void SIM_d_passive_vel(const sim_model_t* m, sim_data_t* d) {
  // all disabled: nothing to add
  if (SIM_DISABLED(SIM_DSBL_SPRING) && SIM_DISABLED(SIM_DSBL_DAMPER)) {
    return;
  }

  int sleep_filter = SIM_ENABLED(SIM_ENBL_SLEEP) && d->ntree_awake < m->ntree;
  int nbody = sleep_filter ? d->nbody_awake : m->nbody;

  // fluid drag model, either body-level (inertia box) or geom-level (ellipsoid)
  if (m->opt.viscosity > 0 || m->opt.density > 0) {
    for (int b=0; b < nbody; b++) {
      int i = sleep_filter ? d->body_awake_ind[b] : b;

      if (m->body_mass[i] < SIM_MINVAL) {
        continue;
      }

      int use_ellipsoid_model = 0;
      // if any child geom uses the ellipsoid model, inertia-box model is disabled for parent body
      for (int j=0; j < m->body_geomnum[i] && use_ellipsoid_model == 0; j++) {
        const int geomid = m->body_geomadr[i] + j;
        use_ellipsoid_model += (m->geom_fluid[SIM_NFLUID*geomid] > 0);
      }
      if (use_ellipsoid_model) {
        SIM_d_ellipsoidFluid(m, d, i);
      } else {
        SIM_d_inertiaBoxFluid(m, d, i);
      }
    }
  }

  // disabled: nothing to add
  if (SIM_DISABLED(SIM_DSBL_DAMPER)) {
    return;
  }

  // dof damping
  int nv = m->nv;
  int nv_awake = sleep_filter ? d->nv_awake : nv;
  for (int j = 0; j < nv_awake; j++) {
    int i = sleep_filter ? d->dof_awake_ind[j] : j;
    d->qDeriv[m->D_rowadr[i] + m->D_diag[i]] -= m->dof_damping[i];
  }

  // flex edge damping
  for (int f=0; f < m->nflex; f++) {
    sim_scalar_t B = -m->flex_edgedamping[f];
    if (m->flex_rigid[f] || !B) {
      continue;
    }

    int flex_edgeadr = m->flex_edgeadr[f];
    int flex_edgenum = m->flex_edgenum[f];

    // process non-rigid edges of this flex
    for (int e=flex_edgeadr; e < flex_edgeadr+flex_edgenum; e++) {
      // skip rigid
      if (m->flexedge_rigid[e]) {
        continue;
      }

      // always sparse
      addJTBJSparse(m, d, d->flexedge_J, &B, 1, e,
                    m->flexedge_J_rownnz, m->flexedge_J_rowadr, m->flexedge_J_colind);
    }
  }

  // tendon damping
  int ntendon = m->ntendon;
  for (int i=0; i < ntendon; i++) {
    // skip tendon in one or two sleeping trees
    if (sleep_filter) {
      int treenum = m->tendon_treenum[i];
      int id1 = m->tendon_treeid[2*i];
      if (treenum == 1 && !d->tree_awake[id1]) continue;
      int id2 = m->tendon_treeid[2*i+1];
      if (treenum == 2 && !d->tree_awake[id1] && !d->tree_awake[id2]) continue;
    }

    sim_scalar_t B = -m->tendon_damping[i];

    if (!B) {
      continue;
    }

    // add sparse
    addJTBJSparse(m, d, d->ten_J, &B, 1, i, d->ten_J_rownnz, d->ten_J_rowadr, d->ten_J_colind);
  }
}


//------------------------- main entry points ------------------------------------------------------

// analytical derivative of smooth forces w.r.t velocities:
//   d->qDeriv = d (qfrc_actuator + qfrc_passive - [qfrc_bias]) / d qvel
void SIM_d_smooth_vel(const sim_model_t* m, sim_data_t* d, int flg_bias) {
  int sleep_filter = SIM_ENABLED(SIM_ENBL_SLEEP) && d->nv_awake < m->nv;

  // clear qDeriv
  if (!sleep_filter) {
    sim_math_zero(d->qDeriv, m->nD);
  } else {
    sim_math_zeroSparse(d->qDeriv, m->D_rownnz, m->D_rowadr, d->dof_awake_ind, d->nv_awake);
  }

  // qDeriv += d qfrc_actuator / d qvel
  SIM_d_actuator_vel(m, d);

  // qDeriv += d qfrc_passive / d qvel
  SIM_d_passive_vel(m, d);

  // qDeriv -= d qfrc_bias / d qvel; optional
  if (flg_bias) {
    SIM_d_rne_vel(m, d);
  }
}
