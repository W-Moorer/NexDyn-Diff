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

#include "engine/engine_support.h"

#include <inttypes.h>  // IWYU pragma: keep
#include <stddef.h>

#include <simcore/SIM_data.h>
#include <simcore/SIM_model.h>
#include <simcore/SIM_san.h>  // IWYU pragma: keep
#include "engine/engine_collision_convex.h"
#include "engine/engine_collision_driver.h"
#include "engine/engine_collision_gjk.h"
#include "engine/engine_collision_primitive.h"
#include "engine/engine_core_util.h"
#include "engine/engine_crossplatform.h"
#include "engine/engine_memory.h"
#include "engine/engine_memory.h"
#include "engine/engine_util_blas.h"
#include "engine/engine_util_errmem.h"
#include "engine/engine_util_misc.h"
#include "engine/engine_util_sparse.h"
#include "engine/engine_util_spatial.h"

#ifdef SIM_USEPLATFORMSIMD
  #if defined(__AVX__) && !defined(SIM_USESINGLE)
    #define SIM_USEAVX
    #include "immintrin.h"
  #endif
#endif

//-------------------------- Constants -------------------------------------------------------------

 #define SIM_VERSION 3005001
#define SIM_VERSIONSTRING "3.5.1"

// names of disable flags
const char* SIM_DISABLESTRING[SIM_NDISABLE] = {
  "Constraint",
  "Equality",
  "Frictionloss",
  "Limit",
  "Contact",
  "Spring",
  "Damper",
  "Gravity",
  "Clampctrl",
  "Warmstart",
  "Filterparent",
  "Actuation",
  "Refsafe",
  "Sensor",
  "Midphase",
  "Eulerdamp",
  "AutoReset",
  "NativeCCD",
  "Island"
};


// names of enable flags
const char* SIM_ENABLESTRING[SIM_NENABLE] = {
  "Override",
  "Energy",
  "Fwdinv",
  "InvDiscrete",
  "MultiCCD",
  "Sleep"
};


// names of timers
const char* SIM_TIMERSTRING[SIM_NTIMER]= {
  "step",
  "forward",
  "inverse",
  "position",
  "velocity",
  "actuation",
  "constraint",
  "advance",
  "pos_kinematics",
  "pos_inertia",
  "pos_collision",
  "pos_make",
  "pos_project",
  "col_broadphase",
  "col_narrowphase"
};


// size of contact data fields
const int SIM_CONDATA_SIZE[SIM_NCONDATA] = {
  1,  // SIM_CONDATA_FOUND
  3,  // SIM_CONDATA_FORCE
  3,  // SIM_CONDATA_TORQUE
  1,  // SIM_CONDATA_DIST
  3,  // SIM_CONDATA_POS
  3,  // SIM_CONDATA_NORMAL
  3   // SIM_CONDATA_TANGENT
};


// size of ray data fields
const int SIM_RAYDATA_SIZE[SIM_NRAYDATA] = {
  1,  // SIM_RAYDATA_DIST
  3,  // SIM_RAYDATA_DIR
  3,  // SIM_RAYDATA_ORIGIN
  3,  // SIM_RAYDATA_POINT
  3,  // SIM_RAYDATA_NORMAL
  1   // SIM_RAYDATA_DEPTH
};

//-------------------------- get/set state ---------------------------------------------------------

// return size of a single state element
static inline int sim_stateElemSize(const sim_model_t* m, SIM_tState sig) {
  switch (sig) {
  case SIM_STATE_TIME:          return 1;
  case SIM_STATE_QPOS:          return m->nq;
  case SIM_STATE_QVEL:          return m->nv;
  case SIM_STATE_ACT:           return m->na;
  case SIM_STATE_HISTORY:       return m->nhistory;
  case SIM_STATE_WARMSTART:     return m->nv;
  case SIM_STATE_CTRL:          return m->nu;
  case SIM_STATE_QFRC_APPLIED:  return m->nv;
  case SIM_STATE_XFRC_APPLIED:  return 6*m->nbody;
  case SIM_STATE_EQ_ACTIVE:     return m->neq;    // sim_byte_t, stored as sim_scalar_t in state vector
  case SIM_STATE_MOCAP_POS:     return 3*m->nmocap;
  case SIM_STATE_MOCAP_QUAT:    return 4*m->nmocap;
  case SIM_STATE_USERDATA:      return m->nuserdata;
  case SIM_STATE_PLUGIN:        return m->npluginstate;
  default:
    SIM_ERROR("invalid state element %u", sig);
    return 0;
  }
}


// return pointer to a single state element
static inline sim_scalar_t* sim_stateElemPtr(const sim_model_t* m, sim_data_t* d, SIM_tState sig) {
  switch (sig) {
  case SIM_STATE_TIME:          return &d->time;
  case SIM_STATE_QPOS:          return d->qpos;
  case SIM_STATE_QVEL:          return d->qvel;
  case SIM_STATE_ACT:           return d->act;
  case SIM_STATE_HISTORY:       return d->history;
  case SIM_STATE_WARMSTART:     return d->qacc_warmstart;
  case SIM_STATE_CTRL:          return d->ctrl;
  case SIM_STATE_QFRC_APPLIED:  return d->qfrc_applied;
  case SIM_STATE_XFRC_APPLIED:  return d->xfrc_applied;
  case SIM_STATE_MOCAP_POS:     return d->mocap_pos;
  case SIM_STATE_MOCAP_QUAT:    return d->mocap_quat;
  case SIM_STATE_USERDATA:      return d->userdata;
  case SIM_STATE_PLUGIN:        return d->plugin_state;
  default:
    SIM_ERROR("invalid state element %u", sig);
    return NULL;
  }
}


static inline const sim_scalar_t* sim_stateElemConstPtr(const sim_model_t* m, const sim_data_t* d, SIM_tState sig) {
  return sim_stateElemPtr(m, (sim_data_t*) d, sig);  // discard const qualifier from d
}


// get size of state signature
int sim_stateSize(const sim_model_t* m, int sig) {
  if (sig < 0) {
    SIM_ERROR("invalid state signature %d < 0", sig);
    return 0;
  }

  if (sig >= (1<<SIM_NSTATE)) {
    SIM_ERROR("invalid state signature %d >= 2^SIM_NSTATE", sig);
    return 0;
  }

  int size = 0;
  for (int i=0; i < SIM_NSTATE; i++) {
    SIM_tState element = 1<<i;
    if (element & sig) {
      size += sim_stateElemSize(m, element);
    }
  }

  return size;
}


// get state
void sim_getState(const sim_model_t* m, const sim_data_t* d, sim_scalar_t* state, int sig) {
  if (sig < 0) {
    SIM_ERROR("invalid state signature %d < 0", sig);
    return;
  }

  if (sig >= (1<<SIM_NSTATE)) {
    SIM_ERROR("invalid state signature %d >= 2^SIM_NSTATE", sig);
    return;
  }

  int adr = 0;
  for (int i=0; i < SIM_NSTATE; i++) {
    SIM_tState element = 1<<i;
    if (element & sig) {
      int size = sim_stateElemSize(m, element);

      // special handling of eq_active (sim_byte_t)
      if (element == SIM_STATE_EQ_ACTIVE) {
        int neq = m->neq;
        for (int j=0; j < neq; j++) {
          state[adr++] = d->eq_active[j];
        }
      }

      // regular state components (sim_scalar_t)
      else {
        const sim_scalar_t* ptr = sim_stateElemConstPtr(m, d, element);
        sim_math_copy(state + adr, ptr, size);
        adr += size;
      }
    }
  }
}


// extract a sub-state from a state
void sim_extractState(const sim_model_t* m, const sim_scalar_t* src, int srcsig, sim_scalar_t* dst, int dstsig) {
  if (srcsig < 0) {
    SIM_ERROR("invalid srcsig %d < 0", srcsig);
    return;
  }

  if (srcsig >= (1<<SIM_NSTATE)) {
    SIM_ERROR("invalid srcsig %d >= 2^SIM_NSTATE", srcsig);
    return;
  }

  if ((srcsig & dstsig) != dstsig) {
    SIM_ERROR("dstsig is not a subset of srcsig");
    return;
  }

  for (int i=0; i < SIM_NSTATE; i++) {
    SIM_tState element = 1<<i;
    if (element & srcsig) {
      int size = sim_stateElemSize(m, element);
      if (element & dstsig) {
        sim_math_copy(dst, src, size);
        dst += size;
      }
      src += size;
    }
  }
}


// set state
void sim_setState(const sim_model_t* m, sim_data_t* d, const sim_scalar_t* state, int sig) {
  if (sig < 0) {
    SIM_ERROR("invalid state signature %d < 0", sig);
    return;
  }

  if (sig >= (1<<SIM_NSTATE)) {
    SIM_ERROR("invalid state signature %d >= 2^SIM_NSTATE", sig);
    return;
  }

  int adr = 0;
  for (int i=0; i < SIM_NSTATE; i++) {
    SIM_tState element = 1<<i;
    if (element & sig) {
      int size = sim_stateElemSize(m, element);

      // special handling of eq_active (sim_byte_t)
      if (element == SIM_STATE_EQ_ACTIVE) {
        int neq = m->neq;
        for (int j=0; j < neq; j++) {
          d->eq_active[j] = state[adr++];
        }
      }

      // regular state components (sim_scalar_t)
      else {
        sim_scalar_t* ptr = sim_stateElemPtr(m, d, element);
        sim_math_copy(ptr, state + adr, size);
        adr += size;
      }
    }
  }
}


// copy state from src to dst
void sim_copyState(const sim_model_t* m, const sim_data_t* src, sim_data_t* dst, int sig) {
  if (sig < 0) {
    SIM_ERROR("invalid state signature %d < 0", sig);
    return;
  }

  if (sig >= (1<<SIM_NSTATE)) {
    SIM_ERROR("invalid state signature %d >= 2^SIM_NSTATE", sig);
    return;
  }

  for (int i=0; i < SIM_NSTATE; i++) {
    SIM_tState element = 1<<i;
    if (element & sig) {
      int size = sim_stateElemSize(m, element);

      // special handling of eq_active (sim_byte_t)
      if (element == SIM_STATE_EQ_ACTIVE) {
        int neq = m->neq;
        for (int j=0; j < neq; j++) {
          dst->eq_active[j] = src->eq_active[j];
        }
      }

      // regular state components (sim_scalar_t)
      else {
        sim_scalar_t* dst_ptr = sim_stateElemPtr(m, dst, element);
        const sim_scalar_t* src_ptr = sim_stateElemConstPtr(m, src, element);
        sim_math_copy(dst_ptr, src_ptr, size);
      }
    }
  }
}


// copy current state to the k-th model keyframe
void sim_setKeyframe(sim_model_t* m, const sim_data_t* d, int k) {
  // check keyframe index
  if (k >= m->nkey) {
    SIM_ERROR("index must be smaller than %" PRId64 " (keyframes allocated in model)", m->nkey);
  }
  if (k < 0) {
    SIM_ERROR("keyframe index cannot be negative");
  }

  // copy state to model keyframe
  m->key_time[k] = d->time;
  sim_math_copy(m->key_qpos + k*m->nq, d->qpos, m->nq);
  sim_math_copy(m->key_qvel + k*m->nv, d->qvel, m->nv);
  sim_math_copy(m->key_act + k*m->na, d->act, m->na);
  sim_math_copy(m->key_mpos + k*3*m->nmocap, d->mocap_pos, 3*m->nmocap);
  sim_math_copy(m->key_mquat + k*4*m->nmocap, d->mocap_quat, 4*m->nmocap);
  sim_math_copy(m->key_ctrl + k*m->nu, d->ctrl, m->nu);
}


//-------------------------- inertia functions -----------------------------------------------------

// convert sparse inertia matrix M into full matrix
void sim_fullM(const sim_model_t* m, sim_scalar_t* dst, const sim_scalar_t* M) {
  int adr = 0, nv = m->nv;
  sim_math_zero(dst, nv*nv);

  for (int i=0; i < nv; i++) {
    int j = i;
    while (j >= 0) {
      dst[i*nv+j] = M[adr];
      dst[j*nv+i] = M[adr];
      j = m->dof_parentid[j];
      adr++;
    }
  }
}


// multiply vector by inertia matrix
void sim_mulM(const sim_model_t* m, const sim_data_t* d, sim_scalar_t* res, const sim_scalar_t* vec) {
  sim_math_mulSymVecSparse(res, d->M, vec, m->nv, m->M_rownnz, m->M_rowadr, m->M_colind);
}


// multiply vector by M^(1/2)
void sim_mulM2(const sim_model_t* m, const sim_data_t* d, sim_scalar_t* res, const sim_scalar_t* vec) {
  int  nv = m->nv;
  const sim_scalar_t* qLD = d->qLD;

  sim_math_zero(res, nv);

  // res = L * vec
  for (int i=0; i < nv; i++) {
    // diagonal
    res[i] = vec[i];

    // non-simple: add off-diagonals
    if (!m->dof_simplenum[i]) {
      int adr = m->M_rowadr[i];
      res[i] += sim_math_dotSparse(qLD+adr, vec, m->M_rownnz[i] - 1, m->M_colind+adr);
    }
  }

  // res *= sqrt(D)
  for (int i=0; i < nv; i++) {
    int diag = m->M_rowadr[i] + m->M_rownnz[i] - 1;
    res[i] *= sim_math_sqrt(qLD[diag]);
  }
}


// add inertia matrix to destination matrix
//  destination can be sparse or dense when all int* are NULL
void sim_addM(const sim_model_t* m, sim_data_t* d, sim_scalar_t* dst,
             int* rownnz, int* rowadr, int* colind) {
  int nv = m->nv;
  // sparse
  if (rownnz && rowadr && colind) {
    sim_markStack(d);
    sim_scalar_t* buf_val = SIM_STACK_ALLOC(d, nv, sim_scalar_t);
    int* buf_ind = SIM_STACK_ALLOC(d, nv, int);

    sim_math_addToMatSparse(dst, rownnz, rowadr, colind, nv,
      d->M, m->M_rownnz, m->M_rowadr, m->M_colind,
      buf_val, buf_ind);

    sim_freeStack(d);
  }

  // dense
  else {
    sim_math_addToSymSparse(dst, d->M, nv, m->M_rownnz, m->M_rowadr, m->M_colind, /*flg_upper*/ 0);
  }
}


//-------------------------- perturbations ---------------------------------------------------------

// add Cartesian force and torque to qfrc_target
void sim_applyFT(const sim_model_t* m, sim_data_t* d,
                const sim_scalar_t force[3], const sim_scalar_t torque[3],
                const sim_scalar_t point[3], int body, sim_scalar_t* qfrc_target) {
  int nv = m->nv;

  // allocate local variables
  sim_markStack(d);
  sim_scalar_t* jacp = force ? SIM_STACK_ALLOC(d, 3*nv, sim_scalar_t) : NULL;
  sim_scalar_t* jacr = torque ? SIM_STACK_ALLOC(d, 3*nv, sim_scalar_t) : NULL;
  sim_scalar_t* qforce = SIM_STACK_ALLOC(d, nv, sim_scalar_t);

  // make sure body is in range
  if (body < 0 || body >= m->nbody) {
    SIM_ERROR("invalid body %d", body);
  }

  // sparse case
  if (sim_isSparse(m)) {
    // construct chain and sparse Jacobians
    int* chain = SIM_STACK_ALLOC(d, nv, int);
    int NV = sim_bodyChain(m, body, chain);
    sim_jacSparse(m, d, jacp, jacr, point, body, NV, chain);

    // compute J'*f and accumulate
    if (force) {
      sim_math_mulMatTVec(qforce, jacp, force, 3, NV);
      for (int i=0; i < NV; i++) {
        qfrc_target[chain[i]] += qforce[i];
      }
    }
    if (torque) {
      sim_math_mulMatTVec(qforce, jacr, torque, 3, NV);
      for (int i=0; i < NV; i++) {
        qfrc_target[chain[i]] += qforce[i];
      }
    }
  }

  // dense case
  else {
    // compute Jacobians
    sim_jac(m, d, jacp, jacr, point, body);

    // compute J'*f and accumulate
    if (force) {
      sim_math_mulMatTVec(qforce, jacp, force, 3, nv);
      sim_math_addTo(qfrc_target, qforce, nv);
    }
    if (torque) {
      sim_math_mulMatTVec(qforce, jacr, torque, 3, nv);
      sim_math_addTo(qfrc_target, qforce, nv);
    }
  }

  sim_freeStack(d);
}


// accumulate xfrc_applied in qfrc
void sim_xfrcAccumulate(const sim_model_t* m, sim_data_t* d, sim_scalar_t* qfrc) {
  int nbody = m->nbody;
  const sim_scalar_t *xfrc = d->xfrc_applied;

  // quick return if identically zero (efficient memcmp implementation)
  if (sim_math_isZeroByte((const unsigned char*)(xfrc+6), 6*(nbody-1)*sizeof(sim_scalar_t))) {
    return;
  }

  // some non-zero wrenches, apply them
  for (int i=1; i < nbody; i++) {
    if (!sim_math_isZero(xfrc+6*i, 6)) {
      sim_applyFT(m, d, xfrc+6*i, xfrc+6*i+3, d->xipos+3*i, i, qfrc);
    }
  }
}



//-------------------------- miscellaneous ---------------------------------------------------------

// returns the smallest distance between two geoms (using nativeccd)
static sim_scalar_t sim_geomDistanceCCD(const sim_model_t* m, const sim_data_t* d, int g1, int g2,
                                 sim_scalar_t distmax, sim_scalar_t fromto[6]) {
  SIM_CCDConfig config;
  SIM_CCDStatus status;

  // set config
  config.max_iterations = m->opt.ccd_iterations;
  config.tolerance = m->opt.ccd_tolerance;
  config.max_contacts = 1;        // want contacts
  config.dist_cutoff = distmax;   // want geom distances

  sim_collision_ccd_object_t obj1, obj2;
  SIM_c_initCCDObj(&obj1, m, d, g1, 0);
  SIM_c_initCCDObj(&obj2, m, d, g2, 0);

  sim_scalar_t dist = SIM_c_ccd(&config, &status, &obj1, &obj2);

  // witness points are only computed if dist <= distmax
  if (fromto && status.nx > 0) {
    sim_math_copy_3(fromto, status.x1);
    sim_math_copy_3(fromto+3, status.x2);
  }

  // clamp dist to distmax as SIM_c_ccd returns DBL_MAX if dist > distmax
  return dist < distmax ? dist : distmax;
}


// returns the smallest distance between two geoms
sim_scalar_t sim_geomDistance(const sim_model_t* m, const sim_data_t* d, int geom1, int geom2, sim_scalar_t distmax,
                       sim_scalar_t fromto[6]) {
  sim_contact_t con[SIM_MAXCONPAIR];
  sim_scalar_t dist = distmax;
  if (fromto) sim_math_zero(fromto, 6);

  // flip geom order if required
  int flip = m->geom_type[geom1] > m->geom_type[geom2];
  int g1 = flip ? geom2 : geom1;
  int g2 = flip ? geom1 : geom2;
  int type1 = m->geom_type[g1];
  int type2 = m->geom_type[g2];

  SIM_fCollision func = SIM_COLLISIONFUNC[type1][type2];

  // call collision function if it exists
  if (!func) {
    return dist;
  }

  // use nativeccd if flag is enabled
  if (!SIM_DISABLED(SIM_DSBL_NATIVECCD)) {
    if (func == SIM_c_Convex || func == SIM_c_BoxBox) {
      return sim_geomDistanceCCD(m, d, geom1, geom2, distmax, fromto);
    }
  }

  // call collision function with distmax as margin
  int num = func(m, d, con, g1, g2, distmax);

  // find smallest distance
  int smallest = -1;
  for (int i=0; i < num; i++) {
    sim_scalar_t dist_i = con[i].dist;
    if (dist_i < dist) {
      dist = dist_i;
      smallest = i;
    }
  }

  // write fromto if given and a collision has been found
  if (fromto && smallest >= 0) {
    sim_scalar_t sign = flip ? -1 : 1;
    sim_math_addScl3(fromto+0, con[smallest].pos, con[smallest].frame, -0.5*sign*dist);
    sim_math_addScl3(fromto+3, con[smallest].pos, con[smallest].frame, 0.5*sign*dist);
  }

  return dist;
}


// compute velocity by finite-differencing two positions
void sim_differentiatePos(const sim_model_t* m, sim_scalar_t* qvel, sim_scalar_t dt,
                         const sim_scalar_t* qpos1, const sim_scalar_t* qpos2) {
  // loop over joints
  for (int j=0; j < m->njnt; j++) {
    // get addresses in qpos and qvel
    int padr = m->jnt_qposadr[j];
    int vadr = m->jnt_dofadr[j];

    switch ((SIM_tJoint) m->jnt_type[j]) {
    case SIM_JNT_FREE:
      for (int i=0; i < 3; i++) {
        qvel[vadr+i] = (qpos2[padr+i] - qpos1[padr+i]) / dt;
      }
      vadr += 3;
      padr += 3;

      // continue with rotations
      SIM_FALLTHROUGH;

    case SIM_JNT_BALL:
      // solve:  qpos1 * quat(qvel * dt) = qpos2
      sim_math_subQuat(qvel+vadr, qpos2+padr, qpos1+padr);
      sim_math_scale_3(qvel+vadr, qvel+vadr, 1/dt);
      break;

    case SIM_JNT_HINGE:
    case SIM_JNT_SLIDE:
      qvel[vadr] = (qpos2[padr] - qpos1[padr]) / dt;
    }
  }
}


// integrate qpos with given qvel for given body indices
void sim_integratePosInd(const sim_model_t* m, sim_scalar_t* qpos, const sim_scalar_t* qvel, sim_scalar_t dt,
                        const int* index, int nbody) {
  for (int b=1; b < nbody; b++) {
    int k = index ? index[b] : b;
    int start = m->body_jntadr[k];
    int end = start + m->body_jntnum[k];
    for (int j=start; j < end; j++) {
      // get addresses in qpos and qvel
      int padr = m->jnt_qposadr[j];
      int vadr = m->jnt_dofadr[j];

      switch ((SIM_tJoint) m->jnt_type[j]) {
      case SIM_JNT_FREE:
        // position update
        for (int i=0; i < 3; i++) {
          qpos[padr+i] += dt * qvel[vadr+i];
        }
        padr += 3;
        vadr += 3;

        // continue with rotation update
        SIM_FALLTHROUGH;

      case SIM_JNT_BALL:
        // quaternion update
        sim_math_quatIntegrate(qpos+padr, qvel+vadr, dt);
        break;

      case SIM_JNT_HINGE:
      case SIM_JNT_SLIDE:
        // scalar update: same for rotation and translation
        qpos[padr] += dt * qvel[vadr];
      }
    }
  }
}


// integrate qpos with given qvel
void sim_integratePos(const sim_model_t* m, sim_scalar_t* qpos, const sim_scalar_t* qvel, sim_scalar_t dt) {
  sim_integratePosInd(m, qpos, qvel, dt, NULL, m->nbody);
}


// normalize all quaternions in qpos-type vector
void sim_normalizeQuat(const sim_model_t* m, sim_scalar_t* qpos) {
  // find quaternion fields and normalize
  for (int i=0; i < m->njnt; i++) {
    if (m->jnt_type[i] == SIM_JNT_BALL || m->jnt_type[i] == SIM_JNT_FREE) {
      sim_math_normalize4(qpos+m->jnt_qposadr[i]+3*(m->jnt_type[i] == SIM_JNT_FREE));
    }
  }
}


// return 1 if actuator i is disabled, 0 otherwise
int sim_actuatorDisabled(const sim_model_t* m, int i) {
  int group = m->actuator_group[i];
  if (group < 0 || group > 30) {
    return 0;
  } else {
    return m->opt.disableactuator & (1 << group) ? 1 : 0;
  }
}


// returns the next activation given current act_dot, after clamping
sim_scalar_t sim_nextActivation(const sim_model_t* m, const sim_data_t* d,
                         int actuator_id, int act_adr, sim_scalar_t act_dot) {
  sim_scalar_t act = d->act[act_adr];

  if (m->actuator_dyntype[actuator_id] == SIM_DYN_FILTEREXACT) {
    // exact filter integration
    // act_dot(0) = (ctrl-act(0)) / tau
    // act(h) = act(0) + (ctrl-act(0)) (1 - exp(-h / tau))
    //        = act(0) + act_dot(0) * tau * (1 - exp(-h / tau))
    sim_scalar_t tau = sim_math_max(SIM_MINVAL, m->actuator_dynprm[actuator_id*SIM_NDYN]);
    act = act + act_dot * tau * (1 - sim_math_exp(-m->opt.timestep / tau));
  } else {
    // Euler integration
    act = act + act_dot * m->opt.timestep;
  }

  // clamp to actrange
  if (m->actuator_actlimited[actuator_id]) {
    sim_scalar_t* actrange = m->actuator_actrange + 2*actuator_id;
    act = sim_math_clip(act, actrange[0], actrange[1]);
  }

  return act;
}


// sum all body masses
sim_scalar_t sim_getTotalmass(const sim_model_t* m) {
  sim_scalar_t res = 0;

  for (int i=1; i < m->nbody; i++) {
    res += m->body_mass[i];
  }

  return res;
}


// scale all body masses and inertias to achieve specified total mass
void sim_setTotalmass(sim_model_t* m, sim_scalar_t newmass) {
  // compute scale factor, avoid zeros
  sim_scalar_t scale = sim_math_max(SIM_MINVAL, newmass / sim_math_max(SIM_MINVAL, sim_getTotalmass(m)));

  // scale all masses and inertias
  for (int i=1; i < m->nbody; i++) {
    m->body_mass[i] *= scale;
    m->body_inertia[3*i] *= scale;
    m->body_inertia[3*i+1] *= scale;
    m->body_inertia[3*i+2] *= scale;
  }

  // don't forget to call sim_set0 after changing masses
}


// version number
int sim_version(void) {
  return SIM_VERSION;
}


// current version of SimCore as a null-terminated string
const char* sim_versionString(void) {
  static const char versionstring[] = SIM_VERSIONSTRING;
  return versionstring;
}


// return total size of data in a contact sensor bitfield specification
int sim_math_condataSize(int dataspec) {
  int size = 0;
  for (int i=0; i < SIM_NCONDATA; i++) {
    if (dataspec & (1 << i)) {
      size += SIM_CONDATA_SIZE[i];
    }
  }
  return size;
}


// return total size of data in a rangefinder sensor bitfield specification
int sim_math_raydataSize(int dataspec) {
  int size = 0;
  for (int i=0; i < SIM_NRAYDATA; i++) {
    if (dataspec & (1 << i)) {
      size += SIM_RAYDATA_SIZE[i];
    }
  }
  return size;
}


// compute camera pixel parameters from model, output are:
//   pixel units: fx, fy (focal lengths), cx, cy (principal point)
//   length units: extent
void sim_math_camIntrinsics(const sim_model_t* m, int camid,
                       sim_scalar_t* fx, sim_scalar_t* fy, sim_scalar_t* cx, sim_scalar_t* cy, sim_scalar_t* extent) {
  const int width = m->cam_resolution[2*camid];
  const int height = m->cam_resolution[2*camid+1];
  const float* sensorsize = m->cam_sensorsize + 2*camid;
  const float* intrinsic = m->cam_intrinsic + 4*camid;
  const SIM_tProjection projection = (SIM_tProjection)m->cam_projection[camid];

  switch (projection) {
  case SIM_PROJ_PERSPECTIVE:
    if (sensorsize[0] && sensorsize[1]) {
      // intrinsic-based perspective camera
      *fx = intrinsic[0] / sensorsize[0] * width;
      *fy = intrinsic[1] / sensorsize[1] * height;
      *cx = intrinsic[2] / sensorsize[0] * width;
      *cy = intrinsic[3] / sensorsize[1] * height;
    } else {
      // fovy-based perspective camera
      *fx = *fy = 0.5 / sim_math_tan(m->cam_fovy[camid] * SIM_PI / 360.0) * height;
      *cx = (sim_scalar_t)width / 2.0;
      *cy = (sim_scalar_t)height / 2.0;
    }
    break;
  case SIM_PROJ_ORTHOGRAPHIC:
    // orthographic: normalize pixel offset to [-1, 1]
    *fx = (sim_scalar_t)width / 2.0;
    *fy = (sim_scalar_t)height / 2.0;
    *cx = *fx;
    *cy = *fy;
    break;
  }

  // extent only used for orthographic cameras
  *extent = m->cam_fovy[camid];
}


// read delayed ctrl value for actuator at given time
sim_scalar_t sim_readCtrl(const sim_model_t* m, const sim_data_t* d, int id, sim_scalar_t time, int interp) {
  // validate actuator id
  if (id < 0 || id >= m->nu) {
    SIM_ERROR("invalid actuator id %d", id);
    return 0;
  }

  // no delay: return current ctrl value
  int nsample = m->actuator_history[2*id];
  if (nsample == 0) {
    return d->ctrl[id];
  }

  // resolve interpolation order: use model's interp if argument is -1
  if (interp < 0) interp = m->actuator_history[2*id+1];

  // get buffer pointer and read from history buffer
  sim_scalar_t delay = m->actuator_delay[id];
  const sim_scalar_t* buf = d->history + m->actuator_historyadr[id];
  sim_scalar_t res;
  const sim_scalar_t* ptr = sim_math_historyRead(buf, nsample, /*dim=*/1, &res, time - delay, interp);
  return ptr ? *ptr : res;
}


// read sensor value from history buffer at given time
const sim_scalar_t* sim_readSensor(const sim_model_t* m, const sim_data_t* d, int id, sim_scalar_t time,
                            sim_scalar_t* result, int interp) {
  // validate sensor id
  if (id < 0 || id >= m->nsensor) {
    SIM_ERROR("invalid sensor id %d", id);
    return NULL;
  }

  // no history: return current sensor value
  int nsample = m->sensor_history[2*id];
  if (nsample == 0) {
    return d->sensordata + m->sensor_adr[id];
  }

  // resolve interpolation order: use model's interp if argument is -1
  if (interp < 0) interp = m->sensor_history[2*id+1];

  // get buffer pointer and read from history buffer
  int dim = m->sensor_dim[id];
  sim_scalar_t delay = m->sensor_delay[id];
  const sim_scalar_t* buf = d->history + m->sensor_historyadr[id];
  return sim_math_historyRead(buf, nsample, dim, result, time - delay, interp);
}


// initialize history buffer for actuator
void sim_initCtrlHistory(const sim_model_t* m, sim_data_t* d, int id,
                        const sim_scalar_t* times, const sim_scalar_t* values) {
  // validate actuator id
  if (id < 0 || id >= m->nu) {
    SIM_ERROR("invalid actuator id %d", id);
    return;
  }

  // check that actuator has a history buffer
  int nsample = m->actuator_history[2*id];
  if (nsample == 0) {
    SIM_ERROR("actuator %d has no history buffer", id);
    return;
  }

  // get buffer pointer
  sim_scalar_t* buf = d->history + m->actuator_historyadr[id];

  // if times is NULL, use existing buffer times
  const sim_scalar_t* buf_times = times ? times : buf + 2;

  // get existing user value (preserve it)
  sim_scalar_t user = buf[0];

  // initialize history buffer
  sim_math_historyInit(buf, nsample, 1, buf_times, values, user);
}


// initialize history buffer for sensor
void sim_initSensorHistory(const sim_model_t* m, sim_data_t* d, int id,
                          const sim_scalar_t* times, const sim_scalar_t* values, sim_scalar_t phase) {
  // validate sensor id
  if (id < 0 || id >= m->nsensor) {
    SIM_ERROR("invalid sensor id %d", id);
    return;
  }

  // check that sensor has a history buffer
  int nsample = m->sensor_history[2*id];
  if (nsample == 0) {
    SIM_ERROR("sensor %d has no history buffer", id);
    return;
  }

  // get buffer pointer and dimension
  sim_scalar_t* buf = d->history + m->sensor_historyadr[id];
  int dim = m->sensor_dim[id];

  // if times is NULL, use existing buffer times
  const sim_scalar_t* buf_times = times ? times : buf + 2;

  // initialize history buffer with provided phase
  sim_math_historyInit(buf, nsample, dim, buf_times, values, phase);
}
