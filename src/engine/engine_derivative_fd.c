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

#include "engine/engine_derivative_fd.h"

#include <stddef.h>

#include <simcore/SIM_data.h>
#include <simcore/SIM_macro.h>
#include <simcore/SIM_model.h>
#include <simcore/SIM_san.h>  // IWYU pragma: keep
#include "engine/engine_forward.h"
#include "engine/engine_inverse.h"
#include "engine/engine_memory.h"
#include "engine/engine_macro.h"
#include "engine/engine_support.h"
#include "engine/engine_util_blas.h"
#include "engine/engine_util_errmem.h"
#include "engine/engine_util_misc.h"



//--------------------------- finite-differencing utility functions --------------------------------

// get state=[qpos; qvel; act] and optionally sensordata
static void getState(const sim_model_t* m, const sim_data_t* d, sim_scalar_t* state, sim_scalar_t* sensordata) {
  sim_getState(m, d, state, SIM_STATE_PHYSICS);
  if (sensordata) {
    sim_math_copy(sensordata, d->sensordata, m->nsensordata);
  }
}


// dx = (x2 - x1) / h
static void diff(sim_scalar_t* restrict dx, const sim_scalar_t* x1, const sim_scalar_t* x2, sim_scalar_t h, int n) {
  sim_scalar_t inv_h = 1/h;
  for (int i=0; i < n; i++) {
    dx[i] = inv_h * (x2[i] - x1[i]);
  }
}


// finite-difference two state vectors ds = (s2 - s1) / h
static void stateDiff(const sim_model_t* m, sim_scalar_t* ds, const sim_scalar_t* s1, const sim_scalar_t* s2, sim_scalar_t h) {
  int nq = m->nq, nv = m->nv, na = m->na;

  if (nq == nv) {
    diff(ds, s1, s2, h, nq+nv+na);
  } else {
    sim_differentiatePos(m, ds, h, s1, s2);
    diff(ds+nv, s1+nq, s2+nq, h, nv+na);
  }
}


// finite-difference two vectors, forward, backward or centered
static void clampedDiff(sim_scalar_t* dx, const sim_scalar_t* x, const sim_scalar_t* x_plus, const sim_scalar_t* x_minus,
                        sim_scalar_t h, int nx) {
  if (x_plus && !x_minus) {
    // forward differencing
    diff(dx, x, x_plus, h, nx);
  } else if (!x_plus && x_minus) {
    // backward differencing
    diff(dx, x_minus, x, h, nx);
  } else if (x_plus && x_minus) {
    // centered differencing
    diff(dx, x_plus, x_minus, 2*h, nx);
  } else {
    // differencing failed, write zeros
    sim_math_zero(dx, nx);
  }
}


// finite-difference two state vectors, forward, backward or centered
static void clampedStateDiff(const sim_model_t* m, sim_scalar_t* ds, const sim_scalar_t* s, const sim_scalar_t* s_plus,
                             const sim_scalar_t* s_minus, sim_scalar_t h) {
  if (s_plus && !s_minus) {
    // forward differencing
    stateDiff(m, ds, s, s_plus, h);
  } else if (!s_plus && s_minus) {
    // backward differencing
    stateDiff(m, ds, s_minus, s, h);
  } else if (s_plus && s_minus) {
    // centered differencing
    stateDiff(m, ds, s_minus, s_plus, 2*h);
  } else {
    // differencing failed, write zeros
    sim_math_zero(ds, 2*m->nv + m->na);
  }
}


// check if two numbers are inside a given range
static int inRange(const sim_scalar_t x1, const sim_scalar_t x2, const sim_scalar_t* range) {
  return x1 >= range[0] && x1 <= range[1] &&
         x2 >= range[0] && x2 <= range[1];
}


// advance simulation using control callback, skipstage is SIM_tStage
void sim_stepSkip(const sim_model_t* m, sim_data_t* d, int skipstage, int skipsensor) {
  TM_START;

  // common to all integrators
  sim_checkPos(m, d);
  sim_checkVel(m, d);
  sim_forwardSkip(m, d, skipstage, skipsensor);
  sim_checkAcc(m, d);

  // compare forward and inverse solutions if enabled
  if (SIM_ENABLED(SIM_ENBL_FWDINV)) {
    sim_compareFwdInv(m, d);
  }

  // use selected integrator
  switch ((SIM_tIntegrator) m->opt.integrator) {
  case SIM_INT_EULER:
    sim_EulerSkip(m, d, skipstage >= SIM_STAGE_POS);
    break;

  case SIM_INT_RK4:
    // ignore skipstage
    sim_RungeKutta(m, d, 4);
    break;

  case SIM_INT_IMPLICIT:
  case SIM_INT_IMPLICITFAST:
    sim_implicitSkip(m, d, skipstage >= SIM_STAGE_VEL);
    break;

  default:
    SIM_ERROR("invalid integrator");
  }

  TM_END(SIM_TIMER_STEP);
}


// compute qfrc_inverse, optionally subtracting qfrc_actuator
static void inverseSkip(const sim_model_t* m, sim_data_t* d, SIM_tStage stage, int skipsensor,
                        int flg_actuation, sim_scalar_t* force) {
  sim_inverseSkip(m, d, stage, skipsensor);
  sim_math_copy(force, d->qfrc_inverse, m->nv);
  if (flg_actuation) {
    sim_fwdActuation(m, d);
    sim_math_subFrom(force, d->qfrc_actuator, m->nv);
  }
}


//------------------------- derivatives of passive forces ------------------------------------------

// add forward fin-diff approximation of (d qfrc_passive / d qvel) to qDeriv
void SIM_d_passive_velFD(const sim_model_t* m, sim_data_t* d, sim_scalar_t eps) {
  int nv = m->nv;

  sim_markStack(d);
  sim_scalar_t* qfrc_passive = SIM_STACK_ALLOC(d, nv, sim_scalar_t);
  sim_scalar_t* fd = SIM_STACK_ALLOC(d, nv, sim_scalar_t);
  int* cnt = SIM_STACK_ALLOC(d, nv, int);

  // clear row counters
  sim_math_zeroInt(cnt, nv);

  // save qfrc_passive, assume sim_fwdVelocity was called
  sim_math_copy(qfrc_passive, d->qfrc_passive, nv);

  // loop over dofs
  for (int i=0; i < nv; i++) {
    // save qvel[i]
    sim_scalar_t saveqvel = d->qvel[i];

    // eval at qvel[i]+eps
    d->qvel[i] = saveqvel + eps;
    sim_fwdVelocity(m, d);

    // restore qvel[i]
    d->qvel[i] = saveqvel;

    // finite difference result in fd
    sim_math_sub(fd, d->qfrc_passive, qfrc_passive, nv);
    sim_math_scl(fd, fd, 1/eps, nv);

    // copy to i-th column of qDeriv
    for (int j=0; j < nv; j++) {
      int adr = m->D_rowadr[j] + cnt[j];
      if (cnt[j] < m->D_rownnz[j] && m->D_colind[adr] == i) {
        d->qDeriv[adr] = fd[j];
        cnt[j]++;
      }
    }
  }

  // restore
  sim_fwdVelocity(m, d);

  sim_freeStack(d);
}


//-------------------- derivatives of all smooth (unconstrained) forces ----------------------------

// centered finite difference approximation to SIM_d_smooth_vel
void SIM_d_smooth_velFD(const sim_model_t* m, sim_data_t* d, sim_scalar_t eps) {
  int nv = m->nv;

  sim_markStack(d);
  sim_scalar_t* plus = SIM_STACK_ALLOC(d, nv, sim_scalar_t);
  sim_scalar_t* minus = SIM_STACK_ALLOC(d, nv, sim_scalar_t);
  sim_scalar_t* fd = SIM_STACK_ALLOC(d, nv, sim_scalar_t);
  int* cnt = SIM_STACK_ALLOC(d, nv, int);

  // clear row counters
  sim_math_zeroInt(cnt, nv);

  // loop over dofs
  for (int i=0; i < nv; i++) {
    // save qvel[i]
    sim_scalar_t saveqvel = d->qvel[i];

    // eval at qvel[i]+eps
    d->qvel[i] = saveqvel + eps;
    sim_fwdVelocity(m, d);
    sim_fwdActuation(m, d);
    sim_math_add(plus, d->qfrc_actuator, d->qfrc_passive, nv);
    sim_math_subFrom(plus, d->qfrc_bias, nv);

    // eval at qvel[i]-eps
    d->qvel[i] = saveqvel - eps;
    sim_fwdVelocity(m, d);
    sim_fwdActuation(m, d);
    sim_math_add(minus, d->qfrc_actuator, d->qfrc_passive, nv);
    sim_math_subFrom(minus, d->qfrc_bias, nv);

    // restore qvel[i]
    d->qvel[i] = saveqvel;

    // finite difference result in fd
    sim_math_sub(fd, plus, minus, nv);
    sim_math_scl(fd, fd, 0.5/eps, nv);

    // copy to sparse qDeriv
    for (int j=0; j < nv; j++) {
      if (cnt[j] < m->D_rownnz[j] && m->D_colind[m->D_rowadr[j]+cnt[j]] == i) {
        d->qDeriv[m->D_rowadr[j]+cnt[j]] = fd[j];
        cnt[j]++;
      }
    }
  }

  // make sure final row counters equal rownnz
  for (int i=0; i < nv; i++) {
    if (cnt[i] != m->D_rownnz[i]) {
      SIM_ERROR("error in constructing FD sparse derivative");
    }
  }

  // restore
  sim_fwdVelocity(m, d);
  sim_fwdActuation(m, d);

  sim_freeStack(d);
}


//------------------------- main entry points ------------------------------------------------------


// finite differenced Jacobian of  (next_state, sensors) = sim_step(state, control)
//   all outputs are optional
//   output dimensions (transposed w.r.t Control Theory convention):
//     DyDq: (nv x 2*nv+na)
//     DyDv: (nv x 2*nv+na)
//     DyDa: (na x 2*nv+na)
//     DyDu: (nu x 2*nv+na)
//     DsDq: (nv x nsensordata)
//     DsDv: (nv x nsensordata)
//     DsDa: (na x nsensordata)
//     DsDu: (nu x nsensordata)
//   single-letter shortcuts:
//     inputs: q=qpos, v=qvel, a=act, u=ctrl
//     outputs: y=next_state (concatenated next qpos, qvel, act), s=sensordata
void SIM_d_stepFD(const sim_model_t* m, sim_data_t* d, sim_scalar_t eps, sim_byte_t flg_centered,
                sim_scalar_t* DyDq, sim_scalar_t* DyDv, sim_scalar_t* DyDa, sim_scalar_t* DyDu,
                sim_scalar_t* DsDq, sim_scalar_t* DsDv, sim_scalar_t* DsDa, sim_scalar_t* DsDu) {
  if (m->nhistory) {
    SIM_ERROR("delays are not supported");
  }

  int nq = m->nq, nv = m->nv, na = m->na, nu = m->nu, ns = m->nsensordata;
  int ndx = 2*nv+na;  // row length of Dy Jacobians
  sim_markStack(d);

  // state to restore after finite differencing
  unsigned int restore_spec = SIM_STATE_FULLPHYSICS | SIM_STATE_CTRL;
  restore_spec |= SIM_DISABLED(SIM_DSBL_WARMSTART) ? 0 : SIM_STATE_WARMSTART;

  sim_scalar_t *fullstate  = SIM_STACK_ALLOC(d, sim_stateSize(m, restore_spec), sim_scalar_t);
  sim_scalar_t *state      = SIM_STACK_ALLOC(d, nq+nv+na, sim_scalar_t);  // current state
  sim_scalar_t *next       = SIM_STACK_ALLOC(d, nq+nv+na, sim_scalar_t);  // next state
  sim_scalar_t *next_plus  = SIM_STACK_ALLOC(d, nq+nv+na, sim_scalar_t);  // forward-nudged next state
  sim_scalar_t *next_minus = SIM_STACK_ALLOC(d, nq+nv+na, sim_scalar_t);  // backward-nudged next state

  // sensors
  int skipsensor = !DsDq && !DsDv && !DsDa && !DsDu;
  sim_scalar_t *sensor       = skipsensor ? NULL : SIM_STACK_ALLOC(d, ns, sim_scalar_t);  // sensor values
  sim_scalar_t *sensor_plus  = skipsensor ? NULL : SIM_STACK_ALLOC(d, ns, sim_scalar_t);  // forward-nudged
  sim_scalar_t *sensor_minus = skipsensor ? NULL : SIM_STACK_ALLOC(d, ns, sim_scalar_t);  // backward-nudged

  // controls
  sim_scalar_t *ctrl = SIM_STACK_ALLOC(d, nu, sim_scalar_t);

  // save current inputs
  sim_getState(m, d, fullstate, restore_spec);
  sim_math_copy(ctrl, d->ctrl, nu);
  getState(m, d, state, NULL);

  // step input
  sim_stepSkip(m, d, SIM_STAGE_NONE, skipsensor);

  // save output
  getState(m, d, next, sensor);

  // restore input
  sim_setState(m, d, fullstate, restore_spec);

  // finite-difference controls: skip=SIM_STAGE_VEL, handle ctrl at range limits
  if (DyDu || DsDu) {
    for (int i=0; i < nu; i++) {
      int limited = m->actuator_ctrllimited[i];
      // nudge forward, if possible given ctrlrange
      int nudge_fwd = !limited || inRange(ctrl[i], ctrl[i]+eps, m->actuator_ctrlrange+2*i);
      if (nudge_fwd) {
        // nudge forward
        d->ctrl[i] += eps;

        // step, get nudged output
        sim_stepSkip(m, d, SIM_STAGE_VEL, skipsensor);
        getState(m, d, next_plus, sensor_plus);

        // reset
        sim_setState(m, d, fullstate, restore_spec);
      }

      // nudge backward, if possible given ctrlrange
      int nudge_back = (flg_centered || !nudge_fwd) &&
                       (!limited || inRange(ctrl[i]-eps, ctrl[i], m->actuator_ctrlrange+2*i));
      if (nudge_back) {
        // nudge backward
        d->ctrl[i] -= eps;

        // step, get nudged output
        sim_stepSkip(m, d, SIM_STAGE_VEL, skipsensor);
        getState(m, d, next_minus, sensor_minus);

        // reset
        sim_setState(m, d, fullstate, restore_spec);
      }

      // difference states
      if (DyDu) {
        clampedStateDiff(m, DyDu+i*ndx, next, nudge_fwd ? next_plus : NULL,
                         nudge_back ? next_minus : NULL, eps);
      }

      // difference sensors
      if (DsDu) {
        clampedDiff(DsDu+i*ns, sensor, nudge_fwd ? sensor_plus : NULL,
                    nudge_back ? sensor_minus : NULL, eps, ns);
      }
    }
  }

  // finite-difference activations: skip=SIM_STAGE_VEL
  if (DyDa || DsDa) {
    for (int i=0; i < na; i++) {
      // nudge forward
      d->act[i] += eps;

      // step, get nudged output
      sim_stepSkip(m, d, SIM_STAGE_VEL, skipsensor);
      getState(m, d, next_plus, sensor_plus);

      // reset
      sim_setState(m, d, fullstate, restore_spec);

      // nudge backward
      if (flg_centered) {
        // nudge backward
        d->act[i] -= eps;

        // step, get nudged output
        sim_stepSkip(m, d, SIM_STAGE_VEL, skipsensor);
        getState(m, d, next_minus, sensor_minus);

        // reset
        sim_setState(m, d, fullstate, restore_spec);
      }

      // difference states
      if (DyDa) {
        if (!flg_centered) {
          stateDiff(m, DyDa+i*ndx, next, next_plus, eps);
        } else {
          stateDiff(m, DyDa+i*ndx, next_minus, next_plus, 2*eps);
        }
      }

      // difference sensors
      if (DsDa) {
        if (!flg_centered) {
          diff(DsDa+i*ns, sensor, sensor_plus, eps, ns);
        } else {
          diff(DsDa+i*ns, sensor_minus, sensor_plus, 2*eps, ns);
        }
      }
    }
  }


  // finite-difference velocities: skip=SIM_STAGE_POS
  if (DyDv || DsDv) {
    for (int i=0; i < nv; i++) {
      // nudge forward
      d->qvel[i] += eps;

      // step, get nudged output
      sim_stepSkip(m, d, SIM_STAGE_POS, skipsensor);
      getState(m, d, next_plus, sensor_plus);

      // reset
      sim_setState(m, d, fullstate, restore_spec);

      // nudge backward
      if (flg_centered) {
        // nudge
        d->qvel[i] -= eps;

        // step, get nudged output
        sim_stepSkip(m, d, SIM_STAGE_POS, skipsensor);
        getState(m, d, next_minus, sensor_minus);

        // reset
        sim_setState(m, d, fullstate, restore_spec);
      }

      // difference states
      if (DyDv) {
        if (!flg_centered) {
          stateDiff(m, DyDv+i*ndx, next, next_plus, eps);
        } else {
          stateDiff(m, DyDv+i*ndx, next_minus, next_plus, 2*eps);
        }
      }

      // difference sensors
      if (DsDv) {
        if (!flg_centered) {
          diff(DsDv+i*ns, sensor, sensor_plus, eps, ns);
        } else {
          diff(DsDv+i*ns, sensor_minus, sensor_plus, 2*eps, ns);
        }
      }
    }
  }

  // finite-difference positions: skip=SIM_STAGE_NONE
  if (DyDq || DsDq) {
    sim_scalar_t *dpos  = SIM_STACK_ALLOC(d, nv, sim_scalar_t);  // allocate position perturbation
    for (int i=0; i < nv; i++) {
      // nudge forward
      sim_math_zero(dpos, nv);
      dpos[i] = 1;
      sim_integratePos(m, d->qpos, dpos, eps);

      // step, get nudged output
      sim_stepSkip(m, d, SIM_STAGE_NONE, skipsensor);
      getState(m, d, next_plus, sensor_plus);

      // reset
      sim_setState(m, d, fullstate, restore_spec);

      // nudge backward
      if (flg_centered) {
        // nudge backward
        sim_math_zero(dpos, nv);
        dpos[i] = 1;
        sim_integratePos(m, d->qpos, dpos, -eps);

        // step, get nudged output
        sim_stepSkip(m, d, SIM_STAGE_NONE, skipsensor);
        getState(m, d, next_minus, sensor_minus);

        // reset
        sim_setState(m, d, fullstate, restore_spec);
      }

      // difference states
      if (DyDq) {
        if (!flg_centered) {
          stateDiff(m, DyDq+i*ndx, next, next_plus, eps);
        } else {
          stateDiff(m, DyDq+i*ndx, next_minus, next_plus, 2*eps);
        }
      }

      // difference sensors
      if (DsDq) {
        if (!flg_centered) {
          diff(DsDq+i*ns, sensor, sensor_plus, eps, ns);
        } else {
          diff(DsDq+i*ns, sensor_minus, sensor_plus, 2*eps, ns);
        }
      }
    }
  }

  sim_freeStack(d);
}


// finite differenced transition matrices (control theory notation)
//   d(x_next) = A*dx + B*du
//   d(sensor) = C*dx + D*du
//   required output matrix dimensions:
//      A: (2*nv+na x 2*nv+na)
//      B: (2*nv+na x nu)
//      C: (nsensordata x 2*nv+na)
//      D: (nsensordata x nu)
void SIM_d_transitionFD(const sim_model_t* m, sim_data_t* d, sim_scalar_t eps, sim_byte_t flg_centered,
                      sim_scalar_t* A, sim_scalar_t* B, sim_scalar_t* C, sim_scalar_t* D) {
  if (m->opt.integrator == SIM_INT_RK4) {
    SIM_ERROR("RK4 integrator is not supported");
  }
  if (m->nhistory) {
    SIM_ERROR("delays are not supported");
  }

  int nv = m->nv, na = m->na, nu = m->nu, ns = m->nsensordata;
  int ndx = 2*nv+na;  // row length of state Jacobians

  // stepFD() offset pointers, initialised to NULL
  sim_scalar_t *DyDq, *DyDv, *DyDa, *DsDq, *DsDv, *DsDa;
  DyDq = DyDv = DyDa = DsDq = DsDv = DsDa = NULL;

  sim_markStack(d);

  // allocate transposed matrices
  sim_scalar_t *AT = A ? SIM_STACK_ALLOC(d, ndx*ndx, sim_scalar_t) : NULL;  // state-transition     (transposed)
  sim_scalar_t *BT = B ? SIM_STACK_ALLOC(d, nu*ndx, sim_scalar_t) : NULL;   // control-transition   (transposed)
  sim_scalar_t *CT = C ? SIM_STACK_ALLOC(d, ndx*ns, sim_scalar_t) : NULL;   // state-observation    (transposed)
  sim_scalar_t *DT = D ? SIM_STACK_ALLOC(d, nu*ns, sim_scalar_t) : NULL;    // control-observation  (transposed)

  // set offset pointers
  if (A) {
    DyDq = AT;
    DyDv = AT+ndx*nv;
    DyDa = AT+ndx*2*nv;
  }

  if (C) {
    DsDq = CT;
    DsDv = CT + ns*nv;
    DsDa = CT + ns*2*nv;
  }

  // get Jacobians
  SIM_d_stepFD(m, d, eps, flg_centered, DyDq, DyDv, DyDa, BT, DsDq, DsDv, DsDa, DT);


  // transpose
  if (A) sim_math_transpose(A, AT, ndx, ndx);
  if (B) sim_math_transpose(B, BT, nu, ndx);
  if (C) sim_math_transpose(C, CT, ndx, ns);
  if (D) sim_math_transpose(D, DT, nu, ns);

  sim_freeStack(d);
}

// finite differenced Jacobians of (force, sensors) = sim_inverse(state, acceleration)
//   all outputs are optional
//   output dimensions (transposed w.r.t Control Theory convention):
//     DfDq: (nv x nv)
//     DfDv: (nv x nv)
//     DfDa: (nv x nv)
//     DsDq: (nv x nsensordata)
//     DsDv: (nv x nsensordata)
//     DsDa: (nv x nsensordata)
//     DmDq: (nv x nM)
//   single-letter shortcuts:
//     inputs: q=qpos, v=qvel, a=qacc
//     outputs: f=qfrc_inverse, s=sensordata, m=qM
//   notes:
//     optionally compute mass matrix Jacobian DmDq
//     flg_actuation specifies whether to subtract qfrc_actuator from qfrc_inverse
void SIM_d_inverseFD(const sim_model_t* m, sim_data_t* d, sim_scalar_t eps, sim_byte_t flg_actuation,
                   sim_scalar_t *DfDq, sim_scalar_t *DfDv, sim_scalar_t *DfDa,
                   sim_scalar_t *DsDq, sim_scalar_t *DsDv, sim_scalar_t *DsDa,
                   sim_scalar_t *DmDq) {
  int nq = m->nq, nv = m->nv, nM = m->nM, ns = m->nsensordata;

  if (m->opt.integrator == SIM_INT_RK4) {
    SIM_ERROR("RK4 integrator is not supported");
  }

  if (m->opt.noslip_iterations) {
    SIM_ERROR("noslip solver is not supported");
  }

  // skip sensor computations if no sensor Jacobians requested
  int skipsensor = !DsDq && !DsDv && !DsDa;

  // local vectors
  sim_markStack(d);
  sim_scalar_t *pos        = SIM_STACK_ALLOC(d, nq, sim_scalar_t);                      // position
  sim_scalar_t *force      = SIM_STACK_ALLOC(d, nv, sim_scalar_t);                      // force
  sim_scalar_t *force_plus = SIM_STACK_ALLOC(d, nv, sim_scalar_t);                      // nudged force
  sim_scalar_t *sensor     = skipsensor ? NULL : SIM_STACK_ALLOC(d, ns, sim_scalar_t);  // sensor values
  sim_scalar_t *mass       = DmDq ? SIM_STACK_ALLOC(d, nM, sim_scalar_t) : NULL;        // mass matrix

  // save current positions
  sim_math_copy(pos, d->qpos, nq);

  // center point outputs
  inverseSkip(m, d, SIM_STAGE_NONE, skipsensor, flg_actuation, force);
  if (sensor) sim_math_copy(sensor, d->sensordata, ns);
  if (mass) sim_math_copy(mass, d->qM, nM);

  // acceleration: skip = SIM_STAGE_VEL
  if (DfDa || DsDa) {
    for (int i=0; i < nv; i++) {
      // nudge acceleration
      sim_scalar_t tmp = d->qacc[i];
      d->qacc[i] += eps;

      // inverse dynamics, get force output
      inverseSkip(m, d, SIM_STAGE_VEL, skipsensor, flg_actuation, force_plus);

      // restore
      d->qacc[i] = tmp;

      // row of force Jacobian
      if (DfDa) diff(DfDa + i*nv, force, force_plus, eps, nv);

      // row of sensor Jacobian
      if (DsDa) diff(DsDa + i*ns, sensor, d->sensordata, eps, ns);
    }
  }

  // velocity: skip = SIM_STAGE_POS
  if (DfDv || DsDv) {
    for (int i=0; i < nv; i++) {
      // nudge velocity
      sim_scalar_t tmp = d->qvel[i];
      d->qvel[i] += eps;

      // inverse dynamics, get force output
      inverseSkip(m, d, SIM_STAGE_POS, skipsensor, flg_actuation, force_plus);

      // restore
      d->qvel[i] = tmp;

      // row of force Jacobian
      if (DfDv) diff(DfDv + i*nv, force, force_plus, eps, nv);

      // row of sensor Jacobian
      if (DsDv) diff(DsDv + i*ns, sensor, d->sensordata, eps, ns);
    }
  }

  // position: skip = SIM_STAGE_NONE
  if (DfDq || DsDq || DmDq) {
    sim_scalar_t *dpos  = SIM_STACK_ALLOC(d, nv, sim_scalar_t);  // allocate position perturbation
    for (int i=0; i < nv; i++) {
      // nudge
      sim_math_zero(dpos, nv);
      dpos[i] = 1.0;
      sim_integratePos(m, d->qpos, dpos, eps);

      // inverse dynamics, get force output
      inverseSkip(m, d, SIM_STAGE_NONE, skipsensor, flg_actuation, force_plus);

      // restore
      sim_math_copy(d->qpos, pos, nq);

      // row of force Jacobian
      if (DfDq) diff(DfDq + i*nv, force, force_plus, eps, nv);

      // row of sensor Jacobian
      if (DsDq) diff(DsDq + i*ns, sensor, d->sensordata, eps, ns);

      // row of inertia Jacobian
      if (DmDq) diff(DmDq + i*nM, mass, d->qM, eps, nM);
    }
  }

  sim_freeStack(d);
}
