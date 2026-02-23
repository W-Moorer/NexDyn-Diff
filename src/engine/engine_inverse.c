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

#include "engine/engine_inverse.h"

#include <stddef.h>

#include <simcore/SIM_data.h>
#include <simcore/SIM_macro.h>
#include <simcore/SIM_model.h>
#include <simcore/SIM_san.h>  // IWYU pragma: keep
#include "engine/engine_collision_driver.h"
#include "engine/engine_core_constraint.h"
#include "engine/engine_core_smooth.h"
#include "engine/engine_derivative.h"
#include "engine/engine_memory.h"
#include "engine/engine_macro.h"
#include "engine/engine_forward.h"
#include "engine/engine_sensor.h"
#include "engine/engine_support.h"
#include "engine/engine_util_blas.h"
#include "engine/engine_util_errmem.h"
#include "engine/engine_util_misc.h"
#include "engine/engine_util_sparse.h"

// position-dependent computations
void sim_invPosition(const sim_model_t* m, sim_data_t* d) {
  TM_START1;
  TM_START;

  // clear flag for lazy evaluation
  d->flg_energypos = 0;

  sim_kinematics(m, d);
  sim_comPos(m, d);
  sim_camlight(m, d);
  sim_flex(m, d);
  sim_tendon(m, d);
  TM_END(SIM_TIMER_POS_KINEMATICS);

  sim_makeM(m, d);      // timed internally (POS_INERTIA)
  sim_factorM(m, d);    // timed internally (POS_INERTIA)

  sim_collision(m, d);  // timed internally (POS_COLLISION)

  TM_RESTART;
  sim_makeConstraint(m, d);
  TM_END(SIM_TIMER_POS_MAKE);

  TM_RESTART;
  sim_transmission(m, d);
  TM_ADD(SIM_TIMER_POS_KINEMATICS);

  TM_END1(SIM_TIMER_POSITION);
}


// velocity-dependent computations
void sim_invVelocity(const sim_model_t* m, sim_data_t* d) {
  sim_fwdVelocity(m, d);
}


// convert discrete-time qacc to continuous-time qacc
static void sim_discreteAcc(const sim_model_t* m, sim_data_t* d) {
  int nv = m->nv, nC = m->nC, nD = m->nD, dof_damping;
  sim_scalar_t *qacc = d->qacc;

  sim_markStack(d);
  sim_scalar_t* qfrc = SIM_STACK_ALLOC(d, nv, sim_scalar_t);

  // use selected integrator
  switch ((SIM_tIntegrator) m->opt.integrator) {
  case SIM_INT_RK4:
    // not supported by RK4
    SIM_ERROR("discrete inverse dynamics is not supported by RK4 integrator");
    return;

  case SIM_INT_EULER:
    // check for dof damping if disable flag is not set
    dof_damping = 0;
    if (!SIM_DISABLED(SIM_DSBL_EULERDAMP)) {
      for (int i=0; i < nv; i++) {
        if (m->dof_damping[i] > 0) {
          dof_damping = 1;
          break;
        }
      }
    }

    // if disabled or no dof damping, nothing to do
    if (!dof_damping) {
      sim_freeStack(d);
      return;
    }

    // set qfrc = (M + h*diag(B)) * qacc
    sim_mulM(m, d, qfrc, qacc);
    for (int i=0; i < nv; i++) {
      qfrc[i] += m->opt.timestep * m->dof_damping[i] * d->qacc[i];
    }
    break;

  case SIM_INT_IMPLICIT:
    // compute qDeriv
    SIM_d_smooth_vel(m, d, /* flg_bias = */ 1);

    // gather qLU <- qM (lower to full)
    sim_math_gatherMasked(d->qLU, d->M, m->mapM2D, nD);

    // set qLU = qM - dt*qDeriv
    sim_math_addToScl(d->qLU, d->qDeriv, -m->opt.timestep, m->nD);

    // set qfrc = qLU * qacc
    sim_math_mulMatVecSparse(qfrc, d->qLU, qacc, nv,
                        m->D_rownnz, m->D_rowadr, m->D_colind, /*rowsuper=*/NULL);
    break;

  case SIM_INT_IMPLICITFAST:
    // compute analytical derivative qDeriv; skip rne derivative
    SIM_d_smooth_vel(m, d, /* flg_bias = */ 0);

    // save mass matrix
    sim_scalar_t* Msave = SIM_STACK_ALLOC(d, m->nC, sim_scalar_t);
    sim_math_copy(Msave, d->M, m->nC);

    // modified mass matrix: gather qH <- qDeriv (full to lower)
    sim_math_gather(d->qH, d->qDeriv, m->mapD2M, nC);

    // set qH = M - dt*qDeriv
    sim_math_addScl(d->qH, d->M, d->qH, -m->opt.timestep, nC);

    // set qfrc = (M - dt*qDeriv) * qacc
    sim_math_mulSymVecSparse(qfrc, d->qH, qacc, m->nv, m->M_rownnz, m->M_rowadr, m->M_colind);
    break;
  }

  // solve for qacc: qfrc = M * qacc
  sim_solveM(m, d, qacc, qfrc, 1);

  sim_freeStack(d);
}


// inverse constraint solver
void sim_invConstraint(const sim_model_t* m, sim_data_t* d) {
  TM_START;
  int nefc = d->nefc;

  // no constraints: clear, return
  if (!nefc) {
    sim_math_zero(d->qfrc_constraint, m->nv);
    TM_END(SIM_TIMER_CONSTRAINT);
    return;
  }

  sim_markStack(d);
  sim_scalar_t* jar = SIM_STACK_ALLOC(d, nefc, sim_scalar_t);

  // compute jar = Jac*qacc - aref
  sim_mulJacVec(m, d, jar, d->qacc);
  sim_math_subFrom(jar, d->efc_aref, nefc);

  // call update function
  sim_constraintUpdate(m, d, jar, NULL, 0);

  sim_freeStack(d);
  TM_END(SIM_TIMER_CONSTRAINT);
}


// inverse dynamics with skip; skipstage is SIM_tStage
void sim_inverseSkip(const sim_model_t* m, sim_data_t* d,
                    int skipstage, int skipsensor) {
  TM_START;
  sim_markStack(d);
  sim_scalar_t* qacc;
  int nv = m->nv;

  // position-dependent
  if (skipstage < SIM_STAGE_POS) {
    sim_invPosition(m, d);
    if (!skipsensor) {
      sim_sensorPos(m, d);
    }
    if (SIM_ENABLED(SIM_ENBL_ENERGY) && !d->flg_energypos) {
      sim_energyPos(m, d);
    }
  }

  // velocity-dependent
  if (skipstage < SIM_STAGE_VEL) {
    sim_invVelocity(m, d);
    if (!skipsensor) {
      sim_sensorVel(m, d);
    }
    if (SIM_ENABLED(SIM_ENBL_ENERGY) && !d->flg_energyvel) {
      sim_energyVel(m, d);
    }
  }

  if (SIM_ENABLED(SIM_ENBL_INVDISCRETE)) {
    // save current qacc
    qacc = SIM_STACK_ALLOC(d, nv, sim_scalar_t);
    sim_math_copy(qacc, d->qacc, nv);

    // modify qacc in-place
    sim_discreteAcc(m, d);
  }

  // acceleration-dependent
  sim_invConstraint(m, d);

  // sum of bias forces in qfrc_inverse = centripetal + Coriolis + tendon bias
  sim_rne(m, d, 0, d->qfrc_inverse);
  sim_tendonBias(m, d, d->qfrc_inverse);

  if (!skipsensor) {
    d->flg_rnepost = 0;  // clear flag for lazy evaluation
    sim_sensorAcc(m, d);
  }

  // compute Ma = M*qacc
  sim_scalar_t* Ma = SIM_STACK_ALLOC(d, nv, sim_scalar_t);
  sim_mulM(m, d, Ma, d->qacc);

  // qfrc_inverse += Ma - qfrc_passive - qfrc_constraint
  for (int i=0; i < nv; i++) {
    d->qfrc_inverse[i] += Ma[i] - d->qfrc_passive[i] - d->qfrc_constraint[i];
  }

  if (SIM_ENABLED(SIM_ENBL_INVDISCRETE)) {
    // restore qacc
    sim_math_copy(d->qacc, qacc, nv);
  }

  sim_freeStack(d);
  TM_END(SIM_TIMER_INVERSE);
}


// inverse dynamics
void sim_inverse(const sim_model_t* m, sim_data_t* d) {
  sim_inverseSkip(m, d, SIM_STAGE_NONE, 0);
}


// compare forward and inverse dynamics, without changing results of forward
//    fwdinv[0] = norm(qfrc_constraint(forward) - qfrc_constraint(inverse))
//    fwdinv[1] = norm(qfrc_applied(forward) - qfrc_inverse)
void sim_compareFwdInv(const sim_model_t* m, sim_data_t* d) {
  int nv = m->nv, nefc = d->nefc;
  sim_scalar_t *qforce, *dif, *save_qfrc_constraint, *save_efc_force;

  // clear result, return if no constraints
  d->solver_fwdinv[0] = d->solver_fwdinv[1] = 0;
  if (!nefc) {
    return;
  }

  // allocate
  sim_markStack(d);
  qforce = SIM_STACK_ALLOC(d, nv, sim_scalar_t);
  dif = SIM_STACK_ALLOC(d, nv, sim_scalar_t);
  save_qfrc_constraint = SIM_STACK_ALLOC(d, nv, sim_scalar_t);
  save_efc_force = SIM_STACK_ALLOC(d, nefc, sim_scalar_t);

  // qforce = qfrc_applied + J'*xfrc_applied + qfrc_actuator
  //  should equal result of inverse dynamics
  sim_math_add(qforce, d->qfrc_applied, d->qfrc_actuator, nv);
  sim_xfrcAccumulate(m, d, qforce);

  // save forward dynamics results that are about to be modified
  sim_math_copy(save_qfrc_constraint, d->qfrc_constraint, nv);
  sim_math_copy(save_efc_force, d->efc_force, nefc);

  // run inverse dynamics, do not update position and velocity,
  sim_inverseSkip(m, d, SIM_STAGE_VEL, 1);  // 1: do not recompute sensors and energy

  // compute statistics
  sim_math_sub(dif, save_qfrc_constraint, d->qfrc_constraint, nv);
  d->solver_fwdinv[0] = sim_math_norm(dif, nv);
  sim_math_sub(dif, qforce, d->qfrc_inverse, nv);
  d->solver_fwdinv[1] = sim_math_norm(dif, nv);

  // restore forward dynamics results
  sim_math_copy(d->qfrc_constraint, save_qfrc_constraint, nv);
  sim_math_copy(d->efc_force, save_efc_force, nefc);

  sim_freeStack(d);
}
