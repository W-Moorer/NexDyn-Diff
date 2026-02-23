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

#include "engine/engine_forward.h"

#include <stddef.h>
#include <stdio.h>

#include <simcore/SIM_data.h>
#include <simcore/SIM_macro.h>
#include <simcore/SIM_model.h>
#include <simcore/SIM_san.h>  // IWYU pragma: keep
#include <simcore/SIM_plugin.h>
#include "engine/engine_callback.h"
#include "engine/engine_collision_driver.h"
#include "engine/engine_core_constraint.h"
#include "engine/engine_core_smooth.h"
#include "engine/engine_derivative.h"
#include "engine/engine_core_util.h"
#include "engine/engine_inverse.h"
#include "engine/engine_island.h"
#include "engine/engine_macro.h"
#include "engine/engine_memory.h"
#include "engine/engine_passive.h"
#include "engine/engine_plugin.h"
#include "engine/engine_sensor.h"
#include "engine/engine_sleep.h"
#include "engine/engine_solver.h"
#include "engine/engine_support.h"
#include "engine/engine_util_blas.h"
#include "engine/engine_util_errmem.h"
#include "engine/engine_util_misc.h"
#include "engine/engine_util_solve.h"
#include "engine/engine_util_sparse.h"
#include "thread/thread_pool.h"
#include "thread/thread_task.h"



//--------------------------- check values ---------------------------------------------------------

// check positions, reset if bad
void sim_checkPos(const sim_model_t* m, sim_data_t* d) {
  int nq = m->nq;
  const sim_scalar_t* qpos = d->qpos;
  for (int i=0; i < nq; i++) {
    if (sim_math_isBad(qpos[i])) {
      sim_runtime_warning(d, SIM_WARN_BADQPOS, i);
      if (!SIM_DISABLED(SIM_DSBL_AUTORESET)) {
        sim_resetData(m, d);
      }
      d->warning[SIM_WARN_BADQPOS].number++;
      d->warning[SIM_WARN_BADQPOS].lastinfo = i;
      return;
    }
  }
}


// check velocities, reset if bad
void sim_checkVel(const sim_model_t* m, sim_data_t* d) {
  int sleep_filter = SIM_ENABLED(SIM_ENBL_SLEEP) && d->nv_awake < m->nv;
  int nv = sleep_filter ? d->nv_awake : m->nv;

  for (int j=0; j < nv; j++) {
    int i = sleep_filter ? d->dof_awake_ind[j] : j;

    if (sim_math_isBad(d->qvel[i])) {
      sim_runtime_warning(d, SIM_WARN_BADQVEL, i);
      if (!SIM_DISABLED(SIM_DSBL_AUTORESET)) {
        sim_resetData(m, d);
      }
      d->warning[SIM_WARN_BADQVEL].number++;
      d->warning[SIM_WARN_BADQVEL].lastinfo = i;
      return;
    }
  }
}


// check accelerations, reset if bad
void sim_checkAcc(const sim_model_t* m, sim_data_t* d) {
  int sleep_filter = SIM_ENABLED(SIM_ENBL_SLEEP) && d->nv_awake < m->nv;
  int nv = sleep_filter ? d->nv_awake : m->nv;

  for (int j=0; j < nv; j++) {
    int i = sleep_filter ? d->dof_awake_ind[j] : j;

    if (sim_math_isBad(d->qacc[i])) {
      sim_runtime_warning(d, SIM_WARN_BADQACC, i);
      if (!SIM_DISABLED(SIM_DSBL_AUTORESET)) {
        sim_resetData(m, d);
      }
      d->warning[SIM_WARN_BADQACC].number++;
      d->warning[SIM_WARN_BADQACC].lastinfo = i;
      if (!SIM_DISABLED(SIM_DSBL_AUTORESET)) {
        sim_forward(m, d);
      }
      return;
    }
  }
}


//-------------------------- solver components -----------------------------------------------------

// args for internal functions in sim_fwdPosition
struct SIM_FwdPositionArgs_ {
  const sim_model_t* m;
  sim_data_t* d;
};
typedef struct SIM_FwdPositionArgs_ SIM_FwdPositionArgs;

// wrapper for sim_crb and sim_factorM
void* sim_inertialThreaded(void* args) {
  SIM_FwdPositionArgs* forward_args = (SIM_FwdPositionArgs*) args;
  sim_makeM(forward_args->m, forward_args->d);
  sim_factorM(forward_args->m, forward_args->d);
  return NULL;
}

// wrapper for sim_collision
void* sim_collisionThreaded(void* args) {
  SIM_FwdPositionArgs* forward_args = (SIM_FwdPositionArgs*) args;
  sim_collision(forward_args->m, forward_args->d);
  return NULL;
}

// kinematics-related computations
void sim_fwdKinematics(const sim_model_t* m, sim_data_t* d) {
  sim_kinematics(m, d);
  sim_comPos(m, d);
  sim_camlight(m, d);
  sim_flex(m, d);
  sim_tendon(m, d);
  if (sim_wakeTendon(m, d)) {
    sim_updateSleep(m, d);
  }
}

// position-dependent computations
void sim_fwdPosition(const sim_model_t* m, sim_data_t* d) {
  TM_START1;

  // clear position-dependent flags for lazy evaluation
  d->flg_energypos = 0;

  TM_START;
  sim_fwdKinematics(m, d);

  TM_END(SIM_TIMER_POS_KINEMATICS);

  // no threadpool: inertia and collision on main thread
  if (!d->threadpool) {
    // inertia, timed internally (POS_INERTIA)
    sim_makeM(m, d);
    sim_factorM(m, d);

    // collision, timed internally (POS_COLLISION)
    sim_collision(m, d);
  }

  // have threadpool: inertia and collision on separate threads
  else {
    SIM_Task tasks[2];
    SIM_FwdPositionArgs forward_args;
    forward_args.m = m;
    forward_args.d = d;

    sim_math_defaultTask(&tasks[0]);
    tasks[0].func = sim_inertialThreaded;
    tasks[0].args = &forward_args;
    sim_math_threadPoolEnqueue((SIM_ThreadPool*)d->threadpool, &tasks[0]);

    sim_math_defaultTask(&tasks[1]);
    tasks[1].func = sim_collisionThreaded;
    tasks[1].args = &forward_args;
    sim_math_threadPoolEnqueue((SIM_ThreadPool*)d->threadpool, &tasks[1]);

    sim_math_taskJoin(&tasks[0]);
    sim_math_taskJoin(&tasks[1]);
  }

  if (sim_wakeCollision(m, d)) {
    sim_updateSleep(m, d);
    sim_collision(m, d);
  }

  if (sim_wakeEquality(m, d)) {
    sim_updateSleep(m, d);
  }

  TM_RESTART;
  sim_makeConstraint(m, d);
  sim_island(m, d);
  TM_END(SIM_TIMER_POS_MAKE);

  TM_RESTART;
  sim_transmission(m, d);
  TM_ADD(SIM_TIMER_POS_KINEMATICS);

  TM_RESTART;
  sim_projectConstraint(m, d);
  TM_END(SIM_TIMER_POS_PROJECT);

  TM_END1(SIM_TIMER_POSITION);
}


// velocity-dependent computations
void sim_fwdVelocity(const sim_model_t* m, sim_data_t* d) {
  TM_START;

  // clear velocity-dependent flags for lazy evaluation
  d->flg_subtreevel = 0;
  d->flg_energyvel = 0;

  // flexedge velocity: always sparse
  sim_math_mulMatVecSparse(d->flexedge_velocity, d->flexedge_J, d->qvel, m->nflexedge,
                      m->flexedge_J_rownnz, m->flexedge_J_rowadr, m->flexedge_J_colind, NULL);

  // tendon velocity: always sparse
  sim_math_mulMatVecSparse(d->ten_velocity, d->ten_J, d->qvel, m->ntendon,
                      d->ten_J_rownnz, d->ten_J_rowadr, d->ten_J_colind, NULL);

  // actuator velocity: always sparse
  if (!SIM_DISABLED(SIM_DSBL_ACTUATION)) {
    sim_math_mulMatVecSparse(d->actuator_velocity, d->actuator_moment, d->qvel, m->nu,
                        d->moment_rownnz, d->moment_rowadr, d->moment_colind, NULL);
  } else {
    sim_math_zero(d->actuator_velocity, m->nu);
  }

  // com-based velocities, passive forces, constraint references
  sim_comVel(m, d);
  sim_passive(m, d);
  sim_referenceConstraint(m, d);

  // compute qfrc_bias with abbreviated RNE (without acceleration)
  sim_rne(m, d, 0, d->qfrc_bias);

  // add bias force due to tendon armature
  sim_tendonBias(m, d, d->qfrc_bias);

  TM_END(SIM_TIMER_VELOCITY);
}



// clamp vector to range
static void clampVec(sim_scalar_t* vec, const sim_scalar_t* range, const sim_byte_t* limited, int n,
                      const int* index) {
  for (int i=0; i < n; i++) {
    int j = index ? index[i] : i;
    if (limited[i]) {
      vec[j] = sim_math_clip(vec[j], range[2*i], range[2*i + 1]);
    }
  }
}


// (qpos, qvel, ctrl, act) => (qfrc_actuator, actuator_force, act_dot)
void sim_fwdActuation(const sim_model_t* m, sim_data_t* d) {
  TM_START;
  int nv = m->nv, nu = m->nu, ntendon = m->ntendon;
  sim_scalar_t gain, bias, tau;
  sim_scalar_t *prm, *force = d->actuator_force;

  // clear actuator_force
  sim_math_zero(force, nu);

  int sleep_filter = SIM_ENABLED(SIM_ENBL_SLEEP);

  // disabled or no actuation: return
  if (nu == 0 || SIM_DISABLED(SIM_DSBL_ACTUATION)) {
    sim_math_zero(d->qfrc_actuator, nv);
    return;
  }

  // any tendon transmission targets with force limits
  int tendon_frclimited = 0;

  // local copy of ctrl
  sim_markStack(d);
  sim_scalar_t *ctrl = SIM_STACK_ALLOC(d, nu, sim_scalar_t);

  // read from ctrl or history buffer for delayed actuators
  for (int i = 0; i < nu; i++) {
    int interp = m->actuator_history[2*i+1];
    ctrl[i] = m->actuator_delay[i] ? sim_readCtrl(m, d, i, d->time, interp) : d->ctrl[i];
  }

  // clamp local copy
  if (!SIM_DISABLED(SIM_DSBL_CLAMPCTRL)) {
    clampVec(ctrl, m->actuator_ctrlrange, m->actuator_ctrllimited, nu, NULL);
  }

  // check controls, set all to 0 if any are bad
  for (int i=0; i < nu; i++) {
    if (sim_math_isBad(ctrl[i])) {
      sim_runtime_warning(d, SIM_WARN_BADCTRL, i);
      sim_math_zero(ctrl, nu);
      break;
    }
  }

  // act_dot for stateful actuators
  for (int i=0; i < nu; i++) {
    if (sleep_filter && sim_sleepState(m, d, SIM_OBJ_ACTUATOR, i) == sim_spec_ASLEEP) {
      continue;
    }

    int act_first = m->actuator_actadr[i];
    if (act_first < 0) {
      continue;
    }

    // zero act_dot for actuator plugins
    if (m->actuator_actnum[i]) {
      sim_math_zero(d->act_dot + act_first, m->actuator_actnum[i]);
    }

    // extract info
    prm = m->actuator_dynprm + i*SIM_NDYN;

    // index into the last element in act. For most actuators it's also the
    // first element, but actuator plugins might store their own state in act.
    int act_last = act_first + m->actuator_actnum[i] - 1;

    // compute act_dot according to dynamics type
    switch ((SIM_tDyn) m->actuator_dyntype[i]) {
    case SIM_DYN_INTEGRATOR:          // simple integrator
      d->act_dot[act_last] = ctrl[i];
      break;

    case SIM_DYN_FILTER:              // linear filter: prm = tau
    case SIM_DYN_FILTEREXACT:
      tau = sim_math_max(SIM_MINVAL, prm[0]);
      d->act_dot[act_last] = (ctrl[i] - d->act[act_last]) / tau;
      break;

    case SIM_DYN_MUSCLE:              // muscle model: prm = (tau_act, tau_deact)
      d->act_dot[act_last] = sim_math_muscleDynamics(
          ctrl[i], d->act[act_last], prm);
      break;

    default:                        // user dynamics
      if (SIM_cb_act_dyn) {
        if (m->actuator_actnum[i] == 1) {
          // scalar activation dynamics, get act_dot
          d->act_dot[act_last] = SIM_cb_act_dyn(m, d, i);
        } else {
          // higher-order dynamics, SIM_cb_act_dyn writes into act_dot directly
          SIM_cb_act_dyn(m, d, i);
        }
      }
    }
  }

  // get act_dot from actuator plugins
  if (m->nplugin) {
    const int nslot = sim_plugin_pluginCount();
    for (int i=0; i < m->nplugin; i++) {
      const int slot = m->plugin[i];
      const SIM_pPlugin* plugin = sim_plugin_getPluginAtSlotUnsafe(slot, nslot);
      if (!plugin) {
        SIM_ERROR("invalid plugin slot: %d", slot);
      }
      if (plugin->capabilityflags & SIM_PLUGIN_ACTUATOR) {
        if (plugin->actuator_act_dot) {
          plugin->actuator_act_dot(m, d, i);
        }
      }
    }
  }

  // force = gain .* [ctrl/act] + bias
  for (int i=0; i < nu; i++) {
    // skip if sleeping
    if (sleep_filter && sim_sleepState(m, d, SIM_OBJ_ACTUATOR, i) == sim_spec_ASLEEP) {
      continue;
    }

    // skip if disabled
    if (sim_actuatorDisabled(m, i)) {
      continue;
    }

    // skip actuator plugins -- these are handled after builtin actuator types
    if (m->actuator_plugin[i] >= 0) {
      continue;
    }

    // check for tendon transmission with force limits
    if (ntendon && !tendon_frclimited && m->actuator_trntype[i] == SIM_TRN_TENDON) {
      tendon_frclimited = m->tendon_actfrclimited[m->actuator_trnid[2*i]];
    }

    // extract gain info
    prm = m->actuator_gainprm + SIM_NGAIN*i;

    // handle according to gain type
    switch ((SIM_tGain) m->actuator_gaintype[i]) {
    case SIM_GAIN_FIXED:              // fixed gain: prm = gain
      gain = prm[0];
      break;

    case SIM_GAIN_AFFINE:             // affine: prm = [const, kp, kv]
      gain = prm[0] + prm[1]*d->actuator_length[i] + prm[2]*d->actuator_velocity[i];
      break;

    case SIM_GAIN_MUSCLE:             // muscle gain
      gain = sim_math_muscleGain(d->actuator_length[i],
                            d->actuator_velocity[i],
                            m->actuator_lengthrange+2*i,
                            m->actuator_acc0[i],
                            prm);
      break;

    default:                        // user gain
      if (SIM_cb_act_gain) {
        gain = SIM_cb_act_gain(m, d, i);
      } else {
        gain = 1;
      }
    }

    // set force = gain .* [ctrl/act]
    if (m->actuator_actadr[i] == -1) {
      force[i] = gain * ctrl[i];
    } else {
      // use last activation variable associated with actuator i
      int act_adr = m->actuator_actadr[i] + m->actuator_actnum[i] - 1;

      sim_scalar_t act;
      if (m->actuator_actearly[i]) {
        act = sim_nextActivation(m, d, i, act_adr, d->act_dot[act_adr]);
      } else {
        act = d->act[act_adr];
      }
      force[i] = gain * act;
    }

    // extract bias info
    prm = m->actuator_biasprm + SIM_NBIAS*i;

    // handle according to bias type
    switch ((SIM_tBias) m->actuator_biastype[i]) {
    case SIM_BIAS_NONE:               // none
      bias = 0.0;
      break;

    case SIM_BIAS_AFFINE:             // affine: prm = [const, kp, kv]
      bias = prm[0] + prm[1]*d->actuator_length[i] + prm[2]*d->actuator_velocity[i];
      break;

    case SIM_BIAS_MUSCLE:             // muscle passive force
      bias =  sim_math_muscleBias(d->actuator_length[i],
                             m->actuator_lengthrange+2*i,
                             m->actuator_acc0[i],
                             prm);
      break;

    default:                        // user bias
      if (SIM_cb_act_bias) {
        bias = SIM_cb_act_bias(m, d, i);
      } else {
        bias = 0;
      }
    }

    // add bias
    force[i] += bias;
  }

  // handle actuator plugins
  if (m->nplugin) {
    const int nslot = sim_plugin_pluginCount();
    for (int i=0; i < m->nplugin; i++) {
      const int slot = m->plugin[i];
      const SIM_pPlugin* plugin = sim_plugin_getPluginAtSlotUnsafe(slot, nslot);
      if (!plugin) {
        SIM_ERROR("invalid plugin slot: %d", slot);
      }
      if (plugin->capabilityflags & SIM_PLUGIN_ACTUATOR) {
        if (!plugin->compute) {
          SIM_ERROR("`compute` is a null function pointer for plugin at slot %d", slot);
        }
        plugin->compute(m, d, i, SIM_PLUGIN_ACTUATOR);
      }
    }
  }

  // clamp tendon total actuator force
  if (tendon_frclimited) {
    // compute total force for each tendon
    sim_scalar_t* tendon_total_force = SIM_STACK_ALLOC(d, ntendon, sim_scalar_t);
    sim_math_zero(tendon_total_force, ntendon);
    for (int i=0; i < nu; i++) {
      if (m->actuator_trntype[i] == SIM_TRN_TENDON) {
        int tendon_id = m->actuator_trnid[2*i];
        if (m->tendon_actfrclimited[tendon_id]) {
          tendon_total_force[tendon_id] += force[i];
        }
      }
    }

    // scale tendon actuator forces if limited and outside range
    for (int i=0; i < nu; i++) {
      if (m->actuator_trntype[i] != SIM_TRN_TENDON) {
        continue;
      }
      int tendon_id = m->actuator_trnid[2*i];
      sim_scalar_t tendon_force = tendon_total_force[tendon_id];
      if (m->tendon_actfrclimited[tendon_id] && tendon_force) {
        const sim_scalar_t* range = m->tendon_actfrcrange + 2 * tendon_id;
        if (tendon_force < range[0]) {
          force[i] *= range[0] / tendon_force;
        } else if (tendon_force > range[1]) {
          force[i] *= range[1] / tendon_force;
        }
      }
    }
  }

  // clamp actuator_force
  clampVec(force, m->actuator_forcerange, m->actuator_forcelimited, nu, NULL);

  // qfrc_actuator = moment' * force
  sim_math_mulMatTVecSparse(d->qfrc_actuator, d->actuator_moment, force, nu, nv,
                       d->moment_rownnz, d->moment_rowadr, d->moment_colind);

  // actuator-level gravity compensation
  if (m->ngravcomp && !SIM_DISABLED(SIM_DSBL_GRAVITY) && sim_math_norm3(m->opt.gravity)) {
    // number of dofs for each joint type: {SIM_JNT_FREE, SIM_JNT_BALL, SIM_JNT_SLIDE, SIM_JNT_HINGE}
    static const int jnt_dofnum[4] = {6, 3, 1, 1};
    int njnt = m->njnt;
    for (int i=0; i < njnt; i++) {
      // skip if gravcomp added as passive force
      if (!m->jnt_actgravcomp[i]) {
        continue;
      }

      // add gravcomp force
      int dofnum = jnt_dofnum[m->jnt_type[i]];
      int dofadr = m->jnt_dofadr[i];
      sim_math_addTo(d->qfrc_actuator + dofadr, d->qfrc_gravcomp + dofadr, dofnum);
    }
  }

  // clamp qfrc_actuator to joint-level actuator force limits
  clampVec(d->qfrc_actuator, m->jnt_actfrcrange, m->jnt_actfrclimited, m->njnt, m->jnt_dofadr);

  sim_freeStack(d);
  TM_END(SIM_TIMER_ACTUATION);
}


// add up all non-constraint forces, compute qacc_smooth
void sim_fwdAcceleration(const sim_model_t* m, sim_data_t* d) {
  int sleep_filter = SIM_ENABLED(SIM_ENBL_SLEEP) && d->nv_awake < m->nv;
  int nv;
  const int* index;

  // qfrc_smooth = qfrc_passive - qfrc_bias + qfrc_applied + qfrc_actuator
  if (!sleep_filter) {
    nv = m->nv;
    index = NULL;
    sim_math_sub(d->qfrc_smooth, d->qfrc_passive, d->qfrc_bias, nv);
    sim_math_addTo(d->qfrc_smooth, d->qfrc_applied, nv);
    sim_math_addTo(d->qfrc_smooth, d->qfrc_actuator, nv);
  } else {
    nv = d->nv_awake;
    index = d->dof_awake_ind;
    sim_math_subInd(d->qfrc_smooth, d->qfrc_passive, d->qfrc_bias, index, nv);
    sim_math_addToInd(d->qfrc_smooth, d->qfrc_applied, index, nv);
    sim_math_addToInd(d->qfrc_smooth, d->qfrc_actuator, index, nv);
  }

  // qfrc_smooth += project(xfrc_applied)
  sim_xfrcAccumulate(m, d, d->qfrc_smooth);

  // copy for in-place solve: qacc_smooth = qfrc_smooth
  if (!sleep_filter) {
    sim_math_copy(d->qacc_smooth, d->qfrc_smooth, nv);
  } else {
    sim_math_copyInd(d->qacc_smooth, d->qfrc_smooth, index, nv);
  }

  // qacc_smooth = M \ qfrc_smooth
  sim_solveLD(d->qacc_smooth, d->qLD, d->qLDiagInv, nv, 1,
             m->M_rownnz, m->M_rowadr, m->M_colind, index);
}


// warmstart/init solver
static void warmstart(const sim_model_t* m, sim_data_t* d) {
  int nv = m->nv, nefc = d->nefc;

  // warmstart with best of (qacc_warmstart, qacc_smooth)
  if (!SIM_DISABLED(SIM_DSBL_WARMSTART)) {
    sim_markStack(d);
    sim_scalar_t* jar = SIM_STACK_ALLOC(d, nefc, sim_scalar_t);

    // start with qacc = qacc_warmstart
    sim_math_copy(d->qacc, d->qacc_warmstart, nv);

    // compute jar(qacc_warmstart)
    sim_mulJacVec(m, d, jar, d->qacc_warmstart);
    sim_math_subFrom(jar, d->efc_aref, nefc);

    // update constraints, save cost(qacc_warmstart)
    sim_scalar_t cost_warmstart;
    sim_constraintUpdate(m, d, jar, &cost_warmstart, 0);

    // PGS
    if (m->opt.solver == SIM_SOL_PGS) {
      // cost(force_warmstart)
      sim_scalar_t PGS_warmstart = sim_math_dot(d->efc_force, d->efc_b, nefc);
      sim_scalar_t* ARf = SIM_STACK_ALLOC(d, nefc, sim_scalar_t);
      if (sim_isSparse(m))
        sim_math_mulMatVecSparse(ARf, d->efc_AR, d->efc_force, nefc,
                            d->efc_AR_rownnz, d->efc_AR_rowadr,
                            d->efc_AR_colind, NULL);
      else {
        sim_math_mulMatVec(ARf, d->efc_AR, d->efc_force, nefc, nefc);
      }
      PGS_warmstart += 0.5*sim_math_dot(d->efc_force, ARf, nefc);

      // use zero if better
      if (PGS_warmstart > 0) {
        sim_math_zero(d->efc_force, nefc);
        sim_math_zero(d->qfrc_constraint, nv);
      }
    }

    // non-PGS
    else {
      // add Gauss to cost(qacc_warmstart)
      sim_scalar_t* Ma = SIM_STACK_ALLOC(d, nv, sim_scalar_t);
      sim_mulM(m, d, Ma, d->qacc_warmstart);
      for (int i=0; i < nv; i++) {
        cost_warmstart += 0.5*(Ma[i]-d->qfrc_smooth[i])*(d->qacc_warmstart[i]-d->qacc_smooth[i]);
      }

      // cost(qacc_smooth)
      sim_scalar_t cost_smooth;
      sim_constraintUpdate(m, d, d->efc_b, &cost_smooth, 0);

      // use qacc_smooth if better
      if (cost_warmstart > cost_smooth) {
        sim_math_copy(d->qacc, d->qacc_smooth, nv);
      }
    }

    // have island structure: unconstrained qacc = qacc_smooth
    if (d->nisland > 0) {
      // loop over unconstrained dofs in map_idof2dof[nidof, nv)
      for (int i=d->nidof; i < nv; i++) {
        int dof = d->map_idof2dof[i];
        d->qacc[dof] = d->qacc_smooth[dof];
      }
    }

    sim_freeStack(d);
  }

  // coldstart with qacc = qacc_smooth, efc_force = 0
  else {
    sim_math_copy(d->qacc, d->qacc_smooth, nv);
    sim_math_zero(d->efc_force, nefc);
  }
}


// struct encapsulating arguments to thread task
struct SIM_SolIslandArgs_ {
  const sim_model_t* m;
  sim_data_t* d;
  int island;
};
typedef struct SIM_SolIslandArgs_ SIM_SolIslandArgs;

// extract arguments, pass to CG solver
static void* CG_wrapper(void* args) {
  SIM_SolIslandArgs* solargs = (SIM_SolIslandArgs*) args;
  sim_solCG_island(solargs->m, solargs->d, solargs->island, solargs->m->opt.iterations);
  return NULL;
}

// extract arguments, pass to Newton solver
static void* Newton_wrapper(void* args) {
  SIM_SolIslandArgs* solargs = (SIM_SolIslandArgs*) args;
  sim_solNewton_island(solargs->m, solargs->d, solargs->island, solargs->m->opt.iterations);
  return NULL;
}

// CG solver, multi-threaded over islands
static void solve_threaded(const sim_model_t* m, sim_data_t* d, int flg_Newton) {
  sim_markStack(d);
  // allocate array of arguments to be passed to threads
  SIM_SolIslandArgs* sol_island_args = SIM_STACK_ALLOC(d, d->nisland, SIM_SolIslandArgs);
  SIM_Task* tasks = SIM_STACK_ALLOC(d, d->nisland, SIM_Task);

  for (int island = 0; island < d->nisland; ++island) {
    sol_island_args[island].m = m;
    sol_island_args[island].d = d;
    sol_island_args[island].island = island;

    sim_math_defaultTask(&tasks[island]);
    tasks[island].func = flg_Newton ? Newton_wrapper : CG_wrapper;
    tasks[island].args = &sol_island_args[island];
    sim_math_threadPoolEnqueue((SIM_ThreadPool*)d->threadpool, &tasks[island]);
  }

  for (int island = 0; island < d->nisland; ++island) {
    sim_math_taskJoin(&tasks[island]);
  }

  sim_freeStack(d);
}


// compute efc_b, efc_force, qfrc_constraint; update qacc
void sim_fwdConstraint(const sim_model_t* m, sim_data_t* d) {
  TM_START;
  int nv = m->nv, nefc = d->nefc, nisland = d->nisland;

  // always clear qfrc_constraint
  sim_math_zero(d->qfrc_constraint, nv);

  // no constraints: copy unconstrained acc, clear forces, return
  if (!nefc) {
    sim_math_copy(d->qacc, d->qacc_smooth, nv);
    sim_math_zeroInt(d->solver_niter, SIM_NISLAND);
    TM_END(SIM_TIMER_CONSTRAINT);
    return;
  }

  // compute efc_b = J*qacc_smooth - aref
  sim_mulJacVec(m, d, d->efc_b, d->qacc_smooth);
  sim_math_subFrom(d->efc_b, d->efc_aref, nefc);

  // warmstart solver
  warmstart(m, d);
  sim_math_zeroInt(d->solver_niter, SIM_NISLAND);

  // check if islands are supported
  int islands_supported = !SIM_DISABLED(SIM_DSBL_ISLAND)    &&
                          nisland > 0                   &&
                          m->opt.noslip_iterations == 0 &&
                          (m->opt.solver == SIM_SOL_CG || m->opt.solver == SIM_SOL_NEWTON);

  // run solver over constraint islands
  if (islands_supported) {
    int nidof = d->nidof;

    // copy inputs to islands (vel+acc deps, pos-dependent already copied in sim_island)
    sim_math_gather(d->ifrc_smooth,     d->qfrc_smooth,     d->map_idof2dof, nidof);
    sim_math_gather(d->ifrc_constraint, d->qfrc_constraint, d->map_idof2dof, nidof);
    sim_math_gather(d->iacc_smooth,     d->qacc_smooth,     d->map_idof2dof, nidof);
    sim_math_gather(d->iacc,            d->qacc,            d->map_idof2dof, nidof);
    sim_math_gather(d->iefc_force,      d->efc_force,       d->map_iefc2efc, nefc);
    sim_math_gather(d->iefc_aref,       d->efc_aref,        d->map_iefc2efc, nefc);

    // solve per island, with or without threads
    if (!d->threadpool) {
      // no threadpool, loop over islands
      for (int island=0; island < nisland; island++) {
        if (m->opt.solver == SIM_SOL_NEWTON) {
          sim_solNewton_island(m, d, island, m->opt.iterations);
        } else {
          sim_solCG_island(m, d, island, m->opt.iterations);
        }
      }
    } else {
      // have threadpool, solve using threads
      solve_threaded(m, d, m->opt.solver == SIM_SOL_NEWTON);
    }

    // copy back solver outputs (scatter dofs since ni <= nv)
    sim_math_scatter(d->qacc,            d->iacc,            d->map_idof2dof, nidof);
    sim_math_scatter(d->qfrc_constraint, d->ifrc_constraint, d->map_idof2dof, nidof);
    sim_math_gather(d->efc_force, d->iefc_force, d->map_efc2iefc, nefc);
  }

  // run solver over all constraints
  else {
    switch ((SIM_tSolver) m->opt.solver) {
    case SIM_SOL_PGS:                     // PGS
      sim_solPGS(m, d, m->opt.iterations);
      break;

    case SIM_SOL_CG:                      // CG
      sim_solCG(m, d, m->opt.iterations);
      break;

    case SIM_SOL_NEWTON:                  // Newton
      sim_solNewton(m, d, m->opt.iterations);
      break;

    default:
      SIM_ERROR("unknown solver type %d", m->opt.solver);
    }
  }

  // run noslip solver if enabled
  if (m->opt.noslip_iterations > 0) {
    sim_solNoSlip(m, d, m->opt.noslip_iterations);
  }

  TM_END(SIM_TIMER_CONSTRAINT);
}


//-------------------------- state advancement and integration  ------------------------------------

// advance state and time given activation derivatives, acceleration, and optional velocity
static void sim_advance(const sim_model_t* m, sim_data_t* d,
                       const sim_scalar_t* act_dot, const sim_scalar_t* qacc, const sim_scalar_t* qvel) {
  int nu = m->nu, nsensor = m->nsensor;

  // advance history buffers
  if (m->nhistory > 0) {
    // advance ctrl history buffers
    for (int i = 0; i < nu; i++) {
      int nsample = m->actuator_history[2*i];
      if (nsample == 0) continue;

      // get history buffer pointer and insert ctrl at current time
      sim_scalar_t* buf = d->history + m->actuator_historyadr[i];
      *sim_math_historyInsert(buf, nsample, /*dim=*/1, d->time) = d->ctrl[i];
    }

    // advance sensor history buffers
    for (int i = 0; i < nsensor; i++) {
      int nsample = m->sensor_history[2*i];
      if (nsample == 0) continue;

      // get history buffer parameters
      int dim = m->sensor_dim[i];
      sim_scalar_t* buf = d->history + m->sensor_historyadr[i];
      sim_scalar_t delay = m->sensor_delay[i];
      sim_scalar_t interval = m->sensor_interval[2*i];

      if (interval > 0) {
        // interval mode: if condition is satisfied, compute; otherwise copy
        sim_scalar_t time_prev = buf[0];  // first slot stores previous sensor tick
        if (time_prev + interval <= d->time) {
          buf[0] += interval;  // advance by exact interval (continuous time)
          sim_scalar_t* slot = sim_math_historyInsert(buf, nsample, dim, d->time);
          if (delay > 0) {
            // have delay, compute sensor
            sim_computeSensor(m, d, i, slot);
          } else {
            // no delay, copy from sensordata (already computed)
            sim_math_copy(slot, d->sensordata + m->sensor_adr[i], dim);
          }
        }
      } else if (delay > 0) {
        // delay-only mode: always compute and insert
        sim_scalar_t* slot = sim_math_historyInsert(buf, nsample, dim, d->time);
        sim_computeSensor(m, d, i, slot);
      } else {
        // history-only mode: copy from sensordata (already computed)
        sim_scalar_t* slot = sim_math_historyInsert(buf, nsample, dim, d->time);
        sim_math_copy(slot, d->sensordata + m->sensor_adr[i], dim);
      }
    }
  }

  // advance activations
  if (m->na && !SIM_DISABLED(SIM_DSBL_ACTUATION)) {
    for (int i=0; i < nu; i++) {
      int actadr = m->actuator_actadr[i];
      int actadr_end = actadr + m->actuator_actnum[i];
      for (int j=actadr; j < actadr_end; j++) {
        // if disabled, set act_dot to 0
        d->act[j] = sim_nextActivation(m, d, i, j, sim_actuatorDisabled(m, i) ? 0 : act_dot[j]);
      }
    }
  }

  // put islands to sleep according to velocity tolerance
  if (sim_sleep(m, d)) {
    // if any trees put to sleep (qvel set to 0), recompute all velocity-dependent quantities
    sim_forwardSkip(m, d, SIM_STAGE_POS, 0);

    // update sleep indices
    sim_updateSleep(m, d);
  }

  // advance velocities
  int sleep_filter = SIM_ENABLED(SIM_ENBL_SLEEP) && d->ntree_awake < m->ntree;
  if (sleep_filter) {
    sim_math_addToSclInd(d->qvel, qacc, d->dof_awake_ind, m->opt.timestep, d->nv_awake);
  } else {
    sim_math_addToScl(d->qvel, qacc, m->opt.timestep, m->nv);
  }

  // advance positions with qvel if given, d->qvel otherwise (semi-implicit)
  const int* index = sleep_filter ? d->body_awake_ind : NULL;
  int nbody = sleep_filter ? d->nbody_awake : m->nbody;
  sim_integratePosInd(m, d->qpos, qvel ? qvel : d->qvel, m->opt.timestep, index, nbody);

  // advance time
  d->time += m->opt.timestep;

  // advance plugin states
  if (m->nplugin) {
    const int nslot = sim_plugin_pluginCount();
    for (int i = 0; i < m->nplugin; ++i) {
      const int slot = m->plugin[i];
      const SIM_pPlugin* plugin = sim_plugin_getPluginAtSlotUnsafe(slot, nslot);
      if (!plugin) {
        SIM_ERROR("invalid plugin slot: %d", slot);
      }
      if (plugin->advance) {
        plugin->advance(m, d, i);
      }
    }
  }

  // save qacc for next step warmstart
  sim_math_copy(d->qacc_warmstart, d->qacc, m->nv);
}

// Euler integrator, semi-implicit in velocity, possibly skipping factorisation
void sim_EulerSkip(const sim_model_t* m, sim_data_t* d, int skipfactor) {
  TM_START;
  sim_markStack(d);
  sim_scalar_t* qfrc = SIM_STACK_ALLOC(d, m->nv, sim_scalar_t);
  sim_scalar_t* qacc = SIM_STACK_ALLOC(d, m->nv, sim_scalar_t);

  // sleep filtering
  int sleep_filter = SIM_ENABLED(SIM_ENBL_SLEEP) && d->nv_awake < m->nv;
  int nv = sleep_filter ? d->nv_awake : m->nv;
  const int* dof_awake_ind = sleep_filter ? d->dof_awake_ind : NULL;

  // check for dof damping if disable flag is not set
  int dof_damping = 0;
  if (!SIM_DISABLED(SIM_DSBL_EULERDAMP) && !SIM_DISABLED(SIM_DSBL_DAMPER)) {
    for (int v=0; v < nv; v++) {
      int i = sleep_filter ? dof_awake_ind[v] : v;
      if (m->dof_damping[i] > 0) {
        dof_damping = 1;
        break;
      }
    }
  }

  // no damping or disabled: explicit velocity integration
  if (!dof_damping) {
    if (sleep_filter) {
      sim_math_copyInd(qacc, d->qacc, dof_awake_ind, nv);
    } else {
      sim_math_copy(qacc, d->qacc, nv);
    }
  }

  // damping: integrate implicitly
  else {
    if (!skipfactor) {
      // qH = M
      if (sleep_filter) {
        sim_math_copySparse(d->qH, d->M, m->M_rownnz, m->M_rowadr, dof_awake_ind, d->nv_awake);
      } else {
        sim_math_copy(d->qH, d->M, m->nC);
      }

      // qH += h*diag(B)
      for (int v=0; v < nv; v++) {
        int i = sleep_filter ? dof_awake_ind[v] : v;
        d->qH[m->M_rowadr[i] + m->M_rownnz[i] - 1] += m->opt.timestep * m->dof_damping[i];
      }

      // factorize in-place
      sim_factorI(d->qH, d->qHDiagInv, nv, m->M_rownnz, m->M_rowadr, m->M_colind, dof_awake_ind);
    }

    // solve
    if (sleep_filter) {
      sim_math_addInd(qfrc, d->qfrc_smooth, d->qfrc_constraint, dof_awake_ind, nv);
      sim_math_copyInd(qacc, qfrc, dof_awake_ind, nv);
    } else {
      sim_math_add(qfrc, d->qfrc_smooth, d->qfrc_constraint, nv);
      sim_math_copy(qacc, qfrc, nv);
    }
    sim_solveLD(qacc, d->qH, d->qHDiagInv, nv, 1,
               m->M_rownnz, m->M_rowadr, m->M_colind, dof_awake_ind);
  }

  // advance state and time
  sim_advance(m, d, d->act_dot, qacc, NULL);

  sim_freeStack(d);

  TM_END(SIM_TIMER_ADVANCE);
}


// Euler integrator, semi-implicit in velocity
void sim_Euler(const sim_model_t* m, sim_data_t* d) {
  sim_EulerSkip(m, d, 0);
}


// RK4 tableau
const sim_scalar_t RK4_A[9] = {
  0.5,    0,      0,
  0,      0.5,    0,
  0,      0,      1
};

const sim_scalar_t RK4_B[4] = {
  1.0/6.0, 1.0/3.0, 1.0/3.0, 1.0/6.0
};


// Runge Kutta explicit order-N integrator
//  (A,B) is the tableau, C is set to row_sum(A)
void sim_RungeKutta(const sim_model_t* m, sim_data_t* d, int N) {
  int nv = m->nv, nq = m->nq, na = m->na;
  sim_scalar_t h = m->opt.timestep, time = d->time;
  sim_scalar_t C[9], T[9], *X[10], *F[10], *dX;
  const sim_scalar_t* A = (N == 4 ? RK4_A : 0);
  const sim_scalar_t* B = (N == 4 ? RK4_B : 0);

  // check order
  if (!A) {
    SIM_ERROR("supported RK orders: N=4");
  }

  // allocate space for intermediate solutions
  sim_markStack(d);
  dX = SIM_STACK_ALLOC(d, 2*nv+na, sim_scalar_t);
  for (int i=0; i < N; i++) {
    X[i] = SIM_STACK_ALLOC(d, nq+nv+na, sim_scalar_t);
    F[i] = SIM_STACK_ALLOC(d, nv+na, sim_scalar_t);
  }

  // precompute C and T;  C,T,A have size (N-1)
  for (int i=1; i < N; i++) {
    // C(i) = sum_j A(i,j)
    C[i-1] = 0;
    for (int j=0; j < i; j++) {
      C[i-1] += A[(i-1)*(N-1)+j];
    }

    // compute T
    T[i-1] = d->time + C[i-1]*h;
  }

  // init X[0], F[0]; sim_forward() was already called
  sim_math_copy(X[0], d->qpos, nq);
  sim_math_copy(X[0]+nq, d->qvel, nv);
  sim_math_copy(F[0], d->qacc, nv);
  if (na) {
    sim_math_copy(X[0]+nq+nv, d->act, na);
    sim_math_copy(F[0]+nv, d->act_dot, na);
  }

  // compute the remaining X[i], F[i]
  for (int i=1; i < N; i++) {
    // compute dX
    sim_math_zero(dX, 2*nv+na);
    for (int j=0; j < i; j++) {
      sim_math_addToScl(dX, X[j]+nq, A[(i-1)*(N-1)+j], nv);
      sim_math_addToScl(dX+nv, F[j], A[(i-1)*(N-1)+j], nv+na);
    }

    // compute X[i] = X[0] '+' dX
    sim_math_copy(X[i], X[0], nq+nv+na);
    sim_integratePos(m, X[i], dX, h);
    sim_math_addToScl(X[i]+nq, dX+nv, h, nv+na);

    // set X[i], T[i-1] in sim_data_t
    sim_math_copy(d->qpos, X[i], nq);
    sim_math_copy(d->qvel, X[i]+nq, nv);
    if (na) {
      sim_math_copy(d->act, X[i]+nq+nv, na);
    }
    d->time = T[i-1];

    // evaluate F[i]
    sim_forwardSkip(m, d, SIM_STAGE_NONE, 1);  // 1: do not recompute sensors and energy
    sim_math_copy(F[i], d->qacc, nv);
    if (na) {
      sim_math_copy(F[i]+nv, d->act_dot, na);
    }
  }

  // compute dX for final update (using B instead of A)
  sim_math_zero(dX, 2*nv+na);
  for (int j=0; j < N; j++) {
    sim_math_addToScl(dX, X[j]+nq, B[j], nv);
    sim_math_addToScl(dX+nv, F[j], B[j], nv+na);
  }

  // reset state and time
  d->time = time;
  sim_math_copy(d->qpos, X[0], nq);
  sim_math_copy(d->qvel, X[0]+nq, nv);
  sim_math_copy(d->act, X[0]+nq+nv, na);

  // advance state and time
  sim_advance(m, d, dX+2*nv, dX+nv, dX);

  sim_freeStack(d);
}


// fully implicit in velocity, possibly skipping factorization
void sim_implicitSkip(const sim_model_t* m, sim_data_t* d, int skipfactor) {
  TM_START;
  int nD = m->nD, nC = m->nC;

  sim_markStack(d);
  sim_scalar_t* qfrc = SIM_STACK_ALLOC(d, m->nv, sim_scalar_t);
  sim_scalar_t* qacc = SIM_STACK_ALLOC(d, m->nv, sim_scalar_t);

  // sleep filtering
  int sleep_filter = SIM_ENABLED(SIM_ENBL_SLEEP) && d->nv_awake < m->nv;
  int nv = sleep_filter ? d->nv_awake : m->nv;
  const int* dof_awake_ind = sleep_filter ? d->dof_awake_ind : NULL;

  // set qfrc = qfrc_smooth + qfrc_constraint
  if (sleep_filter) {
    sim_math_addInd(qfrc, d->qfrc_smooth, d->qfrc_constraint, dof_awake_ind, nv);
  } else {
    sim_math_add(qfrc, d->qfrc_smooth, d->qfrc_constraint, nv);
  }

  // check for flex_interp
  int has_flex_interp = 0;
  for (int f = 0; f < m->nflex; f++) {
    if (m->flex_interp[f]) {
      has_flex_interp = 1;
      break;
    }
  }

  // flex: data structures for reduced dense factorization
  sim_scalar_t* H_flex = NULL;
  int* flex_dof_indices = NULL;
  int nflexdofs = 0;
  int ncoupling = 0;
  sim_scalar_t* coupling_val = NULL;
  int* coupling_row = NULL;
  int* coupling_col = NULL;

  // factorization
  if (!skipfactor) {
    // implicit
    if (m->opt.integrator == SIM_INT_IMPLICIT) {
      // compute analytical derivative qDeriv
      SIM_d_smooth_vel(m, d, /* flg_bias = */ 1);

      // gather qLU <- M (lower to full)
      sim_math_gatherMasked(d->qLU, d->M, m->mapM2D, nD);

      // set qLU = M - dt*qDeriv
      sim_math_addToScl(d->qLU, d->qDeriv, -m->opt.timestep, nD);
    }

    // implicitfast
    else if (m->opt.integrator == SIM_INT_IMPLICITFAST) {
      // compute analytical derivative qDeriv; skip rne derivative
      SIM_d_smooth_vel(m, d, /* flg_bias = */ 0);

      // modified mass matrix: gather qH <- qDeriv (full to lower)
      sim_math_gather(d->qH, d->qDeriv, m->mapD2M, nC);

      // set qH = M - dt*qDeriv
      sim_math_addScl(d->qH, d->M, d->qH, -m->opt.timestep, nC);
    } else {
      SIM_ERROR("integrator must be implicit or implicitfast");
    }

    // flex: reduced dense factorization
    if (has_flex_interp && !sleep_filter) {
      // temporary allocations for body chain
      int* chain_dofs = SIM_STACK_ALLOC(d, nv, int);
      int* seen_dof = SIM_STACK_ALLOC(d, nv, int);
      sim_math_fillInt(seen_dof, 0, nv);

      // identify flex DOFs
      // For pinned nodes (body_dofnum==0): use bodyChain to include parent DOFs
      // For regular flex nodes: use body_dofadr for one-way coupling
      for (int f=0; f < m->nflex; f++) {
        if (m->flex_interp[f]) {
          int nodenum = m->flex_nodenum[f];
          int nodeadr = m->flex_nodeadr[f];
          for (int n=0; n < nodenum; n++) {
            int b = m->flex_nodebodyid[nodeadr + n];
            int chain_nnz;
            if (m->body_dofnum[b] == 0) {
              // Pinned node: use bodyChain to get parent DOFs
              chain_nnz = sim_bodyChain(m, b, chain_dofs);
            } else {
              // Regular flex node: use body's own DOFs only
              chain_nnz = m->body_dofnum[b];
              for (int j = 0; j < chain_nnz; j++) {
                chain_dofs[j] = m->body_dofadr[b] + j;
              }
            }
            for (int i=0; i < chain_nnz; i++) {
              int dof = chain_dofs[i];
              if (!seen_dof[dof]) {
                seen_dof[dof] = 1;
                nflexdofs++;
              }
            }
          }
        }
      }

      // allocations
      if (nflexdofs > 0) {
        flex_dof_indices = SIM_STACK_ALLOC(d, nflexdofs, int);
        int* global2local = SIM_STACK_ALLOC(d, nv, int);
        sim_math_fillInt(global2local, -1, nv);

        // collect unique DOFs in order
        int cnt = 0;
        sim_math_fillInt(seen_dof, 0, nv);
        for (int f=0; f < m->nflex; f++) {
          if (m->flex_interp[f]) {
            int nodenum = m->flex_nodenum[f];
            int nodeadr = m->flex_nodeadr[f];
            for (int n=0; n < nodenum; n++) {
              int b = m->flex_nodebodyid[nodeadr + n];
              int chain_nnz;
              if (m->body_dofnum[b] == 0) {
                // Pinned node: use bodyChain to get parent DOFs
                chain_nnz = sim_bodyChain(m, b, chain_dofs);
              } else {
                // Regular flex node: use body's own DOFs only
                chain_nnz = m->body_dofnum[b];
                for (int j = 0; j < chain_nnz; j++) {
                  chain_dofs[j] = m->body_dofadr[b] + j;
                }
              }
              for (int i=0; i < chain_nnz; i++) {
                int dof = chain_dofs[i];
                if (!seen_dof[dof]) {
                  seen_dof[dof] = 1;
                  flex_dof_indices[cnt] = dof;
                  global2local[dof] = cnt;
                  cnt++;
                }
              }
            }
          }
        }

        const int* rownnz = (m->opt.integrator == SIM_INT_IMPLICIT) ? m->D_rownnz : m->M_rownnz;
        const int* rowadr = (m->opt.integrator == SIM_INT_IMPLICIT) ? m->D_rowadr : m->M_rowadr;
        const int* colind = (m->opt.integrator == SIM_INT_IMPLICIT) ? m->D_colind : m->M_colind;
        const sim_scalar_t* source = (m->opt.integrator == SIM_INT_IMPLICIT) ? d->qLU : d->qH;

        // count coupling terms (off-diagonal: flex row, non-flex col)
        for (int i=0; i < nflexdofs; i++) {
          int row = flex_dof_indices[i];
          int start = rowadr[row];
          int end = start + rownnz[row];
          for (int k=start; k < end; k++) {
            if (global2local[colind[k]] < 0) {
              ncoupling++;
            }
          }
        }

        // allocate coupling storage
        if (ncoupling > 0) {
          coupling_val = SIM_STACK_ALLOC(d, ncoupling, sim_scalar_t);
          coupling_row = SIM_STACK_ALLOC(d, ncoupling, int);
          coupling_col = SIM_STACK_ALLOC(d, ncoupling, int);
        }

        // build H_flex (dense) from qLU (implicit) or qH (implicitfast)
        H_flex = SIM_STACK_ALLOC(d, nflexdofs*nflexdofs, sim_scalar_t);
        sim_math_zero(H_flex, nflexdofs*nflexdofs);

        int coup_cnt = 0;
        for (int i=0; i < nflexdofs; i++) {
          int row = flex_dof_indices[i];
          int start = rowadr[row];
          int end = start + rownnz[row];
          for (int k=start; k < end; k++) {
            int col = colind[k];
            int local_j = global2local[col];
            if (local_j >= 0) {
              H_flex[i*nflexdofs + local_j] = source[k];
            } else if (coup_cnt < ncoupling) {
              coupling_val[coup_cnt] = source[k];
              coupling_row[coup_cnt] = i;  // local flex index
              coupling_col[coup_cnt] = col;  // global parent index
              coup_cnt++;
            }
          }
        }

        // add stiffness to H_flex
        sim_scalar_t h = m->opt.timestep;
        SIM_d_flexInterp_addH(m, d, H_flex, flex_dof_indices, nflexdofs, h);

        // factor H_flex
        sim_math_cholFactor(H_flex, nflexdofs, SIM_MINVAL);
      }
    }

    // standard factorization (implicit / implicitfast)
    if (m->opt.integrator == SIM_INT_IMPLICIT) {
      int* scratch = SIM_STACK_ALLOC(d, nv, int);
      sim_math_factorLUSparse(d->qLU, nv, scratch, m->D_rownnz, m->D_rowadr, m->D_colind, dof_awake_ind);
    } else {
      sim_factorI(d->qH, d->qHDiagInv, nv, m->M_rownnz, m->M_rowadr, m->M_colind, dof_awake_ind);
    }
  }

  // solve
  // standard sparse solve
  if (m->opt.integrator == SIM_INT_IMPLICIT) {
    sim_math_solveLUSparse(qacc, d->qLU, qfrc, nv, m->D_rownnz, m->D_rowadr, m->D_diag, m->D_colind,
                      dof_awake_ind);
  } else {
    // implicitfast
    if (sleep_filter) {
      sim_math_copyInd(qacc, qfrc, dof_awake_ind, nv);
    } else {
      sim_math_copy(qacc, qfrc, nv);
    }
    sim_solveLD(qacc, d->qH, d->qHDiagInv, nv, 1, m->M_rownnz, m->M_rowadr, m->M_colind, dof_awake_ind);
  }

  // flex: reduced dense solve
  if (H_flex) {
    // compute qfrc_flex
    sim_scalar_t* qfrc_flex = SIM_STACK_ALLOC(d, nflexdofs, sim_scalar_t);
    sim_scalar_t* res = SIM_STACK_ALLOC(d, nv, sim_scalar_t);

    sim_scalar_t h = m->opt.timestep;
    sim_scalar_t damp = (m->nflex > 0 && m->flex_damping) ? m->flex_damping[0] : 0;
    sim_scalar_t scl = h * h + h * damp;
    sim_scalar_t factor = (scl > SIM_MINVAL) ? (h/scl) : 0;

    // velocity correction: -h * K * v
    sim_math_zero(res, nv);
    SIM_d_flexInterp_mulKD(m, d, res, d->qvel, h);  // returns -scl * K * v

    for (int i=0; i < nflexdofs; i++) {
      int global_dof = flex_dof_indices[i];
      qfrc_flex[i] = qfrc[global_dof] + res[global_dof] * factor;
    }

    // apply coupling correction: qfrc_flex -= H_coupling * qacc_parent
    if (ncoupling > 0) {
      for (int k=0; k < ncoupling; k++) {
        qfrc_flex[coupling_row[k]] -= coupling_val[k] * qacc[coupling_col[k]];
      }
    }

    // solve H_flex * qacc_flex = qfrc_flex
    // reuse qfrc_flex as result buffer (qacc_flex)
    sim_math_cholSolve(qfrc_flex, H_flex, qfrc_flex, nflexdofs);

    // overwrite flex DOFs with reduced dense solution
    sim_math_scatter(qacc, qfrc_flex, flex_dof_indices, nflexdofs);
  }

  // advance state and time
  sim_advance(m, d, d->act_dot, qacc, NULL);

  sim_freeStack(d);

  TM_END(SIM_TIMER_ADVANCE);
}


// fully implicit in velocity
void sim_implicit(const sim_model_t* m, sim_data_t* d) {
  sim_implicitSkip(m, d, 0);
}


//-------------------------- top-level API ---------------------------------------------------------

// forward dynamics with skip; skipstage is SIM_tStage
void sim_forwardSkip(const sim_model_t* m, sim_data_t* d, int skipstage, int skipsensor) {
  TM_START;

  // position-dependent
  if (skipstage < SIM_STAGE_POS) {
    sim_fwdPosition(m, d);

    if (!skipsensor) {
      sim_sensorPos(m, d);
    }

    if (!d->flg_energypos) {
      if (SIM_ENABLED(SIM_ENBL_ENERGY)) {
        sim_energyPos(m, d);
      } else {
        d->energy[0] = d->energy[1] = 0;
      }
    }
  }

  // velocity-dependent
  if (skipstage < SIM_STAGE_VEL) {
    sim_fwdVelocity(m, d);

    if (!skipsensor) {
      sim_sensorVel(m, d);
    }

    if (SIM_ENABLED(SIM_ENBL_ENERGY) && !d->flg_energyvel) {
      sim_energyVel(m, d);
    }
  }

  // acceleration-dependent
  if (SIM_cb_control && !SIM_DISABLED(SIM_DSBL_ACTUATION)) {
    SIM_cb_control(m, d);
  }

  sim_fwdActuation(m, d);
  sim_fwdAcceleration(m, d);
  sim_fwdConstraint(m, d);
  if (!skipsensor) {
    d->flg_rnepost = 0;  // clear flag for lazy evaluation
    sim_sensorAcc(m, d);
  }

  TM_END(SIM_TIMER_FORWARD);
}


// forward dynamics
void sim_forward(const sim_model_t* m, sim_data_t* d) {
  sim_forwardSkip(m, d, SIM_STAGE_NONE, 0);
}


// advance simulation using control callback
void sim_step(const sim_model_t* m, sim_data_t* d) {
  TM_START;

  // common to all integrators
  sim_checkPos(m, d);
  sim_checkVel(m, d);
  sim_forward(m, d);
  sim_checkAcc(m, d);

  // compare forward and inverse solutions if enabled
  if (SIM_ENABLED(SIM_ENBL_FWDINV)) {
    sim_compareFwdInv(m, d);
  }

  // use selected integrator
  switch ((SIM_tIntegrator) m->opt.integrator) {
  case SIM_INT_EULER:
    sim_Euler(m, d);
    break;

  case SIM_INT_RK4:
    sim_RungeKutta(m, d, 4);
    break;

  case SIM_INT_IMPLICIT:
  case SIM_INT_IMPLICITFAST:
    sim_implicit(m, d);
    break;

  default:
    SIM_ERROR("invalid integrator");
  }

  TM_END(SIM_TIMER_STEP);
}


// advance simulation in two phases: before input is set by user
void sim_step1(const sim_model_t* m, sim_data_t* d) {
  TM_START;
  sim_checkPos(m, d);
  sim_checkVel(m, d);
  sim_fwdPosition(m, d);
  sim_sensorPos(m, d);

  if (!d->flg_energypos) {
    if (SIM_ENABLED(SIM_ENBL_ENERGY)) {
      sim_energyPos(m, d);
    } else {
      d->energy[0] = d->energy[1] = 0;
    }
  }

  sim_fwdVelocity(m, d);
  sim_sensorVel(m, d);
  if (SIM_ENABLED(SIM_ENBL_ENERGY) && !d->flg_energyvel) {
    sim_energyVel(m, d);
  }

  if (SIM_cb_control) {
    SIM_cb_control(m, d);
  }
  TM_END(SIM_TIMER_STEP);
}


//   >>>>   user can modify ctrl and q/xfrc_applied between step1 and step2   <<<<


// advance simulation in two phases: after input is set by user
void sim_step2(const sim_model_t* m, sim_data_t* d) {
  TM_START;
  sim_fwdActuation(m, d);
  sim_fwdAcceleration(m, d);
  sim_fwdConstraint(m, d);
  sim_sensorAcc(m, d);
  sim_checkAcc(m, d);

  // compare forward and inverse solutions if enabled
  if (SIM_ENABLED(SIM_ENBL_FWDINV)) {
    sim_compareFwdInv(m, d);
  }

  // integrate with Euler or implicit; RK4 defaults to Euler
  if (m->opt.integrator == SIM_INT_IMPLICIT || m->opt.integrator == SIM_INT_IMPLICITFAST) {
    sim_implicit(m, d);
  } else {
    sim_Euler(m, d);
  }

  d->timer[SIM_TIMER_STEP].number--;
  TM_END(SIM_TIMER_STEP);
}

