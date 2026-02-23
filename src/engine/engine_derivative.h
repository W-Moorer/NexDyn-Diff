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

#ifndef SIMCORE_SRC_ENGINE_ENGINE_DERIVATIVE_H_
#define SIMCORE_SRC_ENGINE_ENGINE_DERIVATIVE_H_

#include <simcore/SIM_data.h>
#include <simcore/SIM_export.h>
#include <simcore/SIM_model.h>

#ifdef __cplusplus
extern "C" {
#endif

// derivatives of sim_math_subQuat w.r.t inputs
SIM_API void SIM_d_subQuat(const sim_scalar_t qa[4], const sim_scalar_t qb[4], sim_scalar_t Da[9], sim_scalar_t Db[9]);

// derivatives of sim_math_quatIntegrate w.r.t inputs
SIM_API void SIM_d_quatIntegrate(const sim_scalar_t vel[3], sim_scalar_t scale,
                             sim_scalar_t Dquat[9], sim_scalar_t Dvel[9], sim_scalar_t Dscale[3]);

// analytical derivative of smooth forces w.r.t velocities:
//   d->qDeriv = d (qfrc_actuator + qfrc_passive - [qfrc_bias]) / d qvel
SIM_API void SIM_d_smooth_vel(const sim_model_t* m, sim_data_t* d, int flg_bias);

// add (d qfrc_actuator / d qvel) to qDeriv
SIM_API void SIM_d_actuator_vel(const sim_model_t* m, sim_data_t* d);

// add (d qfrc_passive / d qvel) to qDeriv
SIM_API void SIM_d_passive_vel(const sim_model_t* m, sim_data_t* d);

// subtract (d qfrc_bias / d qvel) from qDeriv (dense version)
SIM_API void SIM_d_rne_vel_dense(const sim_model_t* m, sim_data_t* d);

// derivative of flex_interp generalized force w.r.t position: res = (d qfrc_flexinterp / d qpos) * vec
//  res and vec are vectors of size m->nv
SIM_API void SIM_d_flexInterp_mulKD(const sim_model_t* m, sim_data_t* d, sim_scalar_t* res, const sim_scalar_t* vec, sim_scalar_t h);

// assemble flex stiffness matrix H_flex: H += h*h*K + h*D
//  H is a dense matrix of size ndof x ndof, dof_indices maps local rows/cols to global DOFs
SIM_API void SIM_d_flexInterp_addH(const sim_model_t* m, sim_data_t* d, sim_scalar_t* H, const int* dof_indices, int ndof, sim_scalar_t h);


#ifdef __cplusplus
}
#endif

#endif  // SIMCORE_SRC_ENGINE_ENGINE_DERIVATIVE_H_
